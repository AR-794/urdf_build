import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import serial
import threading
import time
import math
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DiffDriveSerial(Node):
    def __init__(self):
        super().__init__('diff_drive_serial')

        # Parameters
        self.wheel_base = 0.3  # distance between left and right wheels
        self.wheel_radius = 0.05
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS2 interfaces
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Open serial
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)

        # Start thread to read MCU
        self.thread = threading.Thread(target=self.read_from_mcu)
        self.thread.daemon = True
        self.thread.start()

        self.get_logger().info("DiffDriveSerial node started")

    def cmd_vel_callback(self, msg: Twist):
        # Convert linear/angular velocity to wheel velocities (m/s)
        v = msg.linear.x
        omega = msg.angular.z

        v_l = v - omega * self.wheel_base / 2
        v_r = v + omega * self.wheel_base / 2

        # Send velocities to MCU via serial (example CSV: "0.1,0.1\n")
        command = f"{v_l:.4f},{v_r:.4f}\n"
        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_from_mcu(self):
        """
        MCU sends data like: encoder_left,encoder_right,imu_yaw
        """
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 3:
                    continue
                enc_l, enc_r, imu_yaw = map(float, parts)

                # Compute odometry
                self.update_odometry(enc_l, enc_r, imu_yaw)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

    def update_odometry(self, enc_l, enc_r, imu_yaw):
        """
        enc_l and enc_r: wheel displacement in meters since last read
        imu_yaw: absolute yaw in radians from IMU
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Distance traveled by each wheel
        dl = enc_l
        dr = enc_r
        dc = (dl + dr) / 2.0
        dtheta = imu_yaw - self.theta  # use IMU yaw

        # Update pose
        self.x += dc * math.cos(self.theta + dtheta/2)
        self.y += dc * math.sin(self.theta + dtheta/2)
        self.theta = imu_yaw

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*q)
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*q)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
