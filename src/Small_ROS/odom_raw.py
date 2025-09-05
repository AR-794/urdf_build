#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial, math, time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import threading

# def euler_to_quaternion(roll, pitch, yaw):
#     """Convert Euler angles to quaternion."""
#     qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
#     qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
#     qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
#     qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
#     return Quaternion(x=qx, y=qy, z=qz, w=qw)

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Robot params
        self.max_linear_speed = 1 #m/s
        self.max_angular_speed = 2.0 #rad/sec
        self.max_pwm = 32665
        self.WHEEL_RADIUS = 0.035   # m
        self.TICKS_PER_REV = 10  # encoder resolution
        self.WHEEL_BASE = 0.17     # m between wheels

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = time.time()

        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info("Robot node started")


    def publish_tf(self, qw, qx, qy, qz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x       # forward/backward
        angular_vel = msg.angular.z     # rotation

        # Differential drive kinematics
        v_left  = linear_vel - (angular_vel * self.WHEEL_BASE / 2.0)
        v_right = linear_vel + (angular_vel * self.WHEEL_BASE / 2.0)

        # Convert velocities to PWM (-max_pwm .. max_pwm)
        pwm_left  = int(max(min(v_left  / self.max_linear_speed * self.max_pwm, self.max_pwm), -self.max_pwm))
        pwm_right = int(max(min(v_right / self.max_linear_speed * self.max_pwm, self.max_pwm), -self.max_pwm))

        # Format command string: e.g., "PWM,L,R\n"
        cmd_str = f"CMD,{pwm_left},{pwm_right}\n"

        # Send over serial
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Failed to send PWM: {e}")

        # Optional: print for debugging
        self.get_logger().info(f"Sent PWM -> Left: {pwm_left}, Right: {pwm_right}")

    def read_serial(self):
        while True:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self.process_line(line)
        
    def process_line(self,line):    
        if not line.startswith("RAW"):
            return

        try:
            parts = line.split(',')
            # Unpack values
            timestamp = int(parts[1])
            left_ticks = int(parts[2])
            right_ticks = int(parts[3])
            qw, qx, qy, qz = map(float, parts[4:8])
            ax, ay, az = map(float, parts[8:11])
            gx, gy, gz = map(float, parts[11:14])
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")
            return

        # ---- Compute wheel displacements ----
        if self.last_left is not None and self.last_right is not None:
            d_left  = (left_ticks  - self.last_left)  * (2 * math.pi * self.WHEEL_RADIUS / self.TICKS_PER_REV)
            d_right = (right_ticks - self.last_right) * (2 * math.pi * self.WHEEL_RADIUS / self.TICKS_PER_REV)
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.WHEEL_BASE

            self.theta += d_theta
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

        self.last_left = left_ticks
        self.last_right = right_ticks

        # ---- Publish Odometry ----
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # odom.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.theta)
        self.odom_pub.publish(odom)

        # ---- Publish IMU ----
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation.x = qx
        imu.orientation.y = qy  
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        self.imu_pub.publish(imu)

        self.publish_tf(qw, qx, qy, qz)

def main(args=None):
    node = None
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()