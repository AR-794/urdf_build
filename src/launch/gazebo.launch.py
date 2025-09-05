from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('small_ros'),
        'urdf',
        'Small_ROS.urdf'
    ])

    return LaunchDescription([
        # Launch Gazebo Classic with ROS plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn your robot URDF in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'Small_ROS'],
            output='screen'
        ),

        # Static TF publisher (base_link -> base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
    ])
