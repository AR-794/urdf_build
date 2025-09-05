cmake_minimum_required(VERSION 3.8)
project(small_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
install(
  PROGRAMS
    Small_ROS/odom_raw.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install directories
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()