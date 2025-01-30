cmake_minimum_required(VERSION 3.8)
project(task_2_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message and service interfaces
rosidl_generate_interfaces(task_2_interfaces
  "msg/JointData.msg"
  "srv/JointState.srv"
  DEPENDENCIES geometry_msgs
)

# Export package
ament_package()