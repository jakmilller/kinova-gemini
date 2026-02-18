cmake_minimum_required(VERSION 3.8)
project(ros2_interfaces)

# Find necessary ROS 2 build tools and generators
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find dependencies whose types are used in your .msg/.srv files
# These should correspond to <depend> tags in your package.xml
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# Add other find_package calls here if your interfaces use types from them (e.g., geometry_msgs)

# Specify the message files
set(msg_files
  "msg/RobotState.msg"
)

set(action_files
  "action/GripperCommand.action"
  "action/MoveToJoints.action"
  "action/MoveToPose.action"
)

# Generate messages and services
# The DEPENDENCIES listed here should be the packages whose types are directly
# included in your .msg or .srv definitions.
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs builtin_interfaces# Add other direct dependencies like geometry_msgs if needed
)

# ament_auto_package handles a lot of the boilerplate for exporting
# dependencies and interface targets.
ament_auto_package()