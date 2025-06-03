cmake_minimum_required(VERSION 3.8)
project(controller_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)   # ➊ add this line

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetXY.msg"
  "msg/JointPlotData.msg"
  DEPENDENCIES builtin_interfaces           # ➋ this now resolves
)

ament_package()

