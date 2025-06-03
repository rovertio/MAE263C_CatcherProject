cmake_minimum_required(VERSION 3.8)
project(idc_node)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)                # ← add
find_package(control_common REQUIRED)
find_package(controller_msgs REQUIRED)

add_executable(inverse_dynamics_node src/inverse_dynamics_node.cpp)

ament_target_dependencies(inverse_dynamics_node
  rclcpp dynamixel_sdk Eigen3
  control_common controller_msgs)

install(TARGETS inverse_dynamics_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

