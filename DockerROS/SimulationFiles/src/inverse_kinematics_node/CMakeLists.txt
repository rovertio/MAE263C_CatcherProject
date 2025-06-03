cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics_node)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)                # ← add
find_package(control_common REQUIRED)
find_package(controller_msgs REQUIRED)

add_executable(ik_driver src/inverse_kinematics_node.cpp)

ament_target_dependencies(ik_driver          # only once, after add_executable
  rclcpp dynamixel_sdk Eigen3
  control_common controller_msgs)

install(TARGETS ik_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

