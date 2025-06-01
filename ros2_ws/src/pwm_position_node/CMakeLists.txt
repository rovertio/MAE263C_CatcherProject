# ~/ros2_ws/src/pwm_position_node/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(pwm_position_node)

add_compile_options(-Wall -Wextra -Wpedantic)

# ── dependencies ───────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_common REQUIRED)
find_package(controller_msgs REQUIRED)

# ── executable ─────────────────────────────────────────────────────
add_executable(pwm_position_node src/pwm_position_node.cpp)

ament_target_dependencies(pwm_position_node
  rclcpp dynamixel_sdk Eigen3
  control_common controller_msgs)

install(TARGETS pwm_position_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

