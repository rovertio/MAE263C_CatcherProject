cmake_minimum_required(VERSION 3.8)
project(pwm_position_node)

# 1) find ament and any packages you depend on
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(inverse_kinematics_node REQUIRED)  # for SetXY

# 2) add your executable
add_executable(pwm_position_node src/pwm_position_node.cpp)

# 3) Link against libraries / add include dirs (usually handled by ament_target_dependencies)
ament_target_dependencies(pwm_position_node
  rclcpp
  dynamixel_sdk
  inverse_kinematics_node
)

# 4) (optional) if you have headers in include/, install them too
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME})

# 5) install your node so ros2 run can find it
install(TARGETS pwm_position_node
  DESTINATION lib/${PROJECT_NAME}
)

# 6) export info for downstream packages
ament_package()

