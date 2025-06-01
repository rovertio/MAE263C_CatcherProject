cmake_minimum_required(VERSION 3.10)
project(adaptive_joint_control LANGUAGES CXX)

# ─── build system ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── dependencies ───────────────────────────────────────────────────────
find_package(rclcpp          REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(dynamixel_sdk   REQUIRED)
find_package(control_common  REQUIRED)
find_package(Eigen3          REQUIRED NO_MODULE)   # provided by eigen3_cmake_module

# ─── node executable ───────────────────────────────────────────────────
add_executable(adaptive_joint_control
  src/adaptive_joint_control.cpp)

target_include_directories(adaptive_joint_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(adaptive_joint_control
  rclcpp controller_msgs dynamixel_sdk control_common)

target_link_libraries(adaptive_joint_control Eigen3::Eigen)

# ─── install ───────────────────────────────────────────────────────────
install(TARGETS adaptive_joint_control
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_package()

