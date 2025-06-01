cmake_minimum_required(VERSION 3.10)
project(robust_operational_control LANGUAGES CXX)

# ─── Build tool & flags ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ──────────────────────────────────────────────────────
find_package(rclcpp          REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(dynamixel_sdk   REQUIRED)
find_package(control_common  REQUIRED)
find_package(Eigen3          REQUIRED NO_MODULE)   # via eigen3_cmake_module

# ─── Executable ───────────────────────────────────────────────────────
add_executable(robust_operational_control
  src/robust_operational_control.cpp)

target_include_directories(robust_operational_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robust_operational_control
  rclcpp controller_msgs dynamixel_sdk control_common)

target_link_libraries(robust_operational_control
  Eigen3::Eigen)

# ─── Install ──────────────────────────────────────────────────────────
install(TARGETS robust_operational_control
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_package()

