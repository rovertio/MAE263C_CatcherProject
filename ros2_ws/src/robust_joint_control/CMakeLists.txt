cmake_minimum_required(VERSION 3.10)
project(robust_joint_control LANGUAGES CXX)

# ─── Build tools ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Build / exec dependencies ─────────────────────────────────────────
find_package(rclcpp          REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(dynamixel_sdk   REQUIRED)
find_package(control_common  REQUIRED)
find_package(Eigen3          REQUIRED NO_MODULE)   # via eigen3_cmake_module

# ─── Node executable ──────────────────────────────────────────────────
add_executable(robust_joint_control src/robust_joint_control.cpp)

target_include_directories(robust_joint_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robust_joint_control
  rclcpp controller_msgs dynamixel_sdk control_common
)

target_link_libraries(robust_joint_control Eigen3::Eigen)

# ─── Install rules ────────────────────────────────────────────────────
install(TARGETS robust_joint_control
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()

