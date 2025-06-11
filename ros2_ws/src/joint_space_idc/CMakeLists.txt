cmake_minimum_required(VERSION 3.10)
project(joint_space_idc LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp          REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(dynamixel_sdk   REQUIRED)
find_package(control_common  REQUIRED)        #  your kinematics/dynamics helpers
find_package(Eigen3          REQUIRED NO_MODULE)

add_executable(joint_space_idc  src/joint_space_idc.cpp)

target_include_directories(joint_space_idc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(joint_space_idc
  rclcpp
  controller_msgs
  dynamixel_sdk
  control_common
)

target_link_libraries(joint_space_idc
  Eigen3::Eigen
)

include_directories(
  include
)
install(TARGETS joint_space_idc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()

