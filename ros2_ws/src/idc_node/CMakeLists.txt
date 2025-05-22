cmake_minimum_required(VERSION 3.8)
project(idc_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(inverse_kinematics_node REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPlotData.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(inverse_dynamics_node src/inverse_dynamics_node.cpp)

ament_target_dependencies(inverse_dynamics_node
  rclcpp
  dynamixel_sdk
  inverse_kinematics_node
  geometry_msgs
)

rosidl_target_interfaces(inverse_dynamics_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS inverse_dynamics_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
