cmake_minimum_required(VERSION 3.8)
project(joint_positions_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- IDL files ----
set(msg_files "msg/SetJointDegrees.msg")
set(srv_files "srv/GetJointDegrees.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ---- node executable ----
add_executable(joint_positions_driver src/joint_positions_node.cpp)
ament_target_dependencies(joint_positions_driver rclcpp dynamixel_sdk)

# link the generated interfaces
rosidl_target_interfaces(joint_positions_driver
	${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
	TARGETS joint_positions_driver
	DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
