cmake_minimum_required(VERSION 3.8)
project(ball_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)   # ➊ add this line

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetXY.msg"
  DEPENDENCIES builtin_interfaces 
)
# ament_export_dependencies(rosidl_default_runtime)

add_executable(depth_node src/ball_predictor_node.cpp)
ament_target_dependencies(
   depth_node
   sensor_msgs
   rclcpp
   geometry_msgs
   cv_bridge
   message_filters
)
target_link_libraries(depth_node
  ${OpenCV_LIBRARIES}
)

add_executable(pixel_to_xy_bridge src/pixel_to_xy_bridge.cpp)
ament_target_dependencies(
    pixel_to_xy_bridge
	  rclcpp 
    geometry_msgs 
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  # center_node
  depth_node
  pixel_to_xy_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(pixel_to_xy_bridge "${cpp_typesupport_target}") 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
