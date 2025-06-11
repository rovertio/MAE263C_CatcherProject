cmake_minimum_required(VERSION 3.8)
project(controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ball_detector REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED) 


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPlotData.msg"
  DEPENDENCIES builtin_interfaces 
)

# ament_export_dependencies(rosidl_default_runtime)

# add_executable(inverse_dynamics_node src/inverse_dynamics_node.cpp)
# ament_target_dependencies(inverse_dynamics_node
#   rclcpp
#   dynamixel_sdk
#   geometry_msgs
#   ball_detector
#   message_filters
#   sensor_msgs
# )

# rosidl_target_interfaces(inverse_dynamics_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

add_executable(simple_pid src/pwm_position_node.cpp)
ament_target_dependencies(simple_pid
  rclcpp
  dynamixel_sdk
  geometry_msgs
  ball_detector
  message_filters
  sensor_msgs
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
        # inverse_dynamics_node
        simple_pid 
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
  # target_link_libraries(inverse_dynamics_node "${cpp_typesupport_target}") 
  target_link_libraries(simple_pid "${cpp_typesupport_target}") 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
