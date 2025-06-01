cmake_minimum_required(VERSION 3.8)
project(ball_predictor_node)

# ── Dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_common REQUIRED)

# ── Executable ───────────────────────────────────────────────────────────
add_executable(ball_predictor_node
  src/ball_predictor_node.cpp
)
target_include_directories(ball_predictor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(ball_predictor_node
	rclcpp sensor_msgs geometry_msgs cv_bridge vision_common
)
target_link_libraries(ball_predictor_node
  ${OpenCV_LIBRARIES}
)

# ── Install rules ─────────────────────────────
install(TARGETS ball_predictor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#  ↓↓↓  this line puts the package marker in the ament index
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



