cmake_minimum_required(VERSION 3.8)
project(ball_centroid_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_common REQUIRED)   # header-only constants

# ── Executable ──────────────────────────────────────────────────────
add_executable(ball_detector src/detector.cpp)

target_include_directories(ball_detector PUBLIC
  ${OpenCV_INCLUDE_DIRS})               # (only needed if OpenCV headers live elsewhere)

ament_target_dependencies(ball_detector
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport
  vision_common)                        # ← add it here

target_link_libraries(ball_detector
  ${OpenCV_LIBRARIES})

# ── Install ─────────────────────────────────────────────────────────
install(TARGETS ball_detector
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ── Linters (optional) ──────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)         # skip cpplint if you want
  set(ament_cmake_copyright_FOUND TRUE)       # skip copyright linter
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

