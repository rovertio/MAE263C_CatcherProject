cmake_minimum_required(VERSION 3.8)
project(control_common)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)          # already there

# ── interface target that carries the include path ────────────────
add_library(control_common INTERFACE)
target_include_directories(control_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})              # ★ add Eigen’s headers

install(DIRECTORY include/ DESTINATION include)
install(TARGETS control_common EXPORT export_control_common)

# export include dirs to downstream packages
ament_export_targets(export_control_common)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})

ament_package()

