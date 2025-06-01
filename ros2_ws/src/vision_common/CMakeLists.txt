cmake_minimum_required(VERSION 3.8)
project(vision_common)

find_package(ament_cmake REQUIRED)

# ── Build-time linters (optional) ───────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ── Install headers ────────────────────────────────────────────────
install(DIRECTORY include/
        DESTINATION include)

ament_package()           # ← exactly once, at the very end

