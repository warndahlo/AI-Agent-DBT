cmake_minimum_required(VERSION 3.8)
project(ai_agent_dbt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# --- Original Node ---
add_executable(rviz_hallway_publisher src/rviz_hallway_publisher.cpp)
target_include_directories(rviz_hallway_publisher
   PRIVATE
    include/rviz_hallway_publisher
    ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(rviz_hallway_publisher rclcpp visualization_msgs Eigen3 ament_index_cpp)
install(
  TARGETS rviz_hallway_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install shared assets directories
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# --- Python Executables Installation Pipeline ---
# CRITICAL FIX: Registered your baseline and hybrid behavior tree navigation scripts
install(PROGRAMS
  scripts/BT_Ollama_waypoint_navigator.py
  scripts/BT_waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
