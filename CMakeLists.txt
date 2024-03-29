cmake_minimum_required(VERSION 3.5)
project(goal_plotter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(goal_plotter_messages REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(common_interfaces REQUIRED)


ament_python_install_package(${PROJECT_NAME})



#add executables
add_executable(main_plot src/main_plot.cpp)
ament_target_dependencies(main_plot rclcpp geometry_msgs visualization_msgs goal_plotter_messages)
target_include_directories(main_plot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


add_executable(get_goal src/get_goal.cpp)
ament_target_dependencies(get_goal rclcpp geometry_msgs goal_plotter_messages)
target_include_directories(get_goal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  launch
  goal_json
  rviz
  param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  main_plot
  get_goal
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/goal_gui.py
  scripts/lv_goal_manager.py
  scripts/lv_goal_converter.py
  scripts/footprint_collision_checker.py
  scripts/costmap_2d.py
  scripts/footprint_generator.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment tshe line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
