cmake_minimum_required(VERSION 3.8)
project(assembly_factory_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(add_collision_box
  src/add_collision_box.cpp
)

add_executable(arm_motion_test
  src/arm_motion_test.cpp
)

add_executable(pick_place
  src/pick_place.cpp
)
ament_target_dependencies(add_collision_box
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)

ament_target_dependencies(arm_motion_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(pick_place
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)

install(TARGETS
  arm_motion_test
  add_collision_box
  pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)\

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
