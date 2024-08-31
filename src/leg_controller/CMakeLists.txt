cmake_minimum_required(VERSION 3.8)
project(leg_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(leg_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
# find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME} "interfaces/msg/DistanceX.msg")
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
add_subdirectory(src/ActiveObject)

# Build motor control node
add_subdirectory(src/MotorController)

add_executable(low_level_control_node src/motor_control_node.cpp)
target_link_libraries(low_level_control_node MotorController)

# Build feed forward control node
add_subdirectory(src/FeedForwardController)

add_executable(feed_forward_control_node src/feed_forward_control_node.cpp)
target_link_libraries(feed_forward_control_node FeedForwardController)

# Install
install(TARGETS low_level_control_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS feed_forward_control_node DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
