add_subdirectory(FFControllerFSM)
add_subdirectory(FFControlLaw)

add_library(FeedForwardController STATIC FeedForwardController.h FeedForwardController.cpp)
target_include_directories(FeedForwardController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
target_link_libraries(FeedForwardController FFControllerFSM)
ament_target_dependencies(FeedForwardController rclcpp std_msgs leg_msgs sensor_msgs Eigen3)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
