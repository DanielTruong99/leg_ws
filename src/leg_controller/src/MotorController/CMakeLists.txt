add_subdirectory(ControlLaw)
add_subdirectory(MotorControllerFsm)

add_library(MotorController STATIC MotorController.h MotorController.cpp)
target_include_directories(MotorController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
target_link_libraries(MotorController MotorControllerFsm)
ament_target_dependencies(MotorController rclcpp Eigen3 std_msgs sensor_msgs)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
