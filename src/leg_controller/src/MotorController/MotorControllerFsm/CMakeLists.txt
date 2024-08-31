add_library(MotorControllerFsm STATIC MotorControllerFsm.h MotorControllerFsm.cpp)
target_include_directories(MotorControllerFsm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
target_link_libraries(MotorControllerFsm ActiveObject ControlLaw)
ament_target_dependencies(MotorControllerFsm rclcpp Eigen3 sensor_msgs)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
