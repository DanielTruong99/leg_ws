add_library(ControlLaw STATIC ControlLaw.h ControlLaw.cpp)
target_include_directories(ControlLaw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
# target_link_libraries(MotorControllerFsm ActiveObject)
ament_target_dependencies(ControlLaw rclcpp Eigen3)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
