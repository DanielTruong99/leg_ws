add_library(FFControllerFSM STATIC FFControllerFSM.h FFControllerFSM.cpp)
target_include_directories(FFControllerFSM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
target_link_libraries(FFControllerFSM ActiveObject FFControlLaw)
ament_target_dependencies(FFControllerFSM rclcpp std_msgs sensor_msgs Eigen3)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
