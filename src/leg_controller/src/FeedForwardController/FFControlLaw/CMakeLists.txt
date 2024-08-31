add_library(FFControlLaw STATIC FFControlLaw.h FFControlLaw.cpp)
target_include_directories(FFControlLaw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(MotorController "${cpp_typesupport_target}")
# target_link_libraries(FFControlLaw ActiveObject)
ament_target_dependencies(FFControlLaw rclcpp std_msgs sensor_msgs Eigen3 leg_msgs)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
