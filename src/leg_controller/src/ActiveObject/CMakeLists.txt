add_library(ActiveObject STATIC ActiveObject.h ActiveObject.cpp)
target_include_directories(ActiveObject PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(CtrlInterface "${cpp_typesupport_target}")
# ament_target_dependencies(CtrlInterface rclcpp geometry_msgs)
# message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
