#include "leg_controller/motor_control_node.h"

using namespace motor_ctrl;

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_handle = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node_handle);
    rclcpp::shutdown();
    return 0;
}