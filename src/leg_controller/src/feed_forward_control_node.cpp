#include "leg_controller/feed_forward_control_node.h"

using namespace ff_controller;

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_handle = std::make_shared<FeedForwardControllerNode>();
    rclcpp::spin(node_handle);
    rclcpp::shutdown();
    return 0;
}