#include "FeedForwardController.h"

namespace ff_controller
{
    FeedForwardControllerNode::FeedForwardControllerNode(): 
        rclcpp::Node("feed_forward_control_node")
    {
        _timer = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&FeedForwardControllerNode::handleTimerEvt, this));
        _desired_torque_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_torque", 10);
        _motor_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&FeedForwardControllerNode::handleMotorStateAvailableEvt, this, std::placeholders::_1));
        _estimated_robot_parameter_sub = this->create_subscription<leg_msgs::msg::EstimatedRobotParameter>("estimated_robot_parameters", 10, std::bind(&FeedForwardControllerNode::handleRobotParameterAvailableEvt, this, std::placeholders::_1));
        _robot_operation_state_sub = this->create_subscription<leg_msgs::msg::RobotOperationState>("estimated_robot_parameters", 10, std::bind(&FeedForwardControllerNode::handleRobotStateAvailableEvt, this, std::placeholders::_1));
        _desired_motor_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("desired_joint_states", 10, std::bind(&FeedForwardControllerNode::handleDesiredMotorStateAvailableEvt, this, std::placeholders::_1));

        _fsm = ffc_fsm::FFControllerFsm();
        _fsm._node_ptr = this;
        _fsm._desired_torque_pub = _desired_torque_pub;
    }

    void FeedForwardControllerNode::handleTimerEvt()
    {
        _fsm.loopEvent();
    }

    void FeedForwardControllerNode::handleRobotParameterAvailableEvt(const leg_msgs::msg::EstimatedRobotParameter::SharedPtr robot_parameter_msg)
    {
        _fsm.updateRobotParameter(robot_parameter_msg);
    }

    void FeedForwardControllerNode::handleMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg)
    {
        _fsm.updateMotorState(motor_state_msg);
    }

    void FeedForwardControllerNode::handleDesiredMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg)
    {
        _fsm.updateDesiredMotorState(desired_motor_state_msg);
    }

    void FeedForwardControllerNode::handleRobotStateAvailableEvt(const leg_msgs::msg::RobotOperationState::SharedPtr robot_state_msg)
    {
        
    }
}