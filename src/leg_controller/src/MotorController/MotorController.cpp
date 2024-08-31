#include "MotorController.h"

namespace motor_ctrl
{
    /*Low level control node*/
    MotorControllerNode::MotorControllerNode():
        rclcpp::Node("low_level_control_node")
    {
        /*Initialize _motor_publisher_hub array*/
        for(const auto &motor_config : this->_motor_configs)
        {
            this->_motor_publisher_hub[motor_config.index] = this->create_publisher<std_msgs::msg::Float64>(motor_config.topic_name, 9);
        }

        /*Initialize finite state machine*/
        this->_fsm = motor_ctrl_fsm::MotorControllerFSM();

        /*Initialize timer*/
        this->_timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&MotorControllerNode::handleTimerEvt, this));

        /*Initialize motor states subscribers*/
        this->_motor_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&MotorControllerNode::handleMotorStateAvailableEvt, this, std::placeholders::_1));
        this->_desired_motor_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("desired_joint_states", 10, std::bind(&MotorControllerNode::handleDesiredMotorStateAvailableEvt, this, std::placeholders::_1));

        this->_desired_motor_torque_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("desired_torque", 10, std::bind(&MotorControllerNode::handleDesiredTorqueAvailableEvt, this, std::placeholders::_1));
    
        /*Declare node's parameters*/
        this->declare_parameter<std::vector<double>>("K_P", std::vector<double>({0.0, 0.0, 0.0}));
        this->declare_parameter<std::vector<double>>("K_D", std::vector<double>({0.0, 0.0, 0.0}));
        this->declare_parameter<std::vector<double>>("LOWER_BOUND_TORQUE", std::vector<double>(10, -1000.0));
        this->declare_parameter<std::vector<double>>("UPPER_BOUND_TORQUE", std::vector<double>(10, 1000.0));

        _K_P = this->get_parameter("K_P");
        _K_D = this->get_parameter("K_D");
        _LOWER_BOUND_TORQUE = this->get_parameter("LOWER_BOUND_TORQUE");
        _UPPER_BOUND_TORQUE = this->get_parameter("UPPER_BOUND_TORQUE");

        _fsm.setKP(_K_P);
        _fsm.setKD(_K_D);
        _fsm.setLowerBoundTorque(_LOWER_BOUND_TORQUE);
        _fsm.setUpperBoundTorque(_UPPER_BOUND_TORQUE);
        _parameter_handler = this->add_on_set_parameters_callback(std::bind(&MotorControllerNode::handleParameterChangeEvt, this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult MotorControllerNode::handleParameterChangeEvt(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "K_P") _fsm.setKP(param);
            else if (param.get_name() == "K_D") _fsm.setKD(param);
            else if (param.get_name() == "LOWER_BOUND_TORQUE") _fsm.setLowerBoundTorque(param);
            else if (param.get_name() == "UPPER_BOUND_TORQUE") _fsm.setUpperBoundTorque(param);
        }

        return result;
    }

    void MotorControllerNode::handleDesiredTorqueAvailableEvt(const std_msgs::msg::Float64MultiArray::SharedPtr desired_motor_torque_msg)
    {
        this->_fsm.updateDesiredTorque(desired_motor_torque_msg);
    }

    void MotorControllerNode::handleMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg)
    {
        this->_fsm.updateMotorState(motor_state_msg);
    }

    void MotorControllerNode::handleDesiredMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg)
    {
        this->_fsm.updateDesiredMotorState(desired_motor_state_msg);
    }

    void MotorControllerNode::handleTimerEvt()
    {
        static Eigen::VectorXd motor_torque;
        
        this->_fsm.loopEvent();
        motor_torque = this->_fsm.getMotorTorque();
        this->publishMotorInterfaces(motor_torque);
    }

    void MotorControllerNode::publishMotorInterfaces(const Eigen::VectorXd &motor_torques)
    {
        static std_msgs::msg::Float64 msg;
        static uint8_t motor_index;

        for(const auto &motor_config : this->_motor_configs)
        {
            motor_index = motor_config.index;
            msg.data = motor_torques(motor_index);
            this->_motor_publisher_hub[motor_index]->publish(msg);
        }
    }
}

