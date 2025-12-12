#pragma once

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "fbot_manipulator/utils.hpp"

namespace fbot_manipulator
{

class MotionPrimitivesBase
{
public:
    MotionPrimitivesBase(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesBase(const std::string& arm_name);

    virtual ~MotionPrimitivesBase() = default;

    void init();

    // Generic API
    bool setGripperPosition(double position);

    // Robot-specific motion primitives (must be implemented by derived class)
    virtual bool moveToNamedTarget(const std::string& target_name) = 0;
    virtual bool moveToJointTarget(const std::vector<double>& joint_positions) = 0;
    virtual bool moveToPose(const geometry_msgs::msg::Pose& pose) = 0;

protected:
    // Overridable hook to initialize robot-specific service clients
    virtual void init_service_clients() = 0;

    // Generic initializers
    virtual void init_action_clients();
    void load_config();

    rclcpp::Node::SharedPtr node_;
    std::string arm_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    YAML::Node manipulator_config_;

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;
};

} // namespace fbot_manipulator
