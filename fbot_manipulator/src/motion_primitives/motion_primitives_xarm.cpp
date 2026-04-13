#include "fbot_manipulator/motion_primitives_xarm.hpp"


namespace fbot_manipulator
{

MotionPrimitivesXArm::MotionPrimitivesXArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name)
    : MotionPrimitivesBase(node, arm_name)
{
    init();
}

MotionPrimitivesXArm::MotionPrimitivesXArm(const std::string& arm_name)
    : MotionPrimitivesBase(arm_name)
{
    init();
}

bool MotionPrimitivesXArm::moveToNamedTarget(const std::string& target_name)
{    
    bool success = move_group_->setNamedTarget(target_name);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Named target '%s' not found in SRDF", target_name.c_str());
        return false;
    }

    success = (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to plan to named target: %s", target_name.c_str());
        return false;
    }

    is_trajectory_ = false;
    success = executePath();
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute path to named target: %s", target_name.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Successfully moved to named target: %s", target_name.c_str());
    return true;
}

bool MotionPrimitivesXArm::moveToJointTarget(const std::vector<double>& joint_positions)
{
    auto plan_success = planJointTarget(joint_positions);

    if (plan_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_joint failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Plan to named target '%s' succeeded, executing...", target_name.c_str());

    auto exec_success = executePath();

    if (exec_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_exec succeeded");

    return true;
}

bool MotionPrimitivesXArm::moveToPose(const geometry_msgs::msg::Pose& pose)
{

    auto plan_success = planPoseTarget(pose);

    if (plan_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_pose failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_pose succeeded");

    auto exec_success = executePath();

    if (exec_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_exec succeeded");

    return true;
}

} // namespace fbot_manipulator
