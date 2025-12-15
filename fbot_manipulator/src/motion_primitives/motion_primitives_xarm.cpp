#include "fbot_manipulator/motion_primitives_xarm.hpp"

#include <chrono>
#include <exception>

using namespace std::chrono_literals;

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
    auto poses = manipulator_config_["poses"];
    if (!poses[target_name]) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] Named target '%s' not found in current manipulator config.",
                     target_name.c_str());
        return false;
    }

    auto target_joint_positions = poses[target_name].as<std::vector<double>>();

    auto plan_success = planJointTarget(target_joint_positions);

    if (plan_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_joint service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_joint succeeded");

    auto exec_success = executePath();

    if (exec_success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_exec service succeeded");

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

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_joint succeeded");

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
