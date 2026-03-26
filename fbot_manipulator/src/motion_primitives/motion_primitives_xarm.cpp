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

bool MotionPrimitivesXArm::moveToNamedTarget(const std::string& target_name) //edit here
{
   static const std::string PLANNING_GROUP = "xarm6";
   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   
    move_group.setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::plan real_plan;
    bool success = (move_group.plan(real_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

 if (success) {
    move_group.execute(my_plan);
    ROS_INFO("Successfully moved to pose: %s", target_pose.c_str());
  } else {
    ROS_ERROR("Failed to plan to pose: %s", target_pose.c_str());
  }

  ros::shutdown();
  return 0;
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
