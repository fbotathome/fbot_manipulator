#pragma once

#include "fbot_manipulator/motion_primitives_base.hpp"

#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>

namespace fbot_manipulator
{

class MotionPrimitivesXArm : public MotionPrimitivesBase
{
public:
    // inherit base constructors
    MotionPrimitivesXArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesXArm(const std::string& arm_name);

    ~MotionPrimitivesXArm() override = default;

    // Implement motion APIs
    bool moveToNamedTarget(const std::string& target_name) override;
    bool moveToJointTarget(const std::vector<double>& joint_positions) override;
    bool moveToPose(const geometry_msgs::msg::Pose& pose) override;

protected:
    // initialize xarm-specific service clients
    void init_service_clients() override;

private:
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr plan_joint_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr plan_exec_client_;
    rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr plan_pose_client_;
};

} // namespace fbot_manipulator
