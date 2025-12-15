#pragma once

#include "fbot_manipulator/motion_primitives_base.hpp"
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

private:

};

} // namespace fbot_manipulator
