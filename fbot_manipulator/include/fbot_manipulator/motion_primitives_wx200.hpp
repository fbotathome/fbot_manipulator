#pragma once

#include "fbot_manipulator/motion_primitives_base.hpp"
namespace fbot_manipulator
{
/*
* @brief Motion primitives implementation for WidowX 200 (wx200) robots.
*/
class MotionPrimitivesWX200 : public MotionPrimitivesBase
{
public:
    MotionPrimitivesWX200(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesWX200(const std::string& arm_name);

    ~MotionPrimitivesWX200() override = default;

    bool moveToNamedTarget(const std::string& target_name) override;
    bool moveToJointTarget(const std::vector<double>& joint_positions) override;
    bool moveToPose(const geometry_msgs::msg::Pose& pose) override;

private:

};

} // namespace fbot_manipulator
