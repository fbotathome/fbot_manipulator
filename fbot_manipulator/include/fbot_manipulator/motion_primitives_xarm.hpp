#pragma once

#include "fbot_manipulator/motion_primitives_base.hpp"
namespace fbot_manipulator
{
/*
* @brief Motion primitives implementation for xArm robots.
*/
class MotionPrimitivesXArm : public MotionPrimitivesBase
{
public:
    // inherit base constructors
    MotionPrimitivesXArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesXArm(const std::string& arm_name);

    ~MotionPrimitivesXArm() override = default;

    /*
    * @brief Move to a named target defined in the manipulator configuration
    * @param target_name Name of the target pose
    * @return true if the movement was successful, false otherwise
    */
    bool moveToNamedTarget(const std::string& target_name) override;

    /*
    * @brief Move to a specified joint target
    * @param joint_positions Desired joint positions
    * @return true if the movement was successful, false otherwise
    */
    bool moveToJointTarget(const std::vector<double>& joint_positions) override;
    /*
    * @brief Move to a specified pose target
    * @param pose Desired end-effector pose
    * @return true if the movement was successful, false otherwise
    */
    bool moveToPose(const geometry_msgs::msg::Pose& pose) override;

private:

};

} // namespace fbot_manipulator
