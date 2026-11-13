#pragma once

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "fbot_manipulator/utils.hpp"
#include "fbot_manipulator/robot_collision.hpp"

namespace fbot_manipulator
{
/*
* @brief Base class for motion primitives. Robot-specific motion primitive classes should inherit from this class.
*/
class MotionPrimitivesBase
{
public:
    MotionPrimitivesBase(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);
    MotionPrimitivesBase(const std::string& arm_name);

    virtual ~MotionPrimitivesBase() = default;

    void init();

    /*
    * @brief Set the gripper position
    * @param position Desired gripper position
    * @return true if the gripper reached the desired position, false otherwise
    */
    bool setGripperPosition(double position);

    // Robot-specific motion primitives (must be implemented by derived class)
    virtual bool moveToNamedTarget(const std::string& target_name) = 0;
    virtual bool moveToJointTarget(const std::vector<double>& joint_positions) = 0;
    virtual bool moveToPose(const geometry_msgs::msg::Pose& pose) = 0;

    /*
    * @brief Plan a joint target
    * @param joint_target Desired joint positions
    * @return true if planning was successful, false otherwise
    */
    virtual bool planJointTarget(const std::vector<double>& joint_target);

    /*
    * @brief Plan a pose target
    * @param pose_target Desired end-effector pose
    * @return true if planning was successful, false otherwise
    */
    virtual bool planPoseTarget(const geometry_msgs::msg::Pose& pose_target);

    /*
    * @brief Execute the planned path
    * @param wait If true, wait for execution to complete 
    * @return true if execution was successful, false otherwise
    */
    virtual bool executePath(bool wait = true);

protected:
    /*
    * @brief Make sure the robot body collision object is in the planning scene, so plans never
    *        go through the robot itself. Must be called before every plan: MTC tasks remove the
    *        object when they finish.
    * @return false if the object could not be applied (do not plan in that case)
    */
    bool ensureRobotCollisionObject();

    // Generic initializers
    virtual void init_action_clients();
    void load_config();

    rclcpp::Node::SharedPtr node_;
    std::string arm_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    YAML::Node manipulator_config_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    moveit_msgs::msg::RobotTrajectory trajectory_;
    bool is_trajectory_;

    // Robot body collision (same object the MTC tasks add)
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
    RobotCollisionConfig robot_collision_;
    std::string robot_collision_id_ = "boris_robot";
    std::string robot_collision_frame_ = "link_base";
    bool enable_robot_collision_ = true;

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;
};

} // namespace fbot_manipulator
