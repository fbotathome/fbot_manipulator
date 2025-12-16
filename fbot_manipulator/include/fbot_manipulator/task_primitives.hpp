#pragma once

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


namespace fbot_manipulator
{

/*
* @brief High-level task primitives for manipulation tasks.
*/
class TaskPrimitives
{
public:
    TaskPrimitives(rclcpp::Node::SharedPtr node);

    // TASKS
    bool pickObject(const geometry_msgs::msg::Pose& pose);
    bool placeObject(const geometry_msgs::msg::Pose& pose);
    
private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_action_client_;
    bool sendGripperTrajectory(double position);

};  

}