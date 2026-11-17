#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace fbot_manipulator
{

struct RobotBoxConfig
{
    double size_x = 0.577045;
    double size_y = 0.434;
    double size_z = 0.493089;
    double offset_x = -0.102573;
    double offset_y = 0.0;
    double offset_z = -0.25;
};

struct RobotCollisionConfig
{
    RobotBoxConfig base{0.577045, 0.434, 0.493089, -0.102573, 0.0, -0.25};
    RobotBoxConfig torso{0.234, 0.274, 0.8, -0.237573, 0.0, 0.3485};
    RobotBoxConfig sick{0.125737, 0.102200, 0.152269, 0.215030, 0.0, -0.082258};
    RobotBoxConfig femtobolt{0.065, 0.115, 0.040, -0.057048, 0.0, 0.848650};
    RobotBoxConfig femtobolt_support{0.040, 0.115, 0.065, -0.057048, 0.0, 0.848650};
    RobotBoxConfig femtobolt_pan_support{0.050, 0.050, 0.063, -0.076073, 0.0, 0.777300};
};

/**
 * @brief Read the robot body collision boxes from node parameters. Missing parameters keep
 *        their defaults.
 */
RobotCollisionConfig loadRobotCollisionConfig(const rclcpp::Node::SharedPtr& node);

/**
 * @brief Build the ADD collision object that represents the robot body (base + torso boxes).
 * @param frame Frame the box offsets are expressed in (e.g. link_base)
 */
moveit_msgs::msg::CollisionObject makeRobotCollisionObject(const RobotCollisionConfig& config,
                                                           const std::string& id,
                                                           const std::string& frame,
                                                           const rclcpp::Time& stamp);

} // namespace fbot_manipulator
