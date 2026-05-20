#ifndef FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_
#define FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace fbot_manipulator
{

class ObjectCollisionManager : public rclcpp::Node
{
public:
    ObjectCollisionManager();
    ~ObjectCollisionManager() = default;

    void addSupportSurface(
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        const geometry_msgs::msg::Vector3& object_size,
        const std::string& frame_id = "link_base",
        double support_thickness = 0.02);

    void removeSupportSurface(const std::string& object_id);

private:
    moveit_msgs::msg::CollisionObject createSupportSurface(
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        const geometry_msgs::msg::Vector3& object_size,
        const std::string& frame_id,
        double support_thickness);

    // Publishers
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
};

}  // namespace fbot_manipulator

#endif  // FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_
