#ifndef FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_
#define FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fbot_manipulator
{

class BottleCollision : public rclcpp::Node
{
public:
    BottleCollision();
    ~BottleCollision() = default;

private:
    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    moveit_msgs::msg::CollisionObject createCollisionObject(
        const geometry_msgs::msg::Point& position, 
        const std::string& frame_id);

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers and Subscribers
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
};

}  // namespace fbot_manipulator

#endif  // FBOT_MANIPULATOR__COLLISION__ADD_OBJECT_COLLISION_HPP_
