#include "fbot_manipulator/robot_collision.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace fbot_manipulator
{

RobotCollisionConfig loadRobotCollisionConfig(const rclcpp::Node::SharedPtr& node)
{
    RobotCollisionConfig config;

    const auto load_box = [&node](const std::string& name, RobotBoxConfig& box) {
        node->get_parameter_or(name + ".size_x", box.size_x, box.size_x);
        node->get_parameter_or(name + ".size_y", box.size_y, box.size_y);
        node->get_parameter_or(name + ".size_z", box.size_z, box.size_z);
        node->get_parameter_or(name + ".offset_x", box.offset_x, box.offset_x);
        node->get_parameter_or(name + ".offset_y", box.offset_y, box.offset_y);
        node->get_parameter_or(name + ".offset_z", box.offset_z, box.offset_z);
    };

    load_box("boris_robot.base", config.base);
    load_box("boris_robot.torso", config.torso);
    return config;
}

moveit_msgs::msg::CollisionObject makeRobotCollisionObject(const RobotCollisionConfig& config,
                                                           const std::string& id,
                                                           const std::string& frame,
                                                           const rclcpp::Time& stamp)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header.frame_id = frame;
    object.header.stamp = stamp;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Explicitly set identity pose for the root collision object
    object.pose.orientation.w = 1.0;

    // Collision boxes and poses are derived from fbot_description/urdf/v2/robot.xacro (or robot_collision_config.yaml).
    const auto add_box = [&object](const RobotBoxConfig& box) {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {box.size_x, box.size_y, box.size_z};

        geometry_msgs::msg::Pose pose;
        pose.position.x = box.offset_x;
        pose.position.y = box.offset_y;
        pose.position.z = box.offset_z;
        pose.orientation.w = 1.0;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
    };

    add_box(config.base);
    add_box(config.torso);
    return object;
}

} // namespace fbot_manipulator
