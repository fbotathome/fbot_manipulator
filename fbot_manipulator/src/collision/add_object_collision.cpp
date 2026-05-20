#include "fbot_manipulator/collision/add_object_collision.hpp"

namespace fbot_manipulator
{

ObjectCollisionManager::ObjectCollisionManager()
    : Node("object_collision_manager")
{
    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);

    RCLCPP_INFO(this->get_logger(), "Object Collision Manager initialized");
}

void ObjectCollisionManager::addSupportSurface(
    const std::string& object_id,
    const geometry_msgs::msg::Pose& object_pose,
    const geometry_msgs::msg::Vector3& object_size,
    const std::string& frame_id,
    double support_thickness)
{
    auto collision_obj = createSupportSurface(object_id, object_pose, object_size, frame_id, support_thickness);

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_obj);
    planning_scene.is_diff = true;

    collision_pub_->publish(planning_scene);

    RCLCPP_INFO(this->get_logger(),
               "Added support surface '%s' below object at (%.3f, %.3f, %.3f)",
               object_id.c_str(),
               object_pose.position.x, object_pose.position.y, object_pose.position.z);
}

void ObjectCollisionManager::removeSupportSurface(const std::string& object_id)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.id = object_id + "_support";
    collision_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_obj);
    planning_scene.is_diff = true;

    collision_pub_->publish(planning_scene);

    RCLCPP_INFO(this->get_logger(), "Removed support surface '%s_support'", object_id.c_str());
}

moveit_msgs::msg::CollisionObject ObjectCollisionManager::createSupportSurface(
    const std::string& object_id,
    const geometry_msgs::msg::Pose& object_pose,
    const geometry_msgs::msg::Vector3& object_size,
    const std::string& frame_id,
    double support_thickness)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.frame_id = frame_id;
    collision_obj.header.stamp = this->get_clock()->now();
    collision_obj.id = object_id + "_support";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = object_size.x + 0.1;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = object_size.y + 0.1;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = support_thickness;

    geometry_msgs::msg::Pose support_pose;
    support_pose.position.x = object_pose.position.x;
    support_pose.position.y = object_pose.position.y;
    support_pose.position.z = object_pose.position.z - (object_size.z / 2.0) - (support_thickness / 2.0);
    support_pose.orientation.x = 0.0;
    support_pose.orientation.y = 0.0;
    support_pose.orientation.z = 0.0;
    support_pose.orientation.w = 1.0;

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(support_pose);
    collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    return collision_obj;
}

} // namespace fbot_manipulator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fbot_manipulator::ObjectCollisionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//Prateleira 1.70m 1.19m 55cm