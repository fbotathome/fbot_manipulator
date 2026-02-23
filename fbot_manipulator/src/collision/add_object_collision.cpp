#include "fbot_manipulator/collision/add_object_collision.hpp"
#include <algorithm>
#include <string>

namespace fbot_manipulator
{

BottleCollision::BottleCollision()
    : Node("add_object_collision")
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);

    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fbot_vision/fr/object_markers", 10,
        std::bind(&BottleCollision::markerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Add Object Collision node initialized");
}

void BottleCollision::markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto& marker : msg->markers)
    {
        std::string marker_text = marker.text;
        std::transform(marker_text.begin(), marker_text.end(), marker_text.begin(),
                      [](unsigned char c){ return std::tolower(c); });

        if (marker_text.find("bottle") != std::string::npos) //In Bottle Change to whatever needs to add collision
        {
            geometry_msgs::msg::PoseStamped pose_camera;
            pose_camera.header.frame_id = "camera_color_optical_frame";
            pose_camera.header.stamp = marker.header.stamp;
            pose_camera.pose.position = marker.pose.position;
            pose_camera.pose.orientation = marker.pose.orientation;

            try
            {
                // Transform to base frame
                geometry_msgs::msg::PoseStamped pose_base;
                pose_base = tf_buffer_->transform(
                    pose_camera, 
                    "link_base",
                    tf2::durationFromSec(1.0));

                RCLCPP_INFO(this->get_logger(), 
                           "Collision updated at X: %.3f", 
                           pose_base.pose.position.x);

                // Create and publish collision object
                auto collision_obj = createCollisionObject(
                    pose_base.pose.position, 
                    "link_base");

                moveit_msgs::msg::PlanningScene planning_scene;
                planning_scene.world.collision_objects.push_back(collision_obj);
                planning_scene.is_diff = true;

                collision_pub_->publish(planning_scene);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF Failed: %s", ex.what());
                return;
            }
        }
    }
}

moveit_msgs::msg::CollisionObject BottleCollision::createCollisionObject(
    const geometry_msgs::msg::Point& position,
    const std::string& frame_id)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.frame_id = frame_id;
    collision_obj.header.stamp = this->get_clock()->now();
    collision_obj.id = "bottle_obstacle";

    // Create box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.12;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.12;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

    geometry_msgs::msg::Pose pose;
    pose.position = position;
    pose.position.z -= 0.10;  

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(pose);
    collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    return collision_obj;
}

} // namespace fbot_manipulator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fbot_manipulator::BottleCollision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





//Prateleira 1.70m 1.19m 55cm