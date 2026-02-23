#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/variant/get.hpp>

#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("add_mesh_shelf_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject shelf_object;
    shelf_object.header.frame_id = "link_base"; 
    shelf_object.id = "mesh_shelf";

    
    std::string mesh_uri = "package://fbot_manipulator/mesh/Shelf.obj";
    
    Eigen::Vector3d scale(1.0, 1.0, 1.0); 

    shapes::Mesh* m = shapes::createMeshFromResource(mesh_uri, scale);
    
    if (m == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load mesh from %s", mesh_uri.c_str());
        executor.cancel();
        rclcpp::shutdown();
        if (executor_thread.joinable()) {
            executor_thread.join();
        }
        return -1;
    }

    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(m, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    delete m; 


    geometry_msgs::msg::Pose shelf_pose;
    shelf_pose.orientation.w = 1.0;
    shelf_pose.position.x = 1.0;
    shelf_pose.position.y = 0.0;
    shelf_pose.position.z = 0.0;

    shelf_object.meshes.push_back(mesh_msg);
    shelf_object.mesh_poses.push_back(shelf_pose);
    shelf_object.operation = shelf_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(shelf_object);

    RCLCPP_INFO(node->get_logger(), "Adding the mesh shelf into the planning scene...");
    planning_scene_interface.applyCollisionObjects(collision_objects);

    rclcpp::sleep_for(std::chrono::seconds(1));

    rclcpp::shutdown();
    executor_thread.join();
    
    return 0;
}