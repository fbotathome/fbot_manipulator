#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_eigen/tf2_eigen.hpp>

#include "fbot_manipulator/robot_collision.hpp"

namespace fbot_manipulator
{
namespace mtc = moveit::task_constructor;

struct ShelfSurfaceConfig
{
    double x_padding = 0.2;
    double y_padding = 0.7;
    double thickness = 0.01;
    double x_offset = 0.06;
    double y_offset = 0.0;
    double z_offset = 0.0;
};

struct ShelfSideConfig
{
    double x_thickness = 0.01;
    double y_padding = 0.3;
    double z_padding = 0.2;
    double x_gap = 0.01;
    double y_offset = 0.2;
    double z_offset = 0.10;
    double orientation_z = 0.70710678;
    double orientation_w = 0.70710678;
};

struct ShelfConfig
{
    ShelfSurfaceConfig bottom;
    ShelfSurfaceConfig top{0.2, 0.7, 0.01, 0.06, 0.0, 0.2};
    ShelfSideConfig right;
    ShelfSideConfig left{0.01, 0.3, 0.2, 0.01, -0.15, 0.10, -0.70710678, 0.70710678};
    double rotation_yaw = 0.0;
    double object_support_clearance = 0.004;
};

struct MtcConfig
{
    std::string arm_group_name = "xarm6";
    std::string hand_group_name = "xarm_gripper";
    std::string hand_frame = "link_tcp";
    std::string world_frame = "world";
    std::string surface_link = "world";
    double approach_min = 0.05;
    double approach_max = 0.15;
    double lift_min = 0.08;
    double lift_max = 0.15;
    double retreat_min = 0.08;
    double retreat_max = 0.15;
    int max_solutions = 5;
    double grasp_angle_delta = M_PI / 4;
    // Grasps are generated within +-grasp_angle_half_range of the straight-on approach.
    double grasp_angle_half_range = 1.32;  // about 75 degrees
    // Grasp IK: solutions kept per grasp pose, and time budget per grasp pose (s). With more
    // than one solution requested the IK stage uses the whole budget unless it finds them all.
    int grasp_ik_max_solutions = 2;
    double grasp_ik_timeout = 0.2;
    double pour_angle_delta = M_PI / 2;
    double pour_side_offset = 0.10;
    double pour_above_offset = 0.15;
    double pour_wait_time = 5.0;
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    double support_height = 0.01;
    bool enable_surfaces = true;
    std::string robot_collision_frame = "link_base";
    std::string robot_collision_id = "boris_robot";
    bool persistent_robot_collision = false;
    RobotCollisionConfig robot_collision;
    ShelfConfig shelf;
};

class MtcTask
{
public:
    using Ptr = std::shared_ptr<MtcTask>;

    MtcTask(const std::string& task_name,
            rclcpp::Node::SharedPtr node);
    virtual ~MtcTask() = default;

    void loadConfig();

    const MtcConfig& getConfig() const { return config_; }

    std::string resolveRobotCollisionFrame();

    void addCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& pose,
                            const geometry_msgs::msg::Vector3& size);

    void removeCollisionObject(const std::string& object_id);

    // Detach the object from the gripper (if attached) and delete it from the planning scene.
    // Used after a failed grasp check so the scene does not keep a phantom object the robot is
    // not actually holding. Safe to call whether or not the object is attached.
    void detachAndRemoveObject(const std::string& object_id);

    // Detach and delete every object still attached to the robot in the planning scene. Called
    // before a pick, which starts by opening the gripper: anything left attached by an earlier
    // run (a failed execution, a pick that was never followed by a place) is no longer held and
    // would otherwise put the start state in collision or distort every later plan.
    void clearAttachedObjects();

    // If object_id is attached to the robot as a box, return true and its dimensions in `size`.
    bool getAttachedObjectSize(const std::string& object_id, geometry_msgs::msg::Vector3& size);

    void addRobotCollisionObject();

    void removeRobotCollisionObject();

    void setSurfaceInfo(const geometry_msgs::msg::Pose& pose,
                        const geometry_msgs::msg::Vector3& size);


    // Functions to add and remove the collision surfaces
    void createSupportSurface(const std::string& object_id);
    
    void createTopSupportSurface(const std::string& object_id);

    void createRightSupportSurface(const std::string& object_id);
    
    void removeRightSupportSurface(const std::string& object_id);

    void createLeftSupportSurface(const std::string& object_id);

    void removeLeftSupportSurface(const std::string& object_id);

    void removeTopSupportSurface(const std::string& object_id);

    void removeSupportSurface(const std::string& object_id);

    

    virtual bool buildTask() = 0;

    bool plan();

    bool execute();

    std::string name() const { return task_name_; }

    // One-line explanation of the last plan() or execute() failure; empty if none.
    const std::string& failureReason() const { return failure_reason_; }

protected:
    void setupSolvers();

    // Diagnostics only: returns the colliding pairs ("a <-> b, ...") if the current robot state
    // is in collision in move_group's planning scene, or an empty string.
    std::string checkStartStateCollision();

    // Logs, per stage, how many candidates succeeded and failed and why, and returns a one-line
    // summary of the stages that rejected every candidate.
    std::string describePlanningFailure();

    std::string failure_reason_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;

    std::string task_name_;
    rclcpp::Node::SharedPtr node_;
    MtcConfig config_;
    mtc::Task task_;

    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner_;

    moveit::planning_interface::PlanningSceneInterface psi_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

    geometry_msgs::msg::Pose object_pose_;
    geometry_msgs::msg::Vector3 object_size_;
    bool has_surface_info_ = false;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Logger logger() const { return node_->get_logger(); }
};

} // namespace fbot_manipulator
