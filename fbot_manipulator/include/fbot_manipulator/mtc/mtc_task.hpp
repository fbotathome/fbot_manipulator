#pragma once

#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_eigen/tf2_eigen.hpp>

namespace fbot_manipulator
{
namespace mtc = moveit::task_constructor;

struct MtcConfig
{
    std::string arm_group_name = "xarm6";
    std::string hand_group_name = "xarm_gripper";
    std::string hand_frame = "link_tcp";
    std::string world_frame = "world";
    std::string surface_link = "world";
    // SRDF named group states (differ between robots; e.g. the Interbotix SRDF uses
    // "Released"/"Grasping" for the gripper and "Home"/"Upright" for the arm).
    std::string hand_open_state = "open";
    std::string hand_closed_state = "close";
    std::string arm_home_state = "home";
    std::string arm_ready_state = "hold-up";  // pose held after a successful pick
    double approach_min = 0.05;
    double approach_max = 0.15;
    double lift_min = 0.08;
    double lift_max = 0.15;
    double retreat_min = 0.08;
    double retreat_max = 0.15;
    int max_solutions = 5;
    double grasp_angle_delta = M_PI / 4;
    // [roll, pitch, yaw] of the grasp IK frame relative to hand_frame. Defaults reproduce
    // the xArm6/link_tcp transform; the Interbotix ee_gripper_link uses [0, 0, 0].
    std::vector<double> grasp_frame_rpy{0.0, -M_PI / 2, M_PI};
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
};

class MtcTask
{
public:
    using Ptr = std::shared_ptr<MtcTask>;

    MtcTask(const std::string& task_name,
            rclcpp::Node::SharedPtr node);
    virtual ~MtcTask() = default;

    void loadConfig();

    void addCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& pose,
                            const geometry_msgs::msg::Vector3& size);

    void removeCollisionObject(const std::string& object_id);

    virtual bool buildTask() = 0;

    bool plan();

    bool execute();

    std::string name() const { return task_name_; }

protected:
    void setupSolvers();

    std::string task_name_;
    rclcpp::Node::SharedPtr node_;
    MtcConfig config_;
    mtc::Task task_;

    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner_;

    moveit::planning_interface::PlanningSceneInterface psi_;

    rclcpp::Logger logger() const { return node_->get_logger(); }
};

} // namespace fbot_manipulator
