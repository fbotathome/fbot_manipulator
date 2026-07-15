#include "fbot_manipulator/mtc/mtc_task.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>

#include <chrono>

namespace fbot_manipulator
{

MtcTask::MtcTask(const std::string& task_name,
                 rclcpp::Node::SharedPtr node)
    : task_name_(task_name),
      node_(node)
{
    loadConfig();
    setupSolvers();
    planning_scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);
}

void MtcTask::loadConfig()
{
    node_->get_parameter_or("mtc.arm_group_name", config_.arm_group_name, config_.arm_group_name);
    node_->get_parameter_or("mtc.hand_group_name", config_.hand_group_name, config_.hand_group_name);
    node_->get_parameter_or("mtc.hand_frame", config_.hand_frame, config_.hand_frame);
    node_->get_parameter_or("mtc.world_frame", config_.world_frame, config_.world_frame);
    node_->get_parameter_or("mtc.surface_link", config_.surface_link, config_.surface_link);
    node_->get_parameter_or("mtc.approach_min", config_.approach_min, config_.approach_min);
    node_->get_parameter_or("mtc.approach_max", config_.approach_max, config_.approach_max);
    node_->get_parameter_or("mtc.lift_min", config_.lift_min, config_.lift_min);
    node_->get_parameter_or("mtc.lift_max", config_.lift_max, config_.lift_max);
    node_->get_parameter_or("mtc.retreat_min", config_.retreat_min, config_.retreat_min);
    node_->get_parameter_or("mtc.retreat_max", config_.retreat_max, config_.retreat_max);
    node_->get_parameter_or("mtc.max_solutions", config_.max_solutions, config_.max_solutions);
    node_->get_parameter_or("mtc.grasp_angle_delta", config_.grasp_angle_delta, config_.grasp_angle_delta);
    node_->get_parameter_or("mtc.pour_angle_delta", config_.pour_angle_delta, config_.pour_angle_delta);
    node_->get_parameter_or("mtc.pour_wait_time", config_.pour_wait_time, config_.pour_wait_time);
    node_->get_parameter_or("mtc.pour_side_offset", config_.pour_side_offset, config_.pour_side_offset);
    node_->get_parameter_or("mtc.pour_above_offset", config_.pour_above_offset, config_.pour_above_offset);

    // Grasp frame: rotate Z to point out of gripper with offset
    double grasp_offset = 0.0;
    node_->get_parameter_or("mtc.grasp_offset", grasp_offset, 0.0);

    node_->get_parameter_or("mtc.support_height", config_.support_height, config_.support_height);

    config_.grasp_frame_transform = Eigen::Isometry3d::Identity();
    // First translate along Z (which becomes the approach direction after rotation)
    config_.grasp_frame_transform.translate(Eigen::Vector3d(0, 0, grasp_offset));
    // Rotate to align gripper approach direction, then flip 180° around Z to correct wrist orientation
    config_.grasp_frame_transform.rotate(
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    RCLCPP_INFO(logger(), "[MtcTask:%s] Grasp offset: %.3f m", task_name_.c_str(), grasp_offset);

    RCLCPP_INFO(logger(), "[MtcTask:%s] Config loaded: arm='%s', hand='%s', frame='%s'",
                task_name_.c_str(),
                config_.arm_group_name.c_str(),
                config_.hand_group_name.c_str(),
                config_.hand_frame.c_str());
}

void MtcTask::setupSolvers()
{
    pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    //pipeline_planner_->setPlannerId("RRTConnectkConfigDefault");
    pipeline_planner_->setMaxVelocityScalingFactor(0.3);
    pipeline_planner_->setMaxAccelerationScalingFactor(0.1);

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.002);

    joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    
}

void MtcTask::addCollisionObject(const std::string& object_id,
                                 const geometry_msgs::msg::Pose& pose,
                                 const geometry_msgs::msg::Vector3& size)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { size.x, size.y, size.z };
    object.pose = pose;

    psi_.applyCollisionObject(object);

    if (planning_scene_pub_) {
        moveit_msgs::msg::PlanningScene ps;
        ps.world.collision_objects.push_back(object);
        ps.is_diff = true;
        planning_scene_pub_->publish(ps);
    }

    // Add small support object underneath the detected object so the planner
    // treats the supporting surface as an obstacle and avoids penetrating it.
    moveit_msgs::msg::CollisionObject support;
    support.id = object_id + std::string("_support");
    support.header.frame_id = config_.world_frame;
    support.primitives.resize(1);
    support.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    support.primitives[0].dimensions = { size.x, size.y, config_.support_height };
    geometry_msgs::msg::Pose support_pose = pose;
    support_pose.position.z = pose.position.z - (size.z / 2.0) - (config_.support_height / 2.0) - 0.004;
    support.pose = support_pose;

    psi_.applyCollisionObject(support);
    if (planning_scene_pub_) {
        moveit_msgs::msg::PlanningScene ps2;
        ps2.world.collision_objects.push_back(support);
        ps2.is_diff = true;
        planning_scene_pub_->publish(ps2);
    }

    RCLCPP_INFO(logger(), "[MtcTask:%s] Added collision object '%s' at (%.2f, %.2f, %.2f) size (%.2f, %.2f, %.2f)",
                task_name_.c_str(), object_id.c_str(),
                pose.position.x, pose.position.y, pose.position.z,
                size.x, size.y, size.z);
}

void MtcTask::removeCollisionObject(const std::string& object_id)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = config_.world_frame;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi_.applyCollisionObject(object);

    if (planning_scene_pub_) {
        moveit_msgs::msg::PlanningScene ps;
        ps.world.collision_objects.push_back(object);
        ps.is_diff = true;
        planning_scene_pub_->publish(ps);
    }

    RCLCPP_INFO(logger(), "[MtcTask:%s] Removed collision object '%s'",
                task_name_.c_str(), object_id.c_str());
}

void MtcTask::setSurfaceInfo(const geometry_msgs::msg::Pose& pose,
                             const geometry_msgs::msg::Vector3& size)
{
    object_pose_ = pose;
    object_size_ = size;
    has_surface_info_ = true;
}

void MtcTask::createSupportSurface(const std::string& object_id)
{
    if (!has_surface_info_)
    {
        RCLCPP_WARN(logger(), "[MtcTask:%s] No surface info provided, skipping support surface",
                    task_name_.c_str());
        return;
    }

    moveit_msgs::msg::CollisionObject support_obj;
    support_obj.id = object_id + "_support";
    support_obj.header.frame_id = config_.world_frame;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = object_size_.x + 0.2;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = object_size_.y + 0.7;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.01;

    geometry_msgs::msg::Pose support_pose;
    support_pose.position.x = object_pose_.position.x + 0.06;
    support_pose.position.y = object_pose_.position.y;
    support_pose.position.z = object_pose_.position.z - (object_size_.z / 2.0) - 0.02;
    support_pose.orientation.x = 0.0;
    support_pose.orientation.y = 0.0;
    support_pose.orientation.z = 0.0;
    support_pose.orientation.w = 1.0;
    support_obj.primitives.push_back(primitive);
    support_obj.primitive_poses.push_back(support_pose);
    support_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi_.applyCollisionObject(support_obj);

    RCLCPP_INFO(logger(),
               "[MtcTask:%s] Added support surface for '%s' at (%.2f, %.2f, %.2f)",
               task_name_.c_str(), object_id.c_str(),
               support_pose.position.x, support_pose.position.y, support_pose.position.z);
}

void MtcTask::createTopSupportSurface(const std::string& object_id)
{
    if (!has_surface_info_)
    {
        RCLCPP_WARN(logger(), "[MtcTask:%s] No surface info provided, skipping support surface",
                    task_name_.c_str());
        return;
    }

    moveit_msgs::msg::CollisionObject support_obj_top;
    support_obj_top.id = object_id + "_support_top";
    support_obj_top.header.frame_id = config_.world_frame;

    shape_msgs::msg::SolidPrimitive primitive_top;
    primitive_top.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive_top.dimensions.resize(3);
    primitive_top.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = object_size_.x + 0.2;
    primitive_top.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = object_size_.y + 0.7;
    primitive_top.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.01;

    geometry_msgs::msg::Pose support_pose_top;
    support_pose_top.position.x = object_pose_.position.x + 0.06;
    support_pose_top.position.y = object_pose_.position.y;
    support_pose_top.position.z = object_pose_.position.z + (object_size_.z / 2.0) + 0.2;
    support_pose_top.orientation.x = 0.0;
    support_pose_top.orientation.y = 0.0;
    support_pose_top.orientation.z = 0.0;
    support_pose_top.orientation.w = 1.0;
    support_obj_top.primitives.push_back(primitive_top);
    support_obj_top.primitive_poses.push_back(support_pose_top);
    support_obj_top.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi_.applyCollisionObject(support_obj_top);

    RCLCPP_INFO(logger(),
               "[MtcTask:%s] Added support top surface for '%s' at (%.2f, %.2f, %.2f)",
               task_name_.c_str(), object_id.c_str(),
               support_pose_top.position.x, support_pose_top.position.y, support_pose_top.position.z);
}


void MtcTask::removeTopSupportSurface(const std::string& object_id) {
    if(!has_surface_info_){
        return;
    }

    moveit_msgs::msg::CollisionObject support_obj_top;
    support_obj_top.id = object_id + "_support_top";
    support_obj_top.header.frame_id = config_.world_frame;
    support_obj_top.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    psi_.applyCollisionObject(support_obj_top);

    RCLCPP_INFO(logger(), "[MtcTask:%s] Removed support surface '%s_support'",
                task_name_.c_str(), object_id.c_str());

}

void MtcTask::removeSupportSurface(const std::string& object_id)
{
    if (!has_surface_info_)
    {
        return;
    }

    moveit_msgs::msg::CollisionObject support_obj;
    support_obj.id = object_id + "_support";
    support_obj.header.frame_id = config_.world_frame;
    support_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    psi_.applyCollisionObject(support_obj);

    RCLCPP_INFO(logger(), "[MtcTask:%s] Removed support surface '%s_support'",
                task_name_.c_str(), object_id.c_str());
}

bool MtcTask::plan()
{
    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Init failed: %s",
                     task_name_.c_str(), e.what());
        return false;
    }

    if (!task_.plan(config_.max_solutions))
    {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Planning failed", task_name_.c_str());
        task_.printState();
        return false;
    }

    RCLCPP_INFO(logger(), "[MtcTask:%s] Planning succeeded with %zu solutions",
                task_name_.c_str(), task_.solutions().size());
    return true;
}

bool MtcTask::execute()
{
    if (task_.solutions().empty())
    {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] No solutions to execute", task_name_.c_str());
        return false;
    }

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(logger(), "[MtcTask:%s] Execution failed with error code %d",
                     task_name_.c_str(), result.val);
        return false;
    }

    RCLCPP_INFO(logger(), "[MtcTask:%s] Execution succeeded", task_name_.c_str());
    return true;
}

} // namespace fbot_manipulator
