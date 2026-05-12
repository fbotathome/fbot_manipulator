#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

MtcTask::MtcTask(const std::string& task_name,
                 rclcpp::Node::SharedPtr node)
    : task_name_(task_name),
      node_(node)
{
    loadConfig();
    setupSolvers();
}

void MtcTask::loadConfig()
{
    node_->get_parameter_or("mtc.arm_group_name", config_.arm_group_name, config_.arm_group_name);
    node_->get_parameter_or("mtc.hand_group_name", config_.hand_group_name, config_.hand_group_name);
    node_->get_parameter_or("mtc.hand_frame", config_.hand_frame, config_.hand_frame);
    node_->get_parameter_or("mtc.world_frame", config_.world_frame, config_.world_frame);
    node_->get_parameter_or("mtc.surface_link", config_.surface_link, config_.surface_link);
    node_->get_parameter_or("mtc.hand_open_state", config_.hand_open_state, config_.hand_open_state);
    node_->get_parameter_or("mtc.hand_closed_state", config_.hand_closed_state, config_.hand_closed_state);
    node_->get_parameter_or("mtc.arm_home_state", config_.arm_home_state, config_.arm_home_state);
    node_->get_parameter_or("mtc.arm_ready_state", config_.arm_ready_state, config_.arm_ready_state);
    node_->get_parameter_or("mtc.approach_min", config_.approach_min, config_.approach_min);
    node_->get_parameter_or("mtc.approach_max", config_.approach_max, config_.approach_max);
    node_->get_parameter_or("mtc.lift_min", config_.lift_min, config_.lift_min);
    node_->get_parameter_or("mtc.lift_max", config_.lift_max, config_.lift_max);
    node_->get_parameter_or("mtc.retreat_min", config_.retreat_min, config_.retreat_min);
    node_->get_parameter_or("mtc.retreat_max", config_.retreat_max, config_.retreat_max);
    node_->get_parameter_or("mtc.max_solutions", config_.max_solutions, config_.max_solutions);
    node_->get_parameter_or("mtc.grasp_angle_delta", config_.grasp_angle_delta, config_.grasp_angle_delta);

    // Grasp IK frame relative to hand_frame. Convention: the IK frame's X axis is the
    // gripper approach direction (what GenerateGraspPose expects), and grasp_offset shifts
    // the grasp point along that approach axis. The rotation (grasp_frame_rpy) depends on
    // the robot's hand-frame convention -- the defaults reproduce the original xArm6/link_tcp
    // transform; the Interbotix ee_gripper_link is already X-forward, so it uses [0, 0, 0].
    double grasp_offset = 0.0;
    node_->get_parameter_or("mtc.grasp_offset", grasp_offset, 0.0);

    std::vector<double> grasp_rpy;
    node_->get_parameter_or("mtc.grasp_frame_rpy", grasp_rpy, config_.grasp_frame_rpy);
    if (grasp_rpy.size() != 3) {
        RCLCPP_WARN(logger(),
                    "[MtcTask:%s] mtc.grasp_frame_rpy must have 3 elements [roll,pitch,yaw]; using default",
                    task_name_.c_str());
        grasp_rpy = {0.0, -M_PI / 2, M_PI};
    }

    config_.grasp_frame_transform = Eigen::Isometry3d::Identity();
    config_.grasp_frame_transform.rotate(
        Eigen::AngleAxisd(grasp_rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(grasp_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(grasp_rpy[0], Eigen::Vector3d::UnitX()));
    config_.grasp_frame_transform.translate(Eigen::Vector3d(grasp_offset, 0.0, 0.0));

    RCLCPP_INFO(logger(),
                "[MtcTask:%s] Grasp: offset=%.3f m along approach, frame_rpy=[%.3f, %.3f, %.3f]",
                task_name_.c_str(), grasp_offset, grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]);

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

    RCLCPP_INFO(logger(), "[MtcTask:%s] Removed collision object '%s'",
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
