#include "fbot_manipulator/mtc/mtc_pour_task.hpp"

#include <algorithm>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace fbot_manipulator
{

namespace
{

class TimedPauseStage : public mtc::PropagatingEitherWay
{

public:
    TimedPauseStage(const std::string& name, double duration, const std::string& group)
        : mtc::PropagatingEitherWay(name), group_(group)
    {
        setProperty("duration", duration);
    }

protected:
    bool compute(const mtc::InterfaceState& state,
                 planning_scene::PlanningScenePtr& scene,
                 mtc::SubTrajectory& trajectory,
                 mtc::Interface::Direction /*dir*/) override
    {
        const double duration = std::max(0.0, properties().get<double>("duration"));
        scene = state.scene()->diff();

        auto wait_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(scene->getRobotModel(), group_);
        const auto& current_state = scene->getCurrentState();
        wait_trajectory->addSuffixWayPoint(current_state, 0.0);
        wait_trajectory->addSuffixWayPoint(current_state, duration);

        trajectory.setTrajectory(wait_trajectory);
        trajectory.setComment("wait");
        return true;
    }

private:
    std::string group_;
};

} // namespace

MtcPourTask::MtcPourTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id,
                         const geometry_msgs::msg::Pose& object_pose)
        : MtcTask("pour", node),
      object_id_(object_id),
      object_pose_(object_pose)
{
}

bool MtcPourTask::buildTask()
{
    const double pour_angle_delta = config_.pour_angle_delta;
    const double pre_pour_side_offset = config_.pour_side_offset;
    const double pre_pour_above_offset = config_.pour_above_offset;
    const double object_side_sign = (object_pose_.position.y >= 0.0) ? 1.0 : -1.0;
    // Rotate toward the side where the target object sits in the world Y axis.
    const double pour_direction_sign = -object_side_sign;
    const double pre_pour_side_sign = -object_side_sign;

    task_.stages()->setName("pour_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);
    const auto hand_links = task_.getRobotModel()
                                ->getJointModelGroup(config_.hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry();
    const auto arm_links = task_.getRobotModel()
                               ->getJointModelGroup(config_.arm_group_name)
                               ->getLinkModelNamesWithCollisionGeometry();

    // ---- Current State (object assumed already attached) ----
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
        task_.add(std::move(stage));
    }

    // The object is expected to be attached already for POUR.
    // Allow collisions with end-effector and arm links during pouring.
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        stage->allowCollisions(object_id_, hand_links, true);
        task_.add(std::move(stage));
    }
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (arm,object)");
        stage->allowCollisions(object_id_, arm_links, true);
        task_.add(std::move(stage));
    }

    // Move to a pre-pour pose above and slightly to the side of object_pose.
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pre-pour approach",
            mtc::stages::Connect::GroupPlannerVector{
                { config_.arm_group_name, pipeline_planner_ }
            });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage));
    }

    {
        auto container = std::make_unique<mtc::SerialContainer>("approach object pose");
        task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate pre-pour pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "pre_pour_pose");
        stage->setObject(object_id_);

        geometry_msgs::msg::PoseStamped pre_pour_target;
        pre_pour_target.header.frame_id = config_.world_frame;
        pre_pour_target.pose = object_pose_;
        pre_pour_target.pose.position.y += pre_pour_side_sign * pre_pour_side_offset;
        pre_pour_target.pose.position.z += pre_pour_above_offset;
        pre_pour_target.pose.orientation.x = 0.0;
        pre_pour_target.pose.orientation.y = 0.0;
        pre_pour_target.pose.orientation.z = 0.0;
        pre_pour_target.pose.orientation.w = 1.0;
        stage->setPose(pre_pour_target);
        stage->setMonitoredStage(attach_object_stage);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("pre-pour pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(4);
        wrapper->setMinSolutionDistance(0.5);
        wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
        wrapper->setTimeout(5.0);
        wrapper->setIgnoreCollisions(true);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        container->insert(std::move(wrapper));

        task_.add(std::move(container));
    }

    // ---- Pour Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("pour object");
        task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Pour by rotating the wrist around the tool axis.
        {
            auto slow_pour_planner = std::make_shared<mtc::solvers::CartesianPath>();
            slow_pour_planner->setMaxVelocityScalingFactor(0.08);
            slow_pour_planner->setMaxAccelerationScalingFactor(0.05);
            slow_pour_planner->setStepSize(0.001);

            auto stage = std::make_unique<mtc::stages::MoveRelative>("pour wrist", slow_pour_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(pour_angle_delta, pour_angle_delta);
            stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            stage->properties().set("marker_ns", "pour_wrist");

            geometry_msgs::msg::TwistStamped twist;
            twist.header.frame_id = config_.hand_frame;
            twist.twist.angular.z = pour_direction_sign;
            stage->setDirection(twist);
            container->insert(std::move(stage));
        }

        // Wait for pour to complete.
        {
            auto stage = std::make_unique<TimedPauseStage>("wait for pour", config_.pour_wait_time, config_.arm_group_name);
            container->insert(std::move(stage));
        }

        // Rotate back after pouring.
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("recover wrist", cartesian_planner_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(pour_angle_delta, pour_angle_delta);
            stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            stage->properties().set("marker_ns", "recover_wrist");

            geometry_msgs::msg::TwistStamped twist;
            twist.header.frame_id = config_.hand_frame;
            twist.twist.angular.z = -pour_direction_sign;
            stage->setDirection(twist);
            container->insert(std::move(stage));
        }
        task_.add(std::move(container));
    }

    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return hold-up", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal("hold-up");
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator
