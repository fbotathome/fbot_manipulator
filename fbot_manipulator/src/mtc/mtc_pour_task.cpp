#include "fbot_manipulator/mtc/mtc_pour_task.hpp"

namespace fbot_manipulator
{

MtcPourTask::MtcPourTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                   const geometry_msgs::msg::Pose& object_pose,
                           const geometry_msgs::msg::Pose& place_pose)
        : MtcTask("pour", node),
      object_id_(object_id),
    object_pose_(object_pose),
      place_pose_(place_pose)
{
}

MtcPourTask::MtcPourTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                   const geometry_msgs::msg::Pose& object_pose,
                           const std::string& place_pose_name)
        : MtcTask("pour", node),
      object_id_(object_id),
    object_pose_(object_pose),
      place_pose_name_(place_pose_name)
{
}

bool MtcPourTask::buildTask()
{
    const double pour_angle_delta = config_.pour_angle_delta;
    const double pre_pour_side_offset = 0.10;
    const double pre_pour_above_offset = 0.15;

    task_.stages()->setName("pour_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    // ---- Current State (object assumed already attached) ----
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
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
        pre_pour_target.pose.position.y -= pre_pour_side_offset;
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
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        container->insert(std::move(wrapper));

        task_.add(std::move(container));
    }

    if (place_pose_name_.empty())
    {
        // ---- Pour and Place Object Container ----
        {
            auto container = std::make_unique<mtc::SerialContainer>("pour and place object");
            task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
            container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

            // Lower near the table before placing.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner_);
                stage->properties().set("marker_ns", "lower");
                stage->properties().set("link", config_.hand_frame);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                stage->setMinMaxDistance(config_.lift_min, config_.lift_max);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = config_.world_frame;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                container->insert(std::move(stage));
            }

            // Generate Place Pose + IK
            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject(object_id_);

                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config_.world_frame;
                target.pose = place_pose_;
                stage->setPose(target);
                stage->setMonitoredStage(attach_object_stage);

                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(4);
                wrapper->setMinSolutionDistance(0.5);
                wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
                wrapper->setTimeout(5.0);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                container->insert(std::move(wrapper));
            }

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
                twist.twist.angular.z = 5.0;
                stage->setDirection(twist);
                container->insert(std::move(stage));
            }

            // Rotate back to a neutral wrist orientation before release.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("recover wrist", cartesian_planner_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                stage->setMinMaxDistance(pour_angle_delta, pour_angle_delta);
                stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
                stage->properties().set("marker_ns", "recover_wrist");

                geometry_msgs::msg::TwistStamped twist;
                twist.header.frame_id = config_.hand_frame;
                twist.twist.angular.z = -1.0;
                stage->setDirection(twist);
                container->insert(std::move(stage));
            }

            // Open gripper
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner_);
                stage->setGroup(config_.hand_group_name);
                stage->setGoal("open");
                container->insert(std::move(stage));
            }

            // Forbid hand-object collision
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                stage->allowCollisions(object_id_,
                                       task_.getRobotModel()
                                           ->getJointModelGroup(config_.hand_group_name)
                                           ->getLinkModelNamesWithCollisionGeometry(),
                                       false);
                container->insert(std::move(stage));
            }

            // Detach object
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                stage->detachObject(object_id_, config_.hand_frame);
                container->insert(std::move(stage));
            }

            // Retreat
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                stage->setMinMaxDistance(config_.retreat_min, config_.retreat_max);
                stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
                stage->properties().set("marker_ns", "retreat");

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = config_.world_frame;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                container->insert(std::move(stage));
            }

            task_.add(std::move(container));
        }
    }
    else
    {
        // ---- Move to named SRDF table pose ----
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
            stage->setGroup(config_.arm_group_name);
            stage->setGoal(place_pose_name_);
            task_.add(std::move(stage));
        }

        // Pour by rotating the wrist around the tool axis.
        {
            auto slow_pour_planner = std::make_shared<mtc::solvers::CartesianPath>();
            slow_pour_planner->setMaxVelocityScalingFactor(0.50);
            slow_pour_planner->setMaxAccelerationScalingFactor(0.40);
            slow_pour_planner->setStepSize(0.1);

            auto stage = std::make_unique<mtc::stages::MoveRelative>("pour wrist", slow_pour_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(pour_angle_delta, pour_angle_delta);
            stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            stage->properties().set("marker_ns", "pour_wrist");

            geometry_msgs::msg::TwistStamped twist;
            twist.header.frame_id = config_.hand_frame;
            twist.twist.angular.z = 5.0;
            stage->setDirection(twist);
            task_.add(std::move(stage));
        }

        // Rotate back before placing object on table.
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("recover wrist", cartesian_planner_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(pour_angle_delta, pour_angle_delta);
            stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            stage->properties().set("marker_ns", "recover_wrist");

            geometry_msgs::msg::TwistStamped twist;
            twist.header.frame_id = config_.hand_frame;
            twist.twist.angular.z = -1.0;
            stage->setDirection(twist);
            task_.add(std::move(stage));
        }

        // Open gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner_);
            stage->setGroup(config_.hand_group_name);
            stage->setGoal("open");
            task_.add(std::move(stage));
        }

        // Forbid hand-object collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(object_id_,
                                   task_.getRobotModel()
                                       ->getJointModelGroup(config_.hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   false);
            task_.add(std::move(stage));
        }

        // Detach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_id_, config_.hand_frame);
            task_.add(std::move(stage));
        }

        // Remove collision object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
            stage->removeObject(object_id_);
            task_.add(std::move(stage));
        }
    }

    // ---- Return Home ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal("home");
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator
