#include "fbot_manipulator/mtc/mtc_place_task.hpp"

namespace fbot_manipulator
{

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                           const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("place", node),
      object_id_(object_id),
      place_pose_(place_pose)
{
}

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const std::string& object_id,
                           const std::string& place_pose_name)
    : MtcTask("place", node),
      object_id_(object_id),
      place_pose_name_(place_pose_name)
{
}

bool MtcPlaceTask::buildTask()
{
    task_.stages()->setName("place_" + object_id_);
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

    // ---- Move to Place (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{
                { config_.arm_group_name, pipeline_planner_ }
            });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage));
    }

    // ---- Place Object Container ----
    {
        auto container = std::make_unique<mtc::SerialContainer>("place object");
        task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        if (place_pose_name_.empty())
        {
            // Lower
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
                wrapper->setTimeout(2.0);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                container->insert(std::move(wrapper));
            }
        }
        else
        {
            // Move to named SRDF state
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
                stage->setGroup(config_.arm_group_name);
                stage->setGoal(place_pose_name_);
                container->insert(std::move(stage));
            }
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
