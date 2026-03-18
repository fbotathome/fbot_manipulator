#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"

namespace fbot_manipulator
{

MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const std::string& object_id,
                                         const geometry_msgs::msg::Pose& place_pose)
    : MtcTask("pick_and_place", node),
      object_id_(object_id),
      place_pose_(place_pose)
{
}

bool MtcPickAndPlaceTask::buildTask()
{
    task_.stages()->setName("pick_and_place_" + object_id_);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    // ---- Current State ----
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    // ---- Open Gripper ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner_);
        stage->setGroup(config_.hand_group_name);
        stage->setGoal("open");
        task_.add(std::move(stage));
    }

    // ---- Move to Pick (Connect) ----
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{
                { config_.arm_group_name, pipeline_planner_ }
            });
        stage->setTimeout(3.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage));
    }

    // ---- Pick Object Container ----
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto container = std::make_unique<mtc::SerialContainer>("pick object");
        task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
        container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // Approach
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
            stage->properties().set("marker_ns", "approach");
            stage->properties().set("link", config_.hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config_.approach_min, config_.approach_max);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config_.hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate Grasp Pose + IK
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject(object_id_);
            stage->setAngleDelta(config_.grasp_angle_delta);
            stage->setMonitoredStage(current_state);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(0.5);
            wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            wrapper->setTimeout(2.0);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            container->insert(std::move(wrapper));
        }

        // Allow hand-object collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(object_id_,
                                   task_.getRobotModel()
                                       ->getJointModelGroup(config_.hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
            container->insert(std::move(stage));
        }

        // Close gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner_);
            stage->setGroup(config_.hand_group_name);
            stage->setGoal("close");
            container->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id_, config_.hand_frame);
            attach_object_stage = stage.get();
            container->insert(std::move(stage));
        }

        // Allow object-surface collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,surface)");
            stage->allowCollisions(object_id_, config_.surface_link, true);
            container->insert(std::move(stage));
        }

        // Lift
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(config_.lift_min, config_.lift_max);
            stage->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            stage->properties().set("marker_ns", "lift");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config_.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Forbid object-surface collision
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
            stage->allowCollisions(object_id_, config_.surface_link, false);
            container->insert(std::move(stage));
        }

        task_.add(std::move(container));
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
