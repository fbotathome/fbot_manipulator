#include "fbot_manipulator/mtc/mtc_pick_task.hpp"

namespace fbot_manipulator
{

MtcPickTask::MtcPickTask(rclcpp::Node::SharedPtr node,
                         const std::string& object_id)
    : MtcTask("pick", node),
      object_id_(object_id)
{
}

bool MtcPickTask::buildTask()
{
    task_.stages()->setName("pick_" + object_id_);
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
        stage->setGoal(config_.hand_open_state);
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

            // Move along the approach axis: X of the grasp IK frame, expressed in hand_frame.
            // (xarm6: grasp_frame_transform maps X->Z, so this is (0,0,1) as before; the Interbotix
            // ee_gripper_link is X-forward, so this is (1,0,0).)
            const Eigen::Vector3d approach_axis =
                config_.grasp_frame_transform.linear() * Eigen::Vector3d::UnitX();
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = config_.hand_frame;
            vec.vector.x = approach_axis.x();
            vec.vector.y = approach_axis.y();
            vec.vector.z = approach_axis.z();
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate Grasp Pose + IK
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose(config_.hand_open_state);
            stage->setObject(object_id_);
            stage->setAngleDelta(config_.grasp_angle_delta);
            stage->setMonitoredStage(current_state);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(4);
            wrapper->setMinSolutionDistance(0.2);
            wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            wrapper->setTimeout(5.0);
            wrapper->setIgnoreCollisions(true);
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
            stage->setGoal(config_.hand_closed_state);
            container->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_id_, config_.hand_frame);
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

        // ---- Return Home ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator
