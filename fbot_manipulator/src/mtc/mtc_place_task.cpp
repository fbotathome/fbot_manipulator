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

    if (place_pose_name_.empty())
    {
        // ---- Move to Place (Connect) ----
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
                "move to place",
                mtc::stages::Connect::GroupPlannerVector{
                    { config_.arm_group_name, pipeline_planner_ }
                });
            stage->setTimeout(1.5);
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
                // Dedicated, smaller descent than the pick lift so the pre-place does not sit so
                // high above the place point -- on a shelf a tall clearance hits the shelf above.
                stage->setMinMaxDistance(config_.place_lower_min, config_.place_lower_max);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = config_.world_frame;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                container->insert(std::move(stage));
            }

            // Generate Place Pose + IK
            {
                // A <6-DOF arm (e.g. the 5-DOF WidowX 200) cannot achieve an arbitrary place
                // orientation, so the recorded place orientation is usually unreachable and
                // "place pose IK" finds no solution. Mirror the waist-aligned grasp (see
                // MtcPickTask): drive the gripper to a single level pose whose yaw points straight
                // out from the base (azimuth = atan2(y, x) of the place point), which the arm CAN
                // reach because the target's azimuth and the gripper yaw are the same angle. The
                // grasp_frame_transform IK frame keeps the held object at the place point, just as
                // it keeps the gripper at the object on pick. 6-DOF+ arms keep the object-centric
                // GeneratePlacePose so the recorded orientation is honoured.
                const std::size_t arm_dof =
                    task_.getRobotModel()->getJointModelGroup(config_.arm_group_name)->getActiveJointModels().size();
                const bool waist_aligned = arm_dof < 6;

                std::unique_ptr<mtc::Stage> generator;
                if (waist_aligned) {
                    const double place_theta =
                        std::atan2(place_pose_.position.y, place_pose_.position.x);
                    RCLCPP_INFO(logger(), "[MtcTask:place] %zu-DOF arm: waist-aligned place, azimuth=%.1f deg",
                                arm_dof, place_theta * 180.0 / M_PI);

                    geometry_msgs::msg::PoseStamped target;
                    target.header.frame_id = config_.world_frame;
                    target.pose.position = place_pose_.position;
                    target.pose.orientation.x = 0.0;
                    target.pose.orientation.y = 0.0;
                    target.pose.orientation.z = std::sin(place_theta / 2.0);
                    target.pose.orientation.w = std::cos(place_theta / 2.0);

                    auto stage = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
                    stage->properties().set("marker_ns", "place_pose");
                    stage->setPose(target);
                    stage->setMonitoredStage(attach_object_stage);
                    generator = std::move(stage);
                } else {
                    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                    stage->properties().configureInitFrom(mtc::Stage::PARENT);
                    stage->properties().set("marker_ns", "place_pose");
                    stage->setObject(object_id_);

                    geometry_msgs::msg::PoseStamped target;
                    target.header.frame_id = config_.world_frame;
                    target.pose = place_pose_;
                    stage->setPose(target);
                    stage->setMonitoredStage(attach_object_stage);
                    generator = std::move(stage);
                }

                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(generator));
                wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
                wrapper->setMinSolutionDistance(0.1);
                wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
                wrapper->setTimeout(1.5);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
                container->insert(std::move(wrapper));
            }

            // Open gripper
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner_);
                stage->setGroup(config_.hand_group_name);
                stage->setGoal(config_.hand_open_state);
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

            // Forbid hand-object collision
            {
                // Re-enable hand<->object collision only after the retreat has physically pulled
                // the fingers clear of the placed object. The explicit allowance from the pick
                // ("allow collision (hand,object)") survives detachObject, so the retreat above
                // can still slide out past a bottle-width object the narrow wx200 gripper cannot
                // clear by opening alone. Doing this before the retreat made the stage fail with
                // 'left/right_finger_link colliding with <object>'.
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                stage->allowCollisions(object_id_,
                                       task_.getRobotModel()
                                           ->getJointModelGroup(config_.hand_group_name)
                                           ->getLinkModelNamesWithCollisionGeometry(),
                                       false);
                container->insert(std::move(stage));
            }

            // Remove collision object
            {
                // Drop the placed object from the planning scene after retreating from it (the
                // retreat above still checks against it). The scene lives in world_frame =
                // wx200/base_link, a robot-fixed frame, so a left-behind object would freeze at its
                // base-relative release pose and, once the robot drives elsewhere, collide with
                // unrelated objects detected on a later pick. The named-pose branch removes it too.
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
                stage->removeObject(object_id_);
                container->insert(std::move(stage));
            }

            task_.add(std::move(container));
        }
    }
    else
    {
        // ---- Move to named SRDF place pose ----
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
            stage->setGroup(config_.arm_group_name);
            stage->setGoal(place_pose_name_);
            task_.add(std::move(stage));
        }

        // Open gripper
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("release object", joint_planner_);
            stage->setGroup(config_.hand_group_name);
            stage->setGoal(config_.hand_open_state);
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
        stage->setGoal(config_.arm_home_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator
