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

    // Arms with fewer than 6 DOF (e.g. the 5-DOF WidowX 200) cannot reach an arbitrary grasp
    // orientation, so the fanned GenerateGraspPose mostly yields poses the arm can't achieve.
    // For those we generate a single "waist-aligned" grasp: the gripper is kept level and its
    // yaw points straight at the object (azimuth = atan2(y, x)), which the arm CAN reach because
    // the target's azimuth and the gripper yaw are the same angle. 6-DOF+ arms keep the fan.
    const std::size_t arm_dof =
        task_.getRobotModel()->getJointModelGroup(config_.arm_group_name)->getActiveJointModels().size();
    const bool waist_aligned = arm_dof < 6;

    geometry_msgs::msg::Pose object_pose;
    if (auto it = object_poses_.find(object_id_); it != object_poses_.end()) {
        object_pose = it->second;
    } else if (waist_aligned) {
        RCLCPP_WARN(logger(), "[MtcTask:pick] No stored pose for '%s'; waist-aligned grasp assumes origin",
                    object_id_.c_str());
    }
    const double grasp_theta = std::atan2(object_pose.position.y, object_pose.position.x);
    if (waist_aligned) {
        RCLCPP_INFO(logger(), "[MtcTask:pick] %zu-DOF arm: waist-aligned grasp, azimuth=%.1f deg",
                    arm_dof, grasp_theta * 180.0 / M_PI);
    }

    // The grasp is generated from (and the approach is checked against) this stage's scene.
    // For the waist-aligned grasp we allow the object<->robot collisions here so BOTH the grasp IK
    // and the collision-checked approach tolerate the expected gripper/arm<->object contact -- the
    // Interbotix gripper_bar_link mesh wraps the finger region, so an object held between the
    // fingers always overlaps it. Scoped to the robot's links only, so object<->table (world) and
    // object<->other-object collisions stay checked. (setIgnoreCollisions on ComputeIK alone would
    // not help the approach stage, which runs its own collision check from the grasp state.)
    mtc::Stage* grasp_monitor = current_state;
    if (waist_aligned) {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow object-robot collisions");
        stage->allowCollisions(object_id_, task_.getRobotModel()->getLinkModelNames(), true);
        grasp_monitor = stage.get();
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

            geometry_msgs::msg::Vector3Stamped vec;
            if (waist_aligned) {
                // Approach radially toward the object in the world frame: the gripper drives
                // outward from the base along the azimuth it is pointing. This is independent of
                // the IK-returned orientation, so it stays valid for a <6-DOF arm.
                vec.header.frame_id = config_.world_frame;
                vec.vector.x = std::cos(grasp_theta);
                vec.vector.y = std::sin(grasp_theta);
                vec.vector.z = 0.0;
            } else {
                // Move along the approach axis: X of the grasp IK frame, expressed in hand_frame.
                // (xarm6: grasp_frame_transform maps X->Z, so this is (0,0,1) as before; the Interbotix
                // ee_gripper_link is X-forward, so this is (1,0,0).)
                const Eigen::Vector3d approach_axis =
                    config_.grasp_frame_transform.linear() * Eigen::Vector3d::UnitX();
                vec.header.frame_id = config_.hand_frame;
                vec.vector.x = approach_axis.x();
                vec.vector.y = approach_axis.y();
                vec.vector.z = approach_axis.z();
            }
            stage->setDirection(vec);
            container->insert(std::move(stage));
        }

        // Generate Grasp Pose + IK
        {
            std::unique_ptr<mtc::Stage> generator;
            if (waist_aligned) {
                // Single level grasp pose: gripper yaw = grasp_theta (points straight at the
                // object), no pitch/roll. Reachable by a <6-DOF arm because the target's azimuth
                // and the gripper yaw are both grasp_theta.
                geometry_msgs::msg::PoseStamped target;
                target.header.frame_id = config_.world_frame;
                target.pose.position = object_pose.position;
                target.pose.orientation.x = 0.0;
                target.pose.orientation.y = 0.0;
                target.pose.orientation.z = std::sin(grasp_theta / 2.0);
                target.pose.orientation.w = std::cos(grasp_theta / 2.0);

                auto stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPose(target);
                // Monitor the allow-collision stage (open gripper + object collisions allowed).
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            } else {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(config_.hand_open_state);
                stage->setObject(object_id_);
                stage->setAngleDelta(config_.grasp_angle_delta);
                stage->setMonitoredStage(grasp_monitor);
                generator = std::move(stage);
            }

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(generator));
            wrapper->setMaxIKSolutions(waist_aligned ? 8 : 4);
            wrapper->setMinSolutionDistance(0.2);
            wrapper->setIKFrame(config_.grasp_frame_transform, config_.hand_frame);
            wrapper->setTimeout(5.0);
            // Waist-aligned grasps are real (oriented) grasps, so enforce collision checking;
            // the fanned path keeps its original permissive behaviour.
            wrapper->setIgnoreCollisions(!waist_aligned);
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
