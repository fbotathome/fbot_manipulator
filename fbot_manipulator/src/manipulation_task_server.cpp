#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fbot_manipulator_msgs/action/manipulation_task.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pour_task.hpp"

namespace fbot_manipulator
{

class ManipulationTaskServer : public rclcpp::Node
{
public:
    using ManipulationTaskAction = fbot_manipulator_msgs::action::ManipulationTask;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ManipulationTaskAction>;

    ManipulationTaskServer(const rclcpp::NodeOptions& options)
        : Node("manipulation_task_server", options)
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<ManipulationTaskAction>(
            this,
            "fbot_manipulator/manipulation_task",
            std::bind(&ManipulationTaskServer::handleGoal, this, _1, _2),
            std::bind(&ManipulationTaskServer::handleCancel, this, _1),
            std::bind(&ManipulationTaskServer::handleAccepted, this, _1));

        grasp_check_ = makeGraspCheckConfig();
        if (grasp_check_.enabled)
        {
            joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                grasp_check_.topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(joint_mtx_);
                    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
                        joint_positions_[msg->name[i]] = msg->position[i];
                });
            RCLCPP_INFO(get_logger(),
                        "Grasp check enabled: joint '%s' on '%s' (closed=%.4f, min opening=%.4f)",
                        grasp_check_.finger_joint.c_str(), grasp_check_.topic.c_str(),
                        grasp_check_.closed_position, grasp_check_.min_gap);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Grasp check disabled (mtc.grasp_check.enabled=false)");
        }

        RCLCPP_INFO(get_logger(), "ManipulationTaskServer ready");
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const ManipulationTaskAction::Goal> goal)
    {
        if (executing_)
        {
            RCLCPP_WARN(get_logger(), "Rejecting goal: another task is executing");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(get_logger(), "Accepting goal: task_type=%d, object='%s'",
                     goal->task_type, goal->object_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread([this, goal_handle]() { executeTask(goal_handle); }).detach();
    }

    void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle,
                         const std::string& stage, float progress)
    {
        auto feedback = std::make_shared<ManipulationTaskAction::Feedback>();
        feedback->current_stage = stage;
        feedback->progress = progress;
        goal_handle->publish_feedback(feedback);
    }

    void executeTask(const std::shared_ptr<GoalHandle> goal_handle)
    {
        executing_ = true;
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ManipulationTaskAction::Result>();
        std::string object_id = goal->object_id;

        // Add collision object from bbox
        publishFeedback(goal_handle, "Adding collision object", 0.0);

        MtcTask::Ptr mtc_task;

        switch (goal->task_type)
        {
        case ManipulationTaskAction::Goal::PICK:
            mtc_task = std::make_shared<MtcPickTask>(shared_from_this(), object_id);
            break;
        case ManipulationTaskAction::Goal::PLACE:
            // Prefer the geometric place when a pose is provided; place_pose_name then acts as
            // the fallback (tried only if the geometric place fails to plan, see below).
            if (hasGeometricPose(goal->place_pose))
            {
                mtc_task = std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose);
            }
            else
            {
                mtc_task = std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
            }
            break;
        case ManipulationTaskAction::Goal::PICK_AND_PLACE:
            if (hasGeometricPose(goal->place_pose))
            {
                mtc_task = std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose);
            }
            else
            {
                mtc_task = std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
            }
            break;
        case ManipulationTaskAction::Goal::POUR:
            mtc_task = std::make_shared<MtcPourTask>(shared_from_this(), object_id, goal->object_pose);
            break;
        default:
            result->success = false;
            result->message = "Unsupported task type: " + std::to_string(goal->task_type);
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // For PLACE the object is already attached to the gripper by the preceding pick.
        // Re-adding it as a world object with the same name creates a duplicate body that
        // collides with the attached one ("eef in collision: <id> - <id>") and breaks the
        // later detach/remove. Only add for tasks that introduce a not-yet-attached object.
        const bool object_already_attached =
            (goal->task_type == ManipulationTaskAction::Goal::PLACE);
        if (!object_already_attached)
        {
            mtc_task->addCollisionObject(object_id, goal->object_pose, goal->object_size);
        }

        // Check cancellation
        if (goal_handle->is_canceling())
        {
            if (!object_already_attached) mtc_task->removeCollisionObject(object_id);
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            executing_ = false;
            return;
        }

        // Build task
        publishFeedback(goal_handle, "Building task", 0.1);
        if (!mtc_task->buildTask())
        {
            if (!object_already_attached) mtc_task->removeCollisionObject(object_id);
            result->success = false;
            result->message = "Failed to build task";
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // Plan
        publishFeedback(goal_handle, "Planning", 0.3);
        if (!mtc_task->plan())
        {
            // Geometric place unreachable: fall back to the recorded place_pose_name (if any).
            // Only meaningful when the primary attempt used a geometric place pose.
            const bool can_fallback =
                (goal->task_type == ManipulationTaskAction::Goal::PLACE ||
                 goal->task_type == ManipulationTaskAction::Goal::PICK_AND_PLACE) &&
                hasGeometricPose(goal->place_pose) && !goal->place_pose_name.empty();

            bool recovered = false;
            if (can_fallback)
            {
                RCLCPP_WARN(get_logger(), "Geometric place unreachable; falling back to '%s'",
                            goal->place_pose_name.c_str());
                publishFeedback(goal_handle, "Planning (fallback pose)", 0.3);

                // Build a fresh named-pose task. Do NOT re-add the collision object: it is already
                // in the planning scene (and for PLACE it is attached to the gripper), so the
                // fallback task's CurrentState picks up the live state.
                auto fallback = buildFallbackTask(goal, object_id);
                if (fallback && fallback->buildTask() && fallback->plan())
                {
                    mtc_task = fallback;
                    recovered = true;
                }
            }

            if (!recovered)
            {
                if (!object_already_attached) mtc_task->removeCollisionObject(object_id);
                result->success = false;
                result->message = can_fallback ? "Planning failed (including fallback)" : "Planning failed";
                goal_handle->abort(result);
                executing_ = false;
                return;
            }
        }

        if (goal_handle->is_canceling())
        {
            if (!object_already_attached) mtc_task->removeCollisionObject(object_id);
            result->success = false;
            result->message = "Cancelled";
            goal_handle->canceled(result);
            executing_ = false;
            return;
        }

        // Execute
        publishFeedback(goal_handle, "Executing", 0.5);
        if (!mtc_task->execute())
        {
            result->success = false;
            result->message = "Execution failed";
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // Verify the grasp at the very end of a pick: if the gripper closed empty we lifted
        // nothing, so fail the action and clear the phantom (still-attached) object out of the
        // planning scene. Only PICK ends holding the object; PLACE/PICK_AND_PLACE/POUR finish
        // with the gripper open, so the finger reading would be meaningless there.
        if (goal->task_type == ManipulationTaskAction::Goal::PICK)
        {
            publishFeedback(goal_handle, "Verifying grasp", 0.9);
            std::string grasp_msg;
            if (!verifyGrasp(mtc_task, object_id, grasp_msg))
            {
                RCLCPP_WARN(get_logger(), "%s", grasp_msg.c_str());
                result->success = false;
                result->message = grasp_msg;
                goal_handle->abort(result);
                executing_ = false;
                return;
            }
        }

        // Success
        publishFeedback(goal_handle, "Done", 1.0);
        result->success = true;
        result->message = "Task completed successfully";
        goal_handle->succeed(result);

        executing_ = false;
    }

    // Tells whether the goal carries a real geometric place pose vs. only a named state.
    // A default-constructed geometry_msgs/Pose is the origin with the rosidl default
    // *identity* quaternion (0,0,0,1) -- NOT an all-zero quaternion. So a goal that only
    // sets place_pose_name still arrives with place_pose = identity-at-origin. Treat both
    // that and an all-zero quaternion as "no geometric pose"; a genuine place pose has a
    // non-zero position (or a non-trivial orientation).
    static bool hasGeometricPose(const geometry_msgs::msg::Pose& p)
    {
        const bool at_origin =
            (p.position.x == 0.0 && p.position.y == 0.0 && p.position.z == 0.0);
        const bool trivial_orientation =
            (p.orientation.x == 0.0 && p.orientation.y == 0.0 && p.orientation.z == 0.0 &&
             (p.orientation.w == 0.0 || p.orientation.w == 1.0));
        return !(at_origin && trivial_orientation);
    }

    // Per-robot gripper geometry for the post-pick grasp check.
    struct GraspCheckConfig
    {
        bool enabled = false;
        std::string topic;          // joint_states topic carrying the finger joint
        std::string finger_joint;   // finger joint whose position reveals the held width
        double closed_position = 0.0;  // finger position when fully closed (empty)
        double min_gap = 0.0;          // min opening above closed_position to count as "holding"
    };

    // Select the finger config from the arm group (set per robot in mtc_config.yaml). The finger
    // joint name and joint_states topic are robot wiring (kept here); the thresholds are tunable
    // and read from mtc.grasp_check.* in the config.
    GraspCheckConfig makeGraspCheckConfig()
    {
        std::string arm_group = "xarm6";
        get_parameter_or("mtc.arm_group_name", arm_group, arm_group);

        GraspCheckConfig cfg;
        if (arm_group == "interbotix_arm")
        {
            // wx200: 'left_finger' settles at ~0.022 m fully closed and 0.037 m fully open, so a
            // grasped object holds the finger above closed; require a small opening to avoid noise.
            cfg.topic = "wx200/joint_states";
            cfg.finger_joint = "left_finger";
        }
        else
        {
            // TODO(xarm6): set the gripper finger joint name and joint_states topic. The check
            // stays off until then (guarded below) so xarm6 picks aren't failed by missing wiring.
        }

        get_parameter_or("mtc.grasp_check.enabled", cfg.enabled, cfg.enabled);
        get_parameter_or("mtc.grasp_check.closed_position", cfg.closed_position, cfg.closed_position);
        get_parameter_or("mtc.grasp_check.min_gap", cfg.min_gap, cfg.min_gap);

        if (cfg.enabled && cfg.finger_joint.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "mtc.grasp_check.enabled is true but no finger joint is wired for this "
                        "robot; disabling grasp check");
            cfg.enabled = false;
        }
        return cfg;
    }

    // Read the latest cached position of the configured finger joint. Returns false if no
    // joint_states carrying that joint has been received yet.
    bool latestFingerPosition(double& position)
    {
        std::lock_guard<std::mutex> lk(joint_mtx_);
        auto it = joint_positions_.find(grasp_check_.finger_joint);
        if (it == joint_positions_.end()) return false;
        position = it->second;
        return true;
    }

    // True if the gripper appears to hold an object (finger settled above fully-closed). On an
    // empty grip, detaches+removes the phantom object from the scene and fills 'message'. If the
    // check is disabled or no finger reading is available, passes (true) so a setup gap never
    // turns a good pick into a failure.
    bool verifyGrasp(const MtcTask::Ptr& task, const std::string& object_id, std::string& message)
    {
        if (!grasp_check_.enabled) return true;

        // Let the gripper settle, then take the most recent finger reading.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        double position = 0.0;
        bool have_reading = false;
        const auto deadline = now() + rclcpp::Duration::from_seconds(1.0);
        do
        {
            have_reading = latestFingerPosition(position);
            if (have_reading) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } while (now() < deadline);

        if (!have_reading)
        {
            RCLCPP_WARN(get_logger(),
                        "Grasp check: no '%s' on '%s'; skipping verification",
                        grasp_check_.finger_joint.c_str(), grasp_check_.topic.c_str());
            return true;
        }

        const double opening = position - grasp_check_.closed_position;
        RCLCPP_INFO(get_logger(),
                    "Grasp check: %s=%.4f (closed=%.4f, opening=%.4f, min=%.4f)",
                    grasp_check_.finger_joint.c_str(), position,
                    grasp_check_.closed_position, opening, grasp_check_.min_gap);

        if (opening <= grasp_check_.min_gap)
        {
            task->detachAndRemoveObject(object_id);
            message = "grasp verification failed: gripper closed empty (" +
                      grasp_check_.finger_joint + "=" + std::to_string(position) + ")";
            return false;
        }
        return true;
    }

    // Build the named-pose place variant used as a fallback when the geometric place can't plan.
    MtcTask::Ptr buildFallbackTask(const std::shared_ptr<const ManipulationTaskAction::Goal>& goal,
                                   const std::string& object_id)
    {
        switch (goal->task_type)
        {
        case ManipulationTaskAction::Goal::PLACE:
            return std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
        case ManipulationTaskAction::Goal::PICK_AND_PLACE:
            return std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
        default:
            return nullptr;
        }
    }

    rclcpp_action::Server<ManipulationTaskAction>::SharedPtr action_server_;
    bool executing_ = false;

    GraspCheckConfig grasp_check_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex joint_mtx_;
    std::map<std::string, double> joint_positions_;
};

} // namespace fbot_manipulator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<fbot_manipulator::ManipulationTaskServer>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
