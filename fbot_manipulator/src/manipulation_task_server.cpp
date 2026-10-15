#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fbot_manipulator_msgs/action/manipulation_task.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_load_cargo_task.hpp"
#include "fbot_manipulator/mtc/mtc_unload_cargo_task.hpp"

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

        RCLCPP_INFO(get_logger(),
                    "Accepting goal: task_type=%d, target_ids=%zu, cargo_indices=%zu, approach_from_front=%s",
                    goal->task_type, goal->target_ids.size(), goal->cargo_indices.size(),
                    goal->approach_from_front ? "true" : "false");
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
        auto action_goal = goal_handle->get_goal();
        auto result = std::make_shared<ManipulationTaskAction::Result>();

        // 1. Validação de segurança dos arrays
        size_t num_objects = action_goal->object_ids.size();
        if (action_goal->object_poses.size() != num_objects || action_goal->object_sizes.size() != num_objects) {
            result->success = false;
            result->message = "Detection Array size inconsistent";
            result->failed_target_id = "";
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        if (action_goal->task_type == ManipulationTaskAction::Goal::LOAD_CARGO &&
            (action_goal->target_ids.size() != action_goal->cargo_indices.size() || action_goal->target_ids.empty()))
        {
            result->success = false;
            result->message = "LOAD_CARGO requires target_ids and cargo_indices to have the same non-empty length";
            result->failed_target_id = action_goal->target_ids.empty() ? "" : action_goal->target_ids.front();
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // 2. Empacotando o objetivo na nova estrutura interna
        fbot_manipulator::ManipulationGoal internal_goal;
        internal_goal.task_type = action_goal->task_type;
        internal_goal.target_ids.reserve(action_goal->target_ids.size());
        for (const auto& target_id : action_goal->target_ids) {
            internal_goal.target_ids.push_back(target_id);
        }
        if (!internal_goal.target_ids.empty()) {
            internal_goal.target_id = internal_goal.target_ids.front();
        }

        internal_goal.cargo_indices.reserve(action_goal->cargo_indices.size());
        for (const auto& cargo_index : action_goal->cargo_indices) {
            internal_goal.cargo_indices.push_back(static_cast<int>(cargo_index));
        }
        if (!internal_goal.cargo_indices.empty()) {
            internal_goal.cargo_id = internal_goal.cargo_indices.front();
        }

        internal_goal.pick_offset = action_goal->pick_offset;
        
        for (const auto& place_pose : action_goal->place_poses) {
            internal_goal.place_poses.push_back(place_pose);
        }
        if (!internal_goal.place_poses.empty()) {
            internal_goal.place_pose = internal_goal.place_poses.front();
        }

        for(size_t i = 0; i < num_objects; i++) {
            fbot_manipulator::ObjectDetection obj;
            obj.id = action_goal->object_ids[i];
            obj.pose = action_goal->object_poses[i];
            obj.size = action_goal->object_sizes[i];
            internal_goal.objects_scene.push_back(obj);
        }
      
        publishFeedback(goal_handle, "Initializing task", 0.0);

        MtcTask::Ptr mtc_task;

        // Flag lida do goal e repassada para Pick, Place e PickAndPlace
        const bool approach_from_front = action_goal->approach_from_front;

        // 3. Instanciando as tarefas passando a struct unificada
        switch (internal_goal.task_type)
        {
        case ManipulationTaskAction::Goal::PICK:
            mtc_task = std::make_shared<MtcPickTask>(shared_from_this(), internal_goal, approach_from_front);
            break;
        case ManipulationTaskAction::Goal::PLACE:
            mtc_task = std::make_shared<MtcPlaceTask>(shared_from_this(), internal_goal, approach_from_front);
            break;
        case ManipulationTaskAction::Goal::PICK_AND_PLACE:
            mtc_task = std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), internal_goal, approach_from_front);
            break;
        case ManipulationTaskAction::Goal::LOAD_CARGO:
            mtc_task = std::make_shared<MtcLoadCargoTask>(shared_from_this(), internal_goal);
            break;
        case ManipulationTaskAction::Goal::UNLOAD_CARGO:
            mtc_task = std::make_shared<MtcUnloadCargoTask>(shared_from_this(), internal_goal);
            break;
        default:
            result->success = false;
            result->message = "Unsupported task type: " + std::to_string(internal_goal.task_type);
            result->failed_target_id = internal_goal.target_id;
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Cancelled before building";
            result->failed_target_id = internal_goal.target_id;
            goal_handle->canceled(result);
            executing_ = false;
            return;
        }

        // Build task
        publishFeedback(goal_handle, "Building task", 0.1);
        if (!mtc_task->buildTask())
        {
            result->success = false;
            result->message = "Failed to build task";
            result->failed_target_id = internal_goal.target_id;
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // Plan
        publishFeedback(goal_handle, "Planning", 0.3);
        if (!mtc_task->plan())
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            
            auto result = std::make_shared<fbot_manipulator_msgs::action::ManipulationTask::Result>();
            result->success = false;
            
            std::string failed_id = mtc_task->firstFailedTargetId();
            
            result->failed_target_id = failed_id.empty() ? internal_goal.target_id : failed_id; 
            result->message = "Planning failed for target '" + result->failed_target_id + "'";
            
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Cancelled before execution";
            result->failed_target_id = internal_goal.target_id;
            goal_handle->canceled(result);
            executing_ = false;
            return;
        }

        // Execute
        publishFeedback(goal_handle, "Executing", 0.5);
        if (!mtc_task->execute())
        {
            result->success = false;
            result->failed_target_id = internal_goal.target_id;
            result->message = "Execution failed for target '" + internal_goal.target_id + "'";
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        // Verify the grasp
        if (internal_goal.task_type == ManipulationTaskAction::Goal::PICK || 
            internal_goal.task_type == ManipulationTaskAction::Goal::UNLOAD_CARGO)
        {
            publishFeedback(goal_handle, "Verifying grasp", 0.9);
            std::string grasp_msg;
            if (!verifyGrasp(mtc_task, internal_goal.target_id, grasp_msg))
            {
                RCLCPP_WARN(get_logger(), "%s", grasp_msg.c_str());
                result->success = false;
                result->failed_target_id = internal_goal.target_id;
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
        result->failed_target_id = "";
        goal_handle->succeed(result);

        if (mtc_task) {
            mtc_task->clearPlanningScene();
        }

        executing_ = false;
    }

    struct GraspCheckConfig
    {
        bool enabled = false;
        std::string topic;          
        std::string finger_joint;   
        double closed_position = 0.0;  
        double min_gap = 0.0;          
    };

    GraspCheckConfig makeGraspCheckConfig()
    {
        std::string arm_group = "xarm6";
        get_parameter_or("mtc.arm_group_name", arm_group, arm_group);

        GraspCheckConfig cfg;
        if (arm_group == "interbotix_arm")
        {
            cfg.topic = "wx200/joint_states";
            cfg.finger_joint = "left_finger";
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

    bool latestFingerPosition(double& position)
    {
        std::lock_guard<std::mutex> lk(joint_mtx_);
        auto it = joint_positions_.find(grasp_check_.finger_joint);
        if (it == joint_positions_.end()) return false;
        position = it->second;
        return true;
    }

    bool verifyGrasp(const MtcTask::Ptr& task, const std::string& target_id, std::string& message)
    {
        if (!grasp_check_.enabled) return true;

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
            task->detachAndRemoveObject(target_id);
            message = "grasp verification failed: gripper closed empty (" +
                      grasp_check_.finger_joint + "=" + std::to_string(position) + ")";
            return false;
        }
        return true;
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
