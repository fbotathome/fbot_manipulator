#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <fbot_manipulator_msgs/action/manipulation_task.hpp>

#include "fbot_manipulator/mtc/mtc_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_task.hpp"
#include "fbot_manipulator/mtc/mtc_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"

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
            if (!goal->place_pose_name.empty())
                mtc_task = std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
            else
                mtc_task = std::make_shared<MtcPlaceTask>(shared_from_this(), object_id, goal->place_pose);
            break;
        case ManipulationTaskAction::Goal::PICK_AND_PLACE:
            if (!goal->place_pose_name.empty())
                mtc_task = std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose_name);
            else
                mtc_task = std::make_shared<MtcPickAndPlaceTask>(shared_from_this(), object_id, goal->place_pose);
            break;
        default:
            result->success = false;
            result->message = "Unsupported task type: " + std::to_string(goal->task_type);
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        mtc_task->addCollisionObject(object_id, goal->object_pose, goal->object_size);

        // Check cancellation
        if (goal_handle->is_canceling())
        {
            mtc_task->removeCollisionObject(object_id);
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
            mtc_task->removeCollisionObject(object_id);
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
            mtc_task->removeCollisionObject(object_id);
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            executing_ = false;
            return;
        }

        if (goal_handle->is_canceling())
        {
            mtc_task->removeCollisionObject(object_id);
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

        // Success
        publishFeedback(goal_handle, "Done", 1.0);
        result->success = true;
        result->message = "Task completed successfully";
        goal_handle->succeed(result);

        executing_ = false;
    }

    rclcpp_action::Server<ManipulationTaskAction>::SharedPtr action_server_;
    bool executing_ = false;
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
