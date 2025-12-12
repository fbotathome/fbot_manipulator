#include "fbot_manipulator/motion_primitives_base.hpp"

#include <chrono>
#include <exception>

using namespace std::chrono_literals;

namespace fbot_manipulator
{

MotionPrimitivesBase::MotionPrimitivesBase(const rclcpp::Node::SharedPtr& node, const std::string& arm_name)
    : node_(node), arm_name_(arm_name)
{
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

MotionPrimitivesBase::MotionPrimitivesBase(const std::string& arm_name)
    : arm_name_(arm_name)
{
    node_ = rclcpp::Node::make_shared("motion_primitives_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void MotionPrimitivesBase::init()
{
    load_config();
    init_action_clients();
    init_service_clients(); 
}

void MotionPrimitivesBase::load_config()
{
    std::string pkg_path = ament_index_cpp::get_package_share_directory("fbot_manipulator");
    std::string config_path = pkg_path + "/config/" + arm_name_ + "/manipulator_config.yaml";
    manipulator_config_ = Utils::readYaml(config_path);
}

void MotionPrimitivesBase::init_action_clients()
{
    std::string action_suffix;
    try {
        action_suffix = manipulator_config_["moveit_controllers"]["gripper_traj"].as<std::string>();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Failed to read gripper action name from YAML: %s",
                     e.what());
        throw;
    }

    std::string gripper_action_name = "/" + action_suffix + "/gripper_action";

    gripper_action_client_ =
        rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node_,
            gripper_action_name,
            callback_group_);
}

bool MotionPrimitivesBase::setGripperPosition(double position)
{
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Gripper action server not available!");
        return false;
    }

    if (position < 0.0 || position > 0.9) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Gripper position %.3f out of range [0.0, 0.9]",
                     position);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitivesBase] Setting gripper position to %.2f", position);

    control_msgs::action::GripperCommand::Goal goal_msg;
    control_msgs::msg::GripperCommand command;
    command.position = position;
    command.max_effort = 50.0;
    goal_msg.command = command;

    auto send_goal_options =
        rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

    auto goal_handle_future =
        gripper_action_client_->async_send_goal(goal_msg, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Failed to send gripper goal. Timed out.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Gripper goal rejected");
        return false;
    }

    auto result_future = gripper_action_client_->async_get_result(goal_handle);

    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Failed to get gripper result. Timed out.");
        return false;
    }

    auto action_result = result_future.get();

    if (action_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitivesBase] Gripper goal failed with status code: %d",
                     static_cast<int>(action_result.code));
        return false;
    }

    if (action_result.result->reached_goal) {
        RCLCPP_INFO(node_->get_logger(),
                    "[MotionPrimitivesBase] Successfully reached commanded gripper position %.3f (Final Position: %.3f)",
                    position, action_result.result->position);
        return true;
    } else if (action_result.result->stalled) {
        RCLCPP_WARN(node_->get_logger(),
                    "[MotionPrimitivesBase] Gripper stalled at position %.3f while trying to reach %.3f",
                    action_result.result->position, position);
        return false;
    } else {
        RCLCPP_WARN(node_->get_logger(),
                     "[MotionPrimitivesBase] Gripper goal completed but did not reach target position %.3f (Final Position: %.3f)",
                     position, action_result.result->position);
        return true;
    }
}

} // namespace fbot_manipulator
