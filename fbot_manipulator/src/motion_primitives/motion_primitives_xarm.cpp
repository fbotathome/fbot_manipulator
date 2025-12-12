#include "fbot_manipulator/motion_primitives_xarm.hpp"

#include <chrono>
#include <exception>

using namespace std::chrono_literals;

namespace fbot_manipulator
{

MotionPrimitivesXArm::MotionPrimitivesXArm(const rclcpp::Node::SharedPtr& node, const std::string& arm_name)
    : MotionPrimitivesBase(node, arm_name)
{
    init();
}

MotionPrimitivesXArm::MotionPrimitivesXArm(const std::string& arm_name)
    : MotionPrimitivesBase(arm_name)
{
    init();
}

void MotionPrimitivesXArm::init_service_clients()
{

    plan_joint_client_ = node_->create_client<xarm_msgs::srv::PlanJoint>("/xarm_joint_plan", rmw_qos_profile_services_default, callback_group_);
    plan_exec_client_  = node_->create_client<xarm_msgs::srv::PlanExec>("/xarm_exec_plan", rmw_qos_profile_services_default, callback_group_);
    plan_pose_client_  = node_->create_client<xarm_msgs::srv::PlanPose>("/xarm_pose_plan", rmw_qos_profile_services_default, callback_group_);
}

bool MotionPrimitivesXArm::moveToNamedTarget(const std::string& target_name)
{
    auto poses = manipulator_config_["poses"];
    if (!poses[target_name]) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] Named target '%s' not found in current manipulator config.",
                     target_name.c_str());
        return false;
    }

    auto target_joint_positions = poses[target_name].as<std::vector<double>>();

    if (!plan_joint_client_ || !plan_joint_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service not available");
        return false;
    }

    auto req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
    req->target = target_joint_positions;

    auto plan_future = plan_joint_client_->async_send_request(req);

    if (plan_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service call timed out");
        return false;
    }

    auto plan_resp = plan_future.get();
    if (!plan_resp) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service returned null response");
        return false;
    }

    if (plan_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_joint service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_joint service succeeded");

    if (plan_exec_client_) {
        if (!plan_exec_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_joint service not available, skipping execution");
            return true;
        }

    auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();

    auto exec_future = plan_exec_client_->async_send_request(exec_req);
    if (exec_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(),
                    "[MotionPrimitives] plan_exec service call timed out");
        return true;
    }
    auto exec_resp = exec_future.get();
    if (!exec_resp) {
        RCLCPP_WARN(node_->get_logger(),
                    "[MotionPrimitives] plan_exec service returned null response");
        return true;
    }

    if (exec_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_exec service succeeded");
    }

    return true;
}

bool MotionPrimitivesXArm::moveToJointTarget(const std::vector<double>& joint_positions)
{

    if (!plan_joint_client_ || !plan_joint_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service not available");
        return false;
    }

    auto req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
    req->target = joint_positions;

    auto plan_future = plan_joint_client_->async_send_request(req);

    if (plan_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service call timed out");
        return false;
    }

    auto plan_resp = plan_future.get();
    if (!plan_resp) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_joint service returned null response");
        return false;
    }

    if (plan_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_joint service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_joint service succeeded");

    if (plan_exec_client_) {
        if (!plan_exec_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service not available, skipping execution");
            return true;
        }

        auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();

        auto exec_future = plan_exec_client_->async_send_request(exec_req);
        if (exec_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service call timed out");
            return true;
        }
        auto exec_resp = exec_future.get();
        if (!exec_resp) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service returned null response");
            return true;
        }

        if (exec_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[MotionPrimitives] plan_exec service succeeded");
    }

    return true;
}

bool MotionPrimitivesXArm::moveToPose(const geometry_msgs::msg::Pose& pose)
{
    auto req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();
    req->target = pose;

    if (!plan_pose_client_ || !plan_pose_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_pose service not available");
        return false;
    }

    auto plan_future = plan_pose_client_->async_send_request(req);

    if (plan_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_pose service call timed out");
        return false;
    }

    auto plan_resp = plan_future.get();
    if (!plan_resp) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MotionPrimitives] plan_pose service returned null response");
        return false;
    }

    if (plan_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] plan_pose service failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MotionPrimitives] plan_pose service succeeded");

    if (plan_exec_client_) {
        if (!plan_exec_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service not available, skipping execution");
            return true;
        }

        auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();

        auto exec_future = plan_exec_client_->async_send_request(exec_req);
        if (exec_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service call timed out");
            return true;
        }

        auto exec_resp = exec_future.get();
        if (!exec_resp) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MotionPrimitives] plan_exec service returned null response");
            return true;
        }

        if (exec_resp->success == false) {
        RCLCPP_ERROR(node_->get_logger(),
            "[MotionPrimitives] Execution service failed");
        return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[MotionPrimitives] plan_exec service succeeded");
    }

    return true;
}

} // namespace fbot_manipulator
