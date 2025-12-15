#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fbot_manipulator_msgs/srv/move_gripper.hpp>
#include <fbot_manipulator_msgs/srv/move_to_named_target.hpp>
#include <fbot_manipulator_msgs/srv/move_joint.hpp>
#include <fbot_manipulator_msgs/srv/move_to_pose.hpp>

#include "fbot_manipulator/motion_primitives_base.hpp"
#include "fbot_manipulator/motion_primitives_xarm.hpp"

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

namespace fbot_manipulator
{
/**
 *  @brief Interface node for manipulation services.
 */
class ManipulationInterface
{
public:
    /**
     * @brief Constructor 
     */
    ManipulationInterface(rclcpp::Node::SharedPtr node);
    ~ManipulationInterface() {};

private:
    /**
     * @brief Service callbacks
     */
    void setGripperPositionCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveGripper::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveGripper::Response> res);
    void moveToNamedTargetCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveToNamedTarget::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveToNamedTarget::Response> res);
    void moveToJointTargetCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveJoint::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveJoint::Response> res);
    void moveToPoseCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveToPose::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveToPose::Response> res);

private:
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<MotionPrimitivesBase> motion_primitives_;

    rclcpp::Service<fbot_manipulator_msgs::srv::MoveGripper>::SharedPtr set_gripper_position_server_;
    rclcpp::Service<fbot_manipulator_msgs::srv::MoveToNamedTarget>::SharedPtr move_to_named_target_server_;
    rclcpp::Service<fbot_manipulator_msgs::srv::MoveJoint>::SharedPtr move_joint_server_;
    rclcpp::Service<fbot_manipulator_msgs::srv::MoveToPose>::SharedPtr move_to_pose_server_;
};

// ctor
ManipulationInterface::ManipulationInterface(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    std::string arm_type;
    node_->get_parameter_or("arm_type", arm_type, std::string("xarm6"));

    RCLCPP_INFO(node_->get_logger(), "\033[1;34mnamespace: %s, arm_type: %s\033[0m",
            node_->get_namespace(), arm_type.c_str());

    if (arm_type == "xarm6") {
        motion_primitives_ = std::make_shared<MotionPrimitivesXArm>(node_, arm_type);
    } 
    // add more arms here:
    // else if (arm_type == "ur5") { motion_primitives_ = std::make_shared<MotionPrimitivesUR5>(node_, arm_type); }
    else {
        RCLCPP_ERROR(node_->get_logger(),
                     "[ManipulationInterface] Unsupported arm_type '%s'.",
                     arm_type.c_str());
        throw std::runtime_error("Unsupported arm_type: " + arm_type);
    }

    set_gripper_position_server_ = node_->create_service<fbot_manipulator_msgs::srv::MoveGripper>("set_gripper_position", BIND_CLS_CB(&ManipulationInterface::setGripperPositionCb));

    move_to_named_target_server_ = node_->create_service<fbot_manipulator_msgs::srv::MoveToNamedTarget>("move_to_named_target", BIND_CLS_CB(&ManipulationInterface::moveToNamedTargetCb));

    move_joint_server_ = node_->create_service<fbot_manipulator_msgs::srv::MoveJoint>("move_joint", BIND_CLS_CB(&ManipulationInterface::moveToJointTargetCb));

    move_to_pose_server_ = node_->create_service<fbot_manipulator_msgs::srv::MoveToPose>("move_to_pose", BIND_CLS_CB(&ManipulationInterface::moveToPoseCb));
}

void ManipulationInterface::setGripperPositionCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveGripper::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveGripper::Response> res)
{
    bool success = false;
    if (motion_primitives_) {
        success = motion_primitives_->setGripperPosition(req->position);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "motion_primitives_ is null");
    }
    res->success = success;
    res->message = success ? "Gripper moved successfully." : "Failed to move gripper.";
}

void ManipulationInterface::moveToNamedTargetCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveToNamedTarget::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveToNamedTarget::Response> res)
{
    bool success = false;
    if (motion_primitives_) {
        success = motion_primitives_->moveToNamedTarget(req->target_name);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "motion_primitives_ is null");
    }
    res->success = success;
    res-> message = success ? "Moved to named target successfully." : "Failed to move to named target.";
}

void ManipulationInterface::moveToJointTargetCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveJoint::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveJoint::Response> res)
{
    bool success = false;
    if (motion_primitives_) {
        success = motion_primitives_->moveToJointTarget(req->joint_positions);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "motion_primitives_ is null");
    }
    res->success = success;
    res->message = success ? "Moved to joint target successfully." : "Failed to move to joint target.";
}

void ManipulationInterface::moveToPoseCb(const std::shared_ptr<fbot_manipulator_msgs::srv::MoveToPose::Request> req, std::shared_ptr<fbot_manipulator_msgs::srv::MoveToPose::Response> res)
{
    bool success = false;
    if (motion_primitives_) {
        success = motion_primitives_->moveToPose(req->pose);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "motion_primitives_ is null");
    }
    res->success = success;
    res->message = std::string(success ? "Moved to pose successfully: " : "Failed to move to pose: ")
        + "pos(" + std::to_string(req->pose.position.x) + ", " + std::to_string(req->pose.position.y) + ", " + std::to_string(req->pose.position.z) + ") "
        + "ori(" + std::to_string(req->pose.orientation.x) + ", " + std::to_string(req->pose.orientation.y) + ", " + std::to_string(req->pose.orientation.z) + ", " + std::to_string(req->pose.orientation.w) + ")";
}

} // namespace fbot_manipulator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("manipulator_interface_node", node_options);

    fbot_manipulator::ManipulationInterface manipulation_interface_node(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    

    RCLCPP_INFO(node->get_logger(), "Keyboard interrupt, shutting down.\n");
    rclcpp::shutdown();
    return 0;
}