#pragma once

#include <string>
#include <memory>

#include "fbot_manipulator/task.hpp"          
#include "geometry_msgs/msg/pose.hpp"        

namespace fbot_manipulator
{

/**
 * @brief Pick task
 *
 * This class performs the following pick behavior:
 *   - move to pre-grasp pose
 *   - open gripper
 *   - move to grasp pose
 *   - close gripper
 *   - lift object
 *
 * Parameters (optionally configured):
 *   - target_pose: pose where the object should be picked
 *   - pre_pick_named_target: MoveIt2 named pose
 */

class PickTask : public Task
{
public:
    using Ptr = std::shared_ptr<PickTask>;

    /**
     * @brief Constructor
     */
    PickTask(rclcpp::Node::SharedPtr node,
             std::shared_ptr<MotionPrimitives> motion)
        : Task(node, motion)
    {}

    /**
     * @brief Task name
     */
    std::string name() const override
    {
        return "pick";
    }

    /**
     * @brief Optional configuration step
     *
     * Loads parameters such as:
     *   - pre_pick pose name
     *   - lift height
     *
     * Return true if configuration succeeded.
     */
    bool configure() override;

    /**
     * @brief Execute the pick sequence
     */
    bool execute() override;

    /// Optional setter for object pose (in case you don’t load via YAML)
    void setTargetPose(const geometry_msgs::msg::Pose& pose)
    {
        target_pose_ = pose;
        has_target_pose_ = true;
    }

private:

    /// If true, the user provided a pose manually
    bool has_target_pose_ = false;

    /// Pose where robot should perform the grasp
    geometry_msgs::msg::Pose target_pose_;

    /// MoveIt named target for the approach pose (e.g., "pre_pick")
    std::string pre_pick_named_pose_ = "pre_pick";

    /// Lift height after grasp
    double lift_distance_ = 0.10; // 10 cm
};

} // namespace fbot_manipulator
