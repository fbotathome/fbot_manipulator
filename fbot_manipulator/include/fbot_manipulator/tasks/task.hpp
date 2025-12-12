#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "fbot_manipulator/motion_primitives.hpp"

namespace fbot_manipulator
{

/**
 * @brief Base class for all high-level manipulation tasks
 *
 * Each derived class must:
 *  - implement run()
 *  - implement name()
 *  
 * Derived tasks typically use MotionPrimitives for robot actions.
 */

class Task
{
public:
    using Ptr = std::shared_ptr<Task>;

    /**
     * @brief Constructor
     * @param node: ROS2 node
     * @param motion: shared interface for robot actions
     */

    Task(rclcpp::Node::SharedPtr node,
         std::shared_ptr<MotionPrimitives> motion)
        : node_(node), motion_(motion)
    {}

    virtual ~Task() = default;

    /**
     * @brief Optional configuration
     * Default: no-op
     * @return true if configuration succeeded
     */
    virtual bool configure()
    {
        return true;
    }

    /**
     * @brief Execute the task’s full behavior.
     * This must be implemented by every derived task.
     * 
     * Return:
     *  - true = task succeeded
     *  - false = task failed
     */
    virtual bool execute() = 0;

    /**
     * @brief Task name
     */
    virtual std::string name() const = 0;

protected:
    /// ROS node
    rclcpp::Node::SharedPtr node_;

    /// Interface for robot motions (MoveIt2)
    std::shared_ptr<MotionPrimitives> motion_;

    // Logging helpers
    inline rclcpp::Logger logger() const { return node_->get_logger(); }

    inline void info(const std::string& msg) const
    { RCLCPP_INFO(logger(), "[Task:%s] %s", name().c_str(), msg.c_str()); }

    inline void warn(const std::string& msg) const
    { RCLCPP_WARN(logger(), "[Task:%s] %s", name().c_str(), msg.c_str()); }

    inline void error(const std::string& msg) const
    { RCLCPP_ERROR(logger(), "[Task:%s] %s", name().c_str(), msg.c_str()); }
};

} // namespace fbot_manipulator
