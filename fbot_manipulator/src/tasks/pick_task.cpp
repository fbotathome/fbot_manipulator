#include "fbot_manipulator/tasks/pick_task.hpp"

namespace fbot_manipulator
{

PickTask::PickTask(rclcpp::Node::SharedPtr node,
                   std::shared_ptr<MotionPrimitives> motion)
    : Task(node, motion)
{
    // Constructor — no additional setup yet
}


bool PickTask::configure()
{
    info("configure() not implemented yet");
    return true;
}


bool PickTask::execute()
{
    info("execute() not implemented yet");
    return false;
}

} // namespace fbot_manipulator