#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPlaceTask : public MtcTask
{
public:
    MtcPlaceTask(rclcpp::Node::SharedPtr node,
                 const ManipulationGoal& goal,
                 bool approach_from_front = false);

    bool buildTask() override;

private:
    ManipulationGoal goal_; 
    bool approach_from_front_;
};

} // namespace fbot_manipulator
