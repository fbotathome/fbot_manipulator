#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPickTask : public MtcTask
{
public:
    MtcPickTask(rclcpp::Node::SharedPtr node,
                const std::string& object_id);

    bool buildTask() override;

private:
    std::string object_id_;
};

} // namespace fbot_manipulator
