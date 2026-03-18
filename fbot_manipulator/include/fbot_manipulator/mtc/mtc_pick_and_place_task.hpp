#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPickAndPlaceTask : public MtcTask
{
public:
    MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                        const std::string& object_id,
                        const geometry_msgs::msg::Pose& place_pose);

    bool buildTask() override;

private:
    std::string object_id_;
    geometry_msgs::msg::Pose place_pose_;
};

} // namespace fbot_manipulator
