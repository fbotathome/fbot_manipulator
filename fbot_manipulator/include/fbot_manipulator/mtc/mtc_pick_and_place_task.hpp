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

    MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                        const std::string& object_id,
                        const std::string& place_pose_name);

    bool buildTask() override;

private:
    std::string object_id_;
    geometry_msgs::msg::Pose place_pose_;
    std::string place_pose_name_;
};

} // namespace fbot_manipulator
