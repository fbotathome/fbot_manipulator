#pragma once

#include "fbot_manipulator/mtc/mtc_task.hpp"

namespace fbot_manipulator
{

class MtcPourTask : public MtcTask
{
public:
  MtcPourTask(rclcpp::Node::SharedPtr node,
              const std::string& object_id,
              const geometry_msgs::msg::Pose& object_pose);

  bool buildTask() override;

private:
  std::string object_id_;
  geometry_msgs::msg::Pose object_pose_;
};

} // namespace fbot_manipulator
