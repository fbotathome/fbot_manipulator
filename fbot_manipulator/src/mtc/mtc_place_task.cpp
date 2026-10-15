#include "fbot_manipulator/mtc/mtc_place_task.hpp"

#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

namespace fbot_manipulator
{

MtcPlaceTask::MtcPlaceTask(rclcpp::Node::SharedPtr node,
                           const ManipulationGoal& goal,
                           bool approach_from_front)
    : MtcTask("place", node),
      goal_(goal),
      approach_from_front_(approach_from_front)
{
}

bool MtcPlaceTask::buildTask()
{
    task_.stages()->setName("place_" + goal_.target_id);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    geometry_msgs::msg::Vector3 pick_offset;
    pick_offset.x = 0.0;
    pick_offset.y = 0.0;
    pick_offset.z = 0.0;

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, goal_.target_id);

    // 1. Current State (object assumed already attached)
    mtc::Stage* attach_object_stage = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        attach_object_stage = stage.get();
        task_.add(std::move(stage));
    }

    // 2. CHAMA A LÓGICA COMPARTILHADA DE PLACE
    MtcSharedLogic::addPlaceStages(
        task_, goal_.target_id, goal_.place_poses.front(), attach_object_stage, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_
    );
    
    // 3. Return Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_home_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator
