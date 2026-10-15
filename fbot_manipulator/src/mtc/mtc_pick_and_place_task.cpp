#include "fbot_manipulator/mtc/mtc_pick_and_place_task.hpp"
#include "fbot_manipulator/mtc/mtc_shared_logic.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fbot_manipulator
{

MtcPickAndPlaceTask::MtcPickAndPlaceTask(rclcpp::Node::SharedPtr node,
                                         const ManipulationGoal& goal,
                                         bool approach_from_front)
    : MtcTask("pick_and_place", node),
      goal_(goal),
      approach_from_front_(approach_from_front)
{
}

bool MtcPickAndPlaceTask::buildTask()
{
    task_.stages()->setName("pick_and_place_" + goal_.target_id);
    task_.loadRobotModel(node_);

    task_.setProperty("group", config_.arm_group_name);
    task_.setProperty("eef", config_.hand_group_name);
    task_.setProperty("ik_frame", config_.hand_frame);

    MtcSharedLogic::setupWorkspace(this, goal_.objects_scene, goal_.pick_offset, goal_.target_id);

    // 1. Current State
    mtc::Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state = stage.get();
        task_.add(std::move(stage));
    }

    if (object_poses_.find(goal_.target_id) == object_poses_.end()) {
        RCLCPP_ERROR(logger(), "ERROR: target_id '%s' was not found!", goal_.target_id.c_str());
        return false;
    }

    // O offset é definido em relação ao frame do braço/base, mas a pose efetiva usada pelo
    // MTC deve ser ajustada no frame da câmera. Então convertemos o offset para o frame da
    // câmera antes de aplicá-lo à pose do objeto.
    geometry_msgs::msg::Pose object_pose = object_poses_[goal_.target_id];

    geometry_msgs::msg::Vector3 translated_offset = goal_.pick_offset;
    const std::vector<std::string> camera_frames = {
        "camera_link",
        "camera_color_optical_frame",
        "camera_optical_frame",
        "realsense_link"
    };

    tf2_ros::Buffer tf_buffer(node_->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer, node_, false);

    for (const auto& camera_frame : camera_frames) {
        if (!tf_buffer.canTransform(camera_frame, config_.world_frame, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
            continue;
        }

        geometry_msgs::msg::Vector3Stamped offset_in_arm;
        offset_in_arm.header.frame_id = config_.world_frame;
        offset_in_arm.vector = goal_.pick_offset;

        geometry_msgs::msg::Vector3Stamped offset_in_camera;
        const auto transform = tf_buffer.lookupTransform(camera_frame, config_.world_frame, tf2::TimePointZero);
        tf2::doTransform(offset_in_arm, offset_in_camera, transform);

        translated_offset = offset_in_camera.vector;
        break;
    }

    object_pose.position.x += translated_offset.x;
    object_pose.position.y += translated_offset.y;
    object_pose.position.z += translated_offset.z;

    // 2. CHAMA O PICK E GUARDA O RESULTADO
    mtc::Stage* attach_stage = MtcSharedLogic::addPickStages(
        task_, goal_.target_id, object_pose, current_state, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_
    );

    // 4. CHAMA O PLACE
    MtcSharedLogic::addPlaceStages(
        task_, goal_.target_id, goal_.place_poses.front(), attach_stage, 
        config_, pipeline_planner_, cartesian_planner_, joint_planner_, logger(),
        approach_from_front_
    );

    // 5. Return Home Final
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", pipeline_planner_);
        stage->setGroup(config_.arm_group_name);
        stage->setGoal(config_.arm_ready_state);
        task_.add(std::move(stage));
    }

    return true;
}

} // namespace fbot_manipulator