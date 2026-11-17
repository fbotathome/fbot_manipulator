#pragma once

#include <string>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>

namespace fbot_manipulator
{
namespace mtc = moveit::task_constructor;

/**
 * @brief Grasp pose generator that only proposes approaches from the robot's side of the object.
 *
 * The stock GenerateGraspPose rotates the grasp a full turn around the object, so most of its
 * poses approach from the far side and can never be reached. This stage centres the rotation on
 * the direction from a reference link (the arm base) to the object and spawns poses within
 * +-angle_half_range of it, the straight-on approach first and then alternating outwards.
 * Each pose costs its angular offset, so straight-on grasps are also preferred among solutions.
 */
class FacingGraspPoseGenerator : public mtc::stages::GenerateGraspPose
{
public:
    FacingGraspPoseGenerator(const std::string& name = "generate grasp pose");

    void init(const moveit::core::RobotModelConstPtr& robot_model) override;
    void compute() override;

    // Link the approach direction is measured from (a link on the arm base axis).
    void setReferenceLink(const std::string& link) { setProperty("reference_link", link); }
    // Largest angular offset (rad) from the straight-on approach, to either side.
    void setAngleHalfRange(double half_range) { setProperty("angle_half_range", half_range); }
};

} // namespace fbot_manipulator
