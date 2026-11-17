#include "fbot_manipulator/mtc/facing_grasp_pose_generator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <boost/any.hpp>
#include <Eigen/Geometry>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/task_constructor/storage.h>
#include <rviz_marker_tools/marker_creation.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace fbot_manipulator
{

FacingGraspPoseGenerator::FacingGraspPoseGenerator(const std::string& name)
    : mtc::stages::GenerateGraspPose(name)
{
    auto& p = properties();
    p.declare<std::string>("reference_link", "link the approach direction is measured from");
    p.declare<double>("angle_half_range", M_PI, "max angular offset from the straight-on approach (rad)");
}

void FacingGraspPoseGenerator::init(const moveit::core::RobotModelConstPtr& robot_model)
{
    mtc::InitStageException errors;
    try {
        mtc::stages::GenerateGraspPose::init(robot_model);
    } catch (mtc::InitStageException& e) {
        errors.append(e);
    }

    const auto& props = properties();
    const std::string& reference_link = props.get<std::string>("reference_link");
    if (!robot_model->hasLinkModel(reference_link))
        errors.push_back(*this, "unknown reference link: " + reference_link);
    if (props.get<double>("angle_half_range") <= 0.)
        errors.push_back(*this, "angle_half_range must be positive");

    if (errors)
        throw errors;
}

void FacingGraspPoseGenerator::compute()
{
    if (upstream_solutions_.empty())
        return;
    planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

    const auto& props = properties();
    const std::string& object = props.get<std::string>("object");
    const moveit::core::JointModelGroup* eef_jmg =
        scene->getRobotModel()->getEndEffector(props.get<std::string>("eef"));

    // Candidates are checked with the gripper in its pre-grasp (open) posture.
    moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
    try {
        const std::string pregrasp = boost::any_cast<std::string>(props.property("pregrasp").value());
        if (!robot_state.setToDefaultValues(eef_jmg, pregrasp)) {
            spawn(mtc::InterfaceState{ scene }, mtc::SubTrajectory::failure("unknown pregrasp state '" + pregrasp + "'"));
            return;
        }
    } catch (const boost::bad_any_cast&) {
        spawn(mtc::InterfaceState{ scene }, mtc::SubTrajectory::failure("pregrasp must be a named state"));
        return;
    }
    robot_state.update();

    // Angle about the rotation axis at which the grasp approach (the pose's x axis) points from
    // the reference link towards the object. Everything is expressed in the object frame.
    const Eigen::Vector3d axis = props.get<Eigen::Vector3d>("rotation_axis").normalized();
    const Eigen::Isometry3d& object_tf = scene->getFrameTransform(object);
    const Eigen::Vector3d reference =
        robot_state.getGlobalLinkTransform(props.get<std::string>("reference_link")).translation();
    const Eigen::Vector3d to_object = object_tf.linear().transpose() * (object_tf.translation() - reference);

    const Eigen::Vector3d approach_at_zero = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(axis) * axis;
    const Eigen::Vector3d facing = to_object - to_object.dot(axis) * axis;
    double center_angle = 0.0;
    if (approach_at_zero.norm() > 1e-6 && facing.norm() > 1e-6)
        center_angle = std::atan2(axis.dot(approach_at_zero.cross(facing)), approach_at_zero.dot(facing));

    // Offsets from the straight-on approach: 0, +d, -d, +2d, -2d, ... up to the half range.
    const double delta = std::abs(props.get<double>("angle_delta"));
    const double half_range = std::min(props.get<double>("angle_half_range"), M_PI);
    std::vector<double> offsets{ 0.0 };
    for (int k = 1; k * delta <= half_range + 1e-9; ++k) {
        offsets.push_back(k * delta);
        // Near a full turn, -k*delta is the same pose as +k*delta.
        if (2. * M_PI - 2. * k * delta > delta / 2.)
            offsets.push_back(-k * delta);
    }

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = object;
    for (const double offset : offsets) {
        const double angle = center_angle + offset;
        const Eigen::Isometry3d target_pose(Eigen::AngleAxisd(angle, axis));

        mtc::InterfaceState state(scene);
        target_pose_msg.pose = tf2::toMsg(target_pose);
        state.properties().set("target_pose", target_pose_msg);
        props.exposeTo(state.properties(), { "pregrasp", "grasp" });

        mtc::SubTrajectory trajectory;
        trajectory.setCost(std::abs(offset));
        trajectory.setComment(std::to_string(angle));
        rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

        spawn(std::move(state), std::move(trajectory));
    }
}

} // namespace fbot_manipulator
