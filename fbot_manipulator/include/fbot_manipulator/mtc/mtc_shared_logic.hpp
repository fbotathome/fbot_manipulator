#ifndef FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP
#define FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Includes do MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers.h>


#include "fbot_manipulator/mtc/mtc_task.hpp" 

namespace fbot_manipulator
{

namespace mtc = moveit::task_constructor;
 

class MtcSharedLogic
{
public:
    static void setupWorkspace(MtcTask* task_instance, 
                                const std::vector<ObjectDetection>& objects_scene,
                                geometry_msgs::msg::Vector3& pick_offset,
                                const std::string& target_id);

    static void setupWorkspace(MtcTask* task_instance,
                                const std::vector<ObjectDetection>& objects_scene,
                                geometry_msgs::msg::Vector3& pick_offset,
                                const std::vector<std::string>& target_ids);
     
    /**
     * @brief Constrói todos os estágios de Pick e injeta na 'task'.
     * @param approach_from_front true: aproxima pela frente (eixo X da garra);
     *                            false: aproxima por cima (top-down).
     * @return O ponteiro para o estágio 'attach_object' (necessário para o Place depois).
     */
    static mtc::Stage* addPickStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& object_pose,
        mtc::Stage* current_state,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        bool approach_from_front = false);

    /**
     * @brief Constrói todos os estágios de Place e injeta na 'task'.
     * @param approach_from_front true: aproxima/recua pela frente (eixo X da garra);
     *                            false: aproxima por cima (top-down).
     */
    static mtc::Stage* addPlaceStages(
        mtc::Task& task,
        const std::string& object_id,
        const geometry_msgs::msg::Pose& place_pose,
        mtc::Stage* attach_stage,
        const MtcConfig& config,
        std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner,
        std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner,
        std::shared_ptr<mtc::solvers::JointInterpolationPlanner> joint_planner,
        rclcpp::Logger logger,
        bool approach_from_front = false);
};

} // namespace fbot_manipulator

#endif // FBOT_MANIPULATOR_MTC_SHARED_LOGIC_HPP