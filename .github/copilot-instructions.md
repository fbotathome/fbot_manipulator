# Copilot Instructions for fbot_manipulator

## Project Overview

**fbot_manipulator** is a ROS 2 package for robot arm manipulation providing:
- **Motion Primitives**: Low-level arm/gripper control via MoveIt 2 (exposed as ROS 2 services)
- **MTC Tasks**: High-level multi-stage manipulation pipelines using MoveIt Task Constructor (exposed via action server)

The codebase supports multiple robot types through a configuration-driven architecture (currently implements xArm6).

## Build & Test Commands

```bash
# Build both message definitions and main package
colcon build --packages-select fbot_manipulator_msgs fbot_manipulator

# Full workspace build (if other ROS 2 dependencies present)
colcon build

# Run linters (ament_lint_auto, enforces -Wall -Wextra -Wpedantic)
colcon build --packages-select fbot_manipulator --cmake-args -DBUILD_TESTING=ON
```

## High-Level Architecture

### Two Main Components

1. **manipulator_interface_node** (`src/manipulator_interface_node.cpp`)
   - ROS 2 service server for direct arm/gripper control
   - Wraps motion primitives implementations
   - Services: `set_gripper_position`, `move_to_named_target`, `move_joint`, `move_to_pose`
   - Robot-specific motion primitives inherit from `MotionPrimitivesBase` in `include/motion_primitives_base.hpp`

2. **manipulation_task_server** (`src/manipulation_task_server.cpp`)
   - ROS 2 action server for multi-stage manipulation tasks
   - Action type: `ManipulationTask` (goal: `task_type`, `object_id`, `object_pose`, `object_size`, `place_pose`)
   - Supported tasks: PICK (0), PLACE (1), PICK_AND_PLACE (2), POUR (3)
   - Each task type has a corresponding `MtcXxxTask` class

### MTC Task Base Class Pattern

All MTC tasks inherit from `MtcTask` (`include/mtc/mtc_task.hpp`):

```cpp
class MtcTask {
  virtual bool buildTask() = 0;  // Subclasses implement stage pipeline
  bool plan();                   // Plan the task
  bool execute();                // Execute the planned task
  void addCollisionObject(...);  // Add objects to planning scene
  void removeCollisionObject(...);
};
```

**Task Pipeline Structure:**
1. Load robot model and configure solvers (PipelinePlanner, CartesianPath, JointInterpolationPlanner)
2. Set task properties (group, EEF, IK frame)
3. Build stages (CurrentState → Connect → Approach/Lower → IK → Actions → Retreat → Return Home)
4. Plan and execute

### Configuration

Robot-specific settings live in `config/<arm_type>/`:
- **manipulator_config.yaml**: Joint names, controller names, named poses (for motion primitives)
- **mtc_config.yaml**: MTC task parameters (frames, distances, IK solutions)

See `MtcConfig` struct in `mtc_task.hpp` for all available MTC parameters (arm_group_name, hand_group_name, hand_frame, world_frame, etc.).

## Key Conventions

### MTC Task Implementation

When adding a new MTC task (e.g., `MtcXxxTask`):

1. **Header** (`include/mtc/mtc_xxx_task.hpp`):
   - Inherit from `MtcTask`
   - Store task-specific parameters as member variables (e.g., `object_id_`, `object_pose_`)
   - Override `buildTask()` to define the pipeline

2. **Source** (`src/mtc/mtc_xxx_task.cpp`):
   - Constructor initializes member variables and calls `MtcTask(task_name, node)`
   - `buildTask()` creates stages using `std::make_unique<mtc::stages::StageName>()` and adds them with `task_.add(std::move(stage))`
   - Use `task_` (inherited from MtcTask) to access the MTC task
   - Use `config_` (inherited from MtcTask) for robot-specific parameters
   - Use `pipeline_planner_`, `cartesian_planner_`, `joint_planner_` (inherited solvers)

3. **Registration** (`src/manipulation_task_server.cpp`):
   - Add task type enum to switch statement
   - Instantiate the task class with the action goal parameters
   - Call `task->plan()` then `task->execute()`

### Namespace

All code is in `namespace fbot_manipulator`. Use `using namespace fbot_manipulator;` or fully qualified names.

### YAML Configuration Loading

- `MtcTask::loadConfig()` reads from ROS 2 parameters under the node's namespace
- Config structure mirrors `MtcConfig` struct members
- Example: `mtc.arm_group_name` parameter becomes `config_.arm_group_name`

### Stage Property Configuration

Stages propagate properties via:
- `stage->properties().configureInitFrom(mtc::Stage::PARENT)` - inherit from parent container
- `stage->properties().configureInitFrom(mtc::Stage::INTERFACE, {"field_names"})` - inherit specific interface fields
- `stage->properties().set("key", value)` - set direct properties (e.g., "marker_ns" for visualization)

### Naming Conventions

- MTC stage names should be descriptive and match their function: "approach object pose", "move to pre-pour approach", "recover wrist"
- Task name passed to `MtcTask` constructor is used in task container name: `task_.stages()->setName("task_" + object_id_)`
- Marker namespaces (`properties().set("marker_ns", ...)`) enable separate rviz visualization channels

## Adding a New Task Type

1. Create `include/mtc/mtc_xxx_task.hpp` and `src/mtc/mtc_xxx_task.cpp` following the pattern above
2. Add source file to `CMakeLists.txt` in `add_executable(manipulation_task_server ...)`
3. Register in `src/manipulation_task_server.cpp` (add enum, create task, call plan/execute)
4. Add task parameters to `config/xarm6/mtc_config.yaml` if needed
5. Update `fbot_manipulator_msgs/action/ManipulationTask.action` if adding new task type enum

## Adding a New Robot

1. Create `config/<robot_name>/manipulator_config.yaml` and `mtc_config.yaml`
2. Implement `MotionPrimitives<Robot>` class in `include/motion_primitives_<robot>.hpp` and `src/motion_primitives/motion_primitives_<robot>.cpp`
3. Register in `manipulator_interface_node.cpp` (add case to arm type switch)
4. Update `CMakeLists.txt` to compile new motion primitives file
5. Launch with `arm_type:=<robot_name>`
