<img width="5848" height="719" alt="fbot_manipulator" src="https://github.com/user-attachments/assets/c73b64f3-05bf-47c7-bb67-3e38c7f4161a" />

ROS 2 packages for robot arm manipulation, providing both low-level motion primitives (via services) and high-level task planning (via MoveIt Task Constructor actions).

## Packages

| Package | Description |
|---------|-------------|
| `fbot_manipulator` | Motion primitives, MTC task classes, and ROS 2 nodes |
| `fbot_manipulator_msgs` | Service and action interface definitions |

## Architecture

```
fbot_manipulator
├── manipulator_interface_node      (services: gripper, joint, pose, named targets)
└── manipulation_task_server        (action: pick, place, pick-and-place via MTC)
```

**Motion Primitives** wrap MoveIt 2's MoveGroupInterface for direct arm/gripper control, exposed as ROS 2 services. Robot-specific implementations inherit from `MotionPrimitivesBase`.

**MTC Tasks** use MoveIt Task Constructor to build multi-stage manipulation pipelines (approach, grasp, lift, move, place, retreat). These are exposed through a single ROS 2 action server that accepts a bounding box from the vision system and executes the requested task.

## Interfaces

### Services (manipulator_interface_node)

| Service | Type | Description |
|---------|------|-------------|
| `fbot_manipulator/set_gripper_position` | `MoveGripper` | Set gripper position (0.0 - 0.9) |
| `fbot_manipulator/move_to_named_target` | `MoveToNamedTarget` | Move to a predefined pose from config |
| `fbot_manipulator/move_joint` | `MoveJoint` | Move to joint angle targets (radians) |
| `fbot_manipulator/move_to_pose` | `MoveToPose` | Move to a Cartesian pose |

### Action (manipulation_task_server)

| Action | Type | Description |
|--------|------|-------------|
| `fbot_manipulator/manipulation_task` | `ManipulationTask` | Execute a pick, place, or pick-and-place task |

**ManipulationTask goal fields:**

| Field | Type | Description |
|-------|------|-------------|
| `task_type` | `uint8` | `PICK=0`, `PLACE=1`, `PICK_AND_PLACE=2` |
| `object_id` | `string` | Identifier for the object |
| `object_pose` | `geometry_msgs/Pose` | Bounding box center position |
| `object_size` | `geometry_msgs/Vector3` | Bounding box dimensions (x, y, z) |
| `place_pose` | `geometry_msgs/Pose` | Target placement pose (for PLACE and PICK_AND_PLACE) |

**Feedback:** `current_stage` (string) + `progress` (0.0 to 1.0)

**Result:** `success` (bool) + `message` (string)

## Directory Structure

```
fbot_manipulator/
├── include/fbot_manipulator/
│   ├── motion_primitives_base.hpp      # Abstract base for arm control
│   ├── motion_primitives_xarm.hpp      # xArm6 implementation
│   ├── utils.hpp                       # YAML config utilities
│   └── mtc/
│       ├── mtc_task.hpp                # MTC base task (solvers, collision objects, plan/execute)
│       ├── mtc_pick_task.hpp           # Pick pipeline
│       ├── mtc_place_task.hpp          # Place pipeline
│       └── mtc_pick_and_place_task.hpp # Combined pick-and-place pipeline
├── src/
│   ├── manipulator_interface_node.cpp  # Service server node
│   ├── manipulation_task_server.cpp    # Action server node
│   ├── utils.cpp
│   ├── motion_primitives/
│   │   ├── motion_primitives_base.cpp
│   │   └── motion_primitives_xarm.cpp
│   └── mtc/
│       ├── mtc_task.cpp
│       ├── mtc_pick_task.cpp
│       ├── mtc_place_task.cpp
│       └── mtc_pick_and_place_task.cpp
├── config/
│   └── xarm6/
│       ├── manipulator_config.yaml  # Motion primitives config
│       └── mtc_config.yaml          # MTC task config
└── launch/
    └── manipulator_interface.launch.py

fbot_manipulator_msgs/
├── srv/
│   ├── MoveGripper.srv
│   ├── MoveJoint.srv
│   ├── MoveToPose.srv
│   └── MoveToNamedTarget.srv
└── action/
    └── ManipulationTask.action
```

## MTC Task Pipelines

### Pick

```
CurrentState → OpenGripper → Connect(move to pick) →
  [ Approach → GenerateGraspPose + IK → AllowCollision(hand,object) →
    CloseGripper → AttachObject → AllowCollision(object,surface) →
    Lift → ForbidCollision(object,surface) ]
```

### Place

```
CurrentState → Connect(move to place) →
  [ Lower → GeneratePlacePose + IK → OpenGripper →
    ForbidCollision(hand,object) → DetachObject → Retreat ] →
  ReturnHome
```

### Pick and Place

Combines both pipelines into a single MTC task with shared stage references.

## Configuration

Robot-specific config lives in `config/<arm_type>/`:

### `manipulator_config.yaml` (Motion Primitives)

```yaml
# Controller names
moveit_controllers:
  traj: xarm_traj_controller
  gripper_traj: xarm_gripper

# Joint names
joints:
  arm_joints: [joint1, joint2, joint3, joint4, joint5, joint6]
  gripper_joints: [drive_joint]

# Named poses (joint angles in radians)
poses:
  home: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### `mtc_config.yaml` (MTC Tasks)

```yaml
manipulation_task_server:
  ros__parameters:
    mtc:
      arm_group_name: "xarm6"
      hand_group_name: "xarm_gripper"
      hand_frame: "link_tcp"
      world_frame: "world"
      surface_link: "world"
      approach_min: 0.05       # meters
      approach_max: 0.15
      lift_min: 0.08
      lift_max: 0.15
      retreat_min: 0.08
      retreat_max: 0.15
      max_solutions: 5
      grasp_angle_delta: 0.785 # radians (pi/4 = 8 grasp angles)
```

## Usage

### Build

```bash
colcon build --packages-select fbot_manipulator_msgs fbot_manipulator
```

### Launch

```bash
# --- xArm6 (simulation) ---
# 1. Launch MoveIt with MTC support
ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py add_gripper:=true add_mtc:=true

# 2. Launch fbot_manipulator nodes (in a new terminal)
ros2 launch fbot_manipulator manipulator_interface.launch.py arm_type:=xarm6
```

```bash
# --- WidowX 200 / wx200 (real hardware) ---
# 1. Launch the Interbotix MoveIt bringup (starts move_group + xs_sdk hardware interface)
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx200 hardware_type:=actual

# 2. Launch fbot_manipulator nodes (in a new terminal)
ros2 launch fbot_manipulator manipulator_interface_wx200.launch.py
```

The wx200 launch file references YAML files inside the `interbotix_xsarm_moveit`
package share. If your installed Interbotix release uses a different layout
(e.g., a `wx200/` subdirectory, or a different `controllers.yaml` filename),
update the paths in `launch/manipulator_interface_wx200.launch.py` to match.

### Send a pick-and-place goal

```bash
ros2 action send_goal /fbot_manipulator/manipulation_task \
  fbot_manipulator_msgs/action/ManipulationTask \
  "{task_type: 2, object_id: 'cup', \
    object_pose: {position: {x: 0.5, y: 0.0, z: 0.1}, orientation: {w: 1.0}}, \
    object_size: {x: 0.06, y: 0.06, z: 0.12}, \
    place_pose: {position: {x: 0.35, y: 0.2, z: 0.06}, orientation: {w: 1.0}}}" \
  --feedback
```

### Call a service

```bash
# Move to home pose
ros2 service call /fbot_manipulator/move_to_named_target \
  fbot_manipulator_msgs/srv/MoveToNamedTarget "{target_name: 'home'}"

# Open gripper
ros2 service call /fbot_manipulator/set_gripper_position \
  fbot_manipulator_msgs/srv/MoveGripper "{position: 0.0}"
```

## Adding a New Robot

1. Create `config/<robot_name>/manipulator_config.yaml` with the robot's joint names, controllers, and named poses
2. Create `config/<robot_name>/mtc_config.yaml` with the MoveIt group names, hand frame, and MTC offsets
3. Implement a new `MotionPrimitives<Robot>` class inheriting from `MotionPrimitivesBase` (header in `include/fbot_manipulator/`, source in `src/motion_primitives/`)
4. Add the new `.cpp` to the `manipulator_interface_node` target in `CMakeLists.txt`
5. Add the arm type case to `manipulator_interface_node.cpp`
6. Either add the robot to the existing `manipulator_interface.launch.py` or create a per-arm launch file (see `manipulator_interface_wx200.launch.py` for an example pointing at a non-xArm MoveIt config package)

## Contributing

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request
