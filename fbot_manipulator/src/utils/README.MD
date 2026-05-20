<img width="5848" height="719" alt="fbot_manipulator" src="https://github.com/user-attachments/assets/c73b64f3-05bf-47c7-bb67-3e38c7f4161a" />

A Script to save the xarm6 pose using the zero-gravity mode and saving it to the SRFD.<br>
For later use it with the moveit

### Useful Services (xarm)

| Service              | Type                         | Description                                              |
| -------------------- | ---------------------------- | -------------------------------------------------------- |
| `xarm/motion_enable` | `xarm_msgs/srv/SetInt16ById` | Set joint and if it is enable or disable (id: 7 data: 0) |
| `xarm/set_mode`      | `xarm_msgs/srv/SetInt16ById` | set arm mode (data: 0...7)                               |
| `xarm/set_state`     | `xarm_msgs/srv/SetInt16ById` | set arm state (data: 0...4)                              |

## State and Modes

| Mode | Description                   |
| ---- | ----------------------------- |
| 0    | Position control mode         |
| 2    | Teaching mode (zero-gravity)  |
| 3    | Cartesian control mode        |
| 4    | Individual Joint control mode |

| State | Description |
| ----- | ----------- |
| 0     | Normal      |
| 1     | Paused      |
| 2     | Stopped     |

## Usage

### Save arm poses with `save_arm_pose.py`

1. Source your ROS 2 workspace and dependencies.
2. Run the script directly from this folder.
3. The robot is switched to teaching mode.
4. Move the arm by hand to the pose you want.
5. Type the pose name when prompted.
6. Choose whether to save another pose.

```bash
# from your workspace root
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

python3 /fbot_ws/src/fbot_manipulator/fbot_manipulator/src/utils/save_arm_pose.py
```

Prompt flow:

- Robot enters teaching mode (`mode=2`).
- Prompt: `Move the arm to the desired pose and enter its name...`
- The pose is read from `/joint_states` and saved into `<group_state ...>` in the xacro.
- Prompt: `Do you want to add more poses? (y/n)`
- If `n`, robot returns to position mode (`mode=0`) and exits.

Pose name rules:

- Allowed characters: letters, numbers, `_`, `-`, `.`
- Existing pose names are updated in-place.

### Call a service

```bash
# Lock Joints
ros2 service call /xarm/motion_enable \
  xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"

# Set mode to teaching (zero-gravity)
ros2 service call /xarm/set_state \
    xarm_msgs/srv/SetInt16ById "{data: 2}"

ros2 service call /xarm/set_mode \
    xarm_msgs/srv/SetInt16ById "{data: 0}"
```