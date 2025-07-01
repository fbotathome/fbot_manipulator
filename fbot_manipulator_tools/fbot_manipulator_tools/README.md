# fbot_manipulator_tools

## Overview

This package contains a node for saving WX200 arm poses. The package allows users to capture and store robotic arm joint configurations for later use in manipulation applications.

## Package Structure

```
fbot_manipulator_tools/
├── package.xml
├── setup.cfg
├── setup.py
├── fbot_manipulator_tools/
│   ├── __init__.py
│   └── save_wx200_arm_pose.py
└── resource/
    └── fbot_manipulator_tools
```

## Features

### `save_wx200_arm_pose.py`

This module implements the `ArmJointStateSaver` node that allows saving WX200 arm poses to a YAML file for later use.

#### Key Features:

- **Safe Torque Disabling**: The node automatically disables arm torque to allow manual manipulation
- **Joint State Capture**: Captures current positions of all arm joints (excluding gripper)
- **YAML Saving**: Stores named poses in structured YAML format
- **Interactive Interface**: Intuitive command-line interface for naming and saving poses

## How to Use

### Prerequisites

1. Ensure the WX200 arm is connected and functioning
2. Launch the arm control package:
   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx200
   ```

### Running the Pose Saving Node

1. Execute the saving node:
   ```bash
   ros2 run fbot_manipulator_tools manipulator_saver
   ```

2. Follow the on-screen instructions:
   - Confirm that the arm is in sleep position before disabling torque
   - Specify the YAML file name (e.g., `arm_poses.yaml`)
   - Manually move the arm to the desired pose
   - Enter a name for the pose (e.g., 'PrePickup', 'LookToGarbage')
   - Choose whether to add more poses

3. The file will be saved in the workspace config directory

### Usage Example

```bash
$ ros2 run fbot_manipulator_tools manipulator_saver

The torque will be disabled. The Arm is in the sleep pose? (y/n): y
[WARN] [wx200ArmPoseSaver]: Disabling the torque
Enter the name of the file to save the joint states (e.g., arm_poses.yaml): my_poses.yaml
Move the arm to the desired pose and enter its name (e.g., 'PrePickup', 'LookToGarbage'): PrePickup
[INFO] [wx200ArmPoseSaver]: Received msg: [('waist', 0.0), ('shoulder', -1.57), ('elbow', 1.57), ('wrist_angle', 0.0), ('wrist_rotate', 0.0)]
[INFO] [wx200ArmPoseSaver]: Pose 'PrePickup' saved.
Do you want to add more poses? (y/n): n
[WARN] [wx200ArmPoseSaver]: Enabling the torque
[INFO] [wx200ArmPoseSaver]: Poses saved to my_poses.yaml. Shutting down node.
```

### YAML File Format

The generated file will have the following structure:

```yaml
poses:
  PrePickup:
    waist: 0.0
    shoulder: -1.5707963267948966
    elbow: 1.5707963267948966
    wrist_angle: 0.0
    wrist_rotate: 0.0
  LookToGarbage:
    waist: 1.2
    shoulder: -0.8
    elbow: 1.2
    wrist_angle: 0.5
    wrist_rotate: 0.0
```

## Safety

⚠️ **Important Warning**: 
- **Always ensure the arm is in a safe position (sleep) before disabling torque**
- Torque will be automatically re-enabled after saving all poses - the arm will not return to sleep position