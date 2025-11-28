### To test this thing just:

GUI Test:
1. Run gui_commander.py
2. Click buttons and verify messages on /comandos using rostopic echo /comandos
   
Joystick Test:
1. Connect a joystick (if available, otherwise mock /joy topic)
2. Run joy_commander
3. Publish to /joy manually or use a real joystick
4. Verify messages on /comandos.


## Overview
This document describes how to control the Kinova robot using the GUI and the Logitech F310 Joystick.

The system implements a priority logic to prevent conflicting commands:

1.  **Luis's Laptop (ID 1)**: Highest priority. Can interrupt any other controller.
2.  **Joystick (ID 2)**: Medium priority. Can take control if no one else is controlling.
3.  **GUI (ID 3)**: Low priority.

Joystick Control (Logitech F310)
Safety: You must hold the Deadman Switch (LB) at all times to move the robot.

| Button / Axis | Function | Description |
| :--- | :--- | :--- |
| **LB (Button 4)** | **Deadman Switch** | **MUST BE HELD TO MOVE** |
| **Left Stick** | Linear Velocity (X/Y) | Move robot in X and Y plane |
| **Buttons Y / A** | Linear Velocity (Z) | Y = Up, A = Down |
| **Right Stick** | Angular Velocity (Yaw/Pitch) | Rotate robot (Yaw/Pitch) |
| **Buttons B / X** | Angular Velocity (Roll) | B = Roll Right, X = Roll Left |

## GUI Control
Run the GUI with:
```bash
rosrun proyecto_pose gui_commander.py
```
- Enter values for Position or Velocity.
- Click the corresponding button to send the command.
- **STOP**: Sends a zero-velocity command to halt the robot.

## Launching
1.  Start ROS Core: `roscore`
2.  Start Driver: `roslaunch kinova_bringup kinova_robot.launch ...`
3.  Start Pose Client: `rosrun proyecto_pose pose_client`
4.  Start Joystick Node: `rosrun joy joy_node`
5.  Start Joy Commander: `rosrun proyecto_pose joy_commander`
