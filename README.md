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



# HOW TO TEST THIS FROM ZERO

This guide explains how to set up the hardware and software for the cooperative control scenario with **3 machines** (or 2 if one acts as both Robot Host and Controller).

## 1. System Architecture

*   **Machine A (Robot Host)**: Physically connected to the Kinova Robot via USB. Runs the ROS Master (`roscore`) and the robot driver.
*   **Machine B (Joystick)**: "Luis's Laptop". Connected to the Logitech F310. Runs the Joystick node. **High Priority**.
*   **Machine C (GUI)**: Runs the Python GUI. **Low Priority**.

> **Note**: All machines must be on the **same local network** (Wi-Fi or Ethernet).

## 2. Requirements

### Machine A (Robot Host)
*   **OS**: Ubuntu (Native recommended, as per project notes).
*   **Software**: ROS (Melodic/Noetic), `kinova-ros` packages, `proyecto_pose` package.
*   **Hardware**: Kinova Robot connected via USB.

### Machine B (Joystick Controller)
*   **OS**: Ubuntu or Windows (with WSL2 + ROS).
*   **Software**: ROS, `joy` package (`sudo apt install ros-<distro>-joy`), `proyecto_pose` package.
*   **Hardware**: Logitech F310 Gamepad connected via USB.

### Machine C (GUI Controller)
*   **OS**: Ubuntu or Windows (with WSL2 + ROS).
*   **Software**: ROS, `tkinter` (usually included with Python), `proyecto_pose` package.

## 3. Network Configuration (Crucial!)

You must configure ROS to communicate across machines.

1.  **Find IP Addresses**: Run `ifconfig` (Linux) or `ipconfig` (Windows) on each machine to get their IP (e.g., `192.168.1.X`).
2.  **Edit `/etc/hosts` (Optional but recommended)**: Add the hostname and IP of the other machines to `/etc/hosts` on all computers for easier resolution.

### Environment Variables
Run these commands in **every terminal** you open (or add to `~/.bashrc`):

**On Machine A (Robot Host - IP: 192.168.1.100)**:
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100
```

**On Machine B (Joystick - IP: 192.168.1.101)**:
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311  <-- Points to Machine A
export ROS_IP=192.168.1.101                       <-- Its own IP
```

**On Machine C (GUI - IP: 192.168.1.102)**:
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311  <-- Points to Machine A
export ROS_IP=192.168.1.102                       <-- Its own IP
```

## 4. Step-by-Step Testing

### Step 1: Start the Robot (Machine A)
1.  Open Terminal 1:
    ```bash
    roscore
    ```
2.  Open Terminal 2:
    ```bash
    roslaunch kinova_bringup kinova_robot.launch
    ```
3.  Open Terminal 3 (Pose Client):
    ```bash
    rosrun proyecto_pose pose_client
    ```

### Step 2: Start the Joystick (Machine B)
1.  Connect Logitech F310. Make sure the switch on the back is set to **X** (XInput).
2.  Open Terminal:
    ```bash
    # Configure network exports first!
    rosrun joy joy_node
    ```
3.  Open Another Terminal:
    ```bash
    # Configure network exports first!
    # Start commander with Source ID 1 (Luis/Priority)
    rosrun proyecto_pose joy_commander _source_id:=1
    ```

### Step 3: Start the GUI (Machine C)
1.  Open Terminal:
    ```bash
    # Configure network exports first!
    rosrun proyecto_pose gui_commander.py
    ```

### Step 4: Verify Cooperative Control
1.  **Test GUI**: On Machine C, enter a small velocity (e.g., 0.1 m/s) and click "Vel Linear". The robot should move.
2.  **Test Priority**: While the robot is moving (or while sending commands from GUI), hold **LB** on the Joystick (Machine B) and move the sticks.
    *   **Expected Result**: The robot should immediately stop listening to the GUI and obey the Joystick, because Machine B has `source_id:=1` (High Priority).
3.  **Test Release**: Release the Joystick buttons. After 2 seconds (timeout), the GUI should be able to control the robot again.

