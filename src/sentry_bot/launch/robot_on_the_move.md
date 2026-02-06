Here are the finalized launch commands in Markdown format. I have consolidated the steps into a logical order so you can easily copy and paste them into a `README.md` file or a cheat sheet.

---

# 🚀 Chitti Robot Simulation Guide

### 1. Preparation & Memory Cleanup

Run this command **every time** before starting the simulation to prevent Jetson memory errors (SHM locks).

```bash
# Kill all potential ghost processes and clear shared memory
./clean_robot.sh*

```

---

### 2. Build and Source

Ensure your workspace is up to date with the latest URDF and controller fixes.

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select sentry_bot
source install/setup.bash

```

---

### 3. Execution (Sequential Terminals)

#### **Terminal 1: Main Launch**

Starts Robot State Publisher, Gazebo (Headless), the ROS-GZ Bridge, and the Spawner.

```bash
source ~/robot_ws/install/setup.bash
ros2 launch sentry_bot rsp_sim.launch.py

```

#### **Terminal 2: Unpause Simulation**

Gazebo starts paused. You **must** run this for the clock and sensors to start.

```bash
ign service -s /world/shapes/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req "pause: false"

```

#### **Terminal 3: Teleop (Movement)**

The `u` key is for diagonal; use `i` for straight forward.

```bash
source ~/robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

```

#### **Terminal 4: RViz Visualization**

Launch RViz with your saved configuration (or open a fresh one).

```bash
# If you have a saved config:
rviz2 -d ~/robot_ws/src/sentry_bot/config/chitti_view.rviz

# Or just a fresh instance:
rviz2

```

---

### 👁️ RViz Setup Checklist

Once RViz is open, ensure these settings are applied to see the data:

| Feature | Setting / Topic | Note |
| --- | --- | --- |
| **Fixed Frame** | `odom` | **Crucial:** Manually type if not in list |
| **Robot Model** | Add -> RobotModel | Set Description Topic to `/robot_description` |
| **Lidar (Laser)** | Add -> LaserScan | Topic: `/scan` |
| **Camera** | Add -> Image | Topic: `/camera/image_raw` |

---

### 🛠️ Troubleshooting Commands

**Check if the "Nervous System" is active:**

```bash
# See if all 8 joints (6 wheels + 2 camera) are publishing
ros2 topic echo /joint_states

```

**Check if the "Heartbeat" is active:**

```bash
# If numbers are changing, the sim is unpaused
ros2 topic echo /clock

```

**Check if the "Motors" are active:**

```bash
# List all active hardware controllers
ros2 control list_controllers

```

Quick Checklist for your first "One-Click" run:

Terminal 1: Run chittistart (or click the icon).

Terminal 2: Run the Unpause service command.

Terminal 3: Run Teleop to drive.

Terminal 4: Run RViz (it will now remember your settings if you saved the .rviz file).
---
