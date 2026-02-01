### 1. Preparation (SSH via Mac Terminal)

Keep your standard Mac terminal open to manage the background services. You do not need to reinstall NoMachine, but you should ensure the server is ready for a "headless" connection.

1. **Check NoMachine Status**:
```bash
/usr/NX/bin/nxserver --status

```


2. **Restart Services (If needed)**: If you previously had the "wallpaper-only" issue, running this restart command often forces the virtual display to initialize correctly:
```bash
sudo /usr/NX/bin/nxserver --restart

```



---

### 2. Connect via NoMachine App

1. Open the **NoMachine** application on your Mac.
2. Double-click your connection.
3. **Handle the Display Prompt**: If you see a message saying "Cannot detect any display running," click **Yes** to create a new virtual display.
4. **Login**: Enter your username and password.

### 3. Setup the Workspace (Inside NoMachine)

Once the Jetson desktop appears, open a terminal **inside the NoMachine window** to start your ROS 2 nodes. Running them here ensures that the 3D graphics (RViz) stay local to the Jetson's GPU.

**Terminal 1: Launch Robot State Publisher**

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/robot_ws/src/sentry_bot/description/robot.urdf.xacro)"

```

**Terminal 2: Launch Joint State Publisher**
Since you have a 6-wheel rover with "continuous" joints, you must run this to make the wheels appear in the correct positions:

```bash
ros2 run joint_state_publisher joint_state_publisher

```

**Terminal 3: Launch RViz2**

```bash
rviz2

```

---

### 4. RViz Setup Checklist

Each time you open RViz, verify these settings to ensure you can see your multi-colored robot:

* **Fixed Frame**: Manually type `base_link`.
* **Background Color**: Change from dark gray to **Light Gray** so the **black chassis** is visible.
* **RobotModel**:
* **Description Topic**: Ensure it is `/robot_description`.
* **Durability Policy**: Set to `Transient Local`.



**Does following this specific order allow you to see the grey wheels and colored sensors immediately without any "No transform" errors?**

---

### 5. Materials

Here are the reference materials:

* **Creating a rough 3D model of our robot with URDF**: https://www.youtube.com/watch?v=BcjHyhV0kIs.
* **Background Color**: Change from dark gray to **Light Gray** so the **black chassis** is visible.
* **RobotModel**:
* **Description Topic**: Ensure it is `/robot_description`.
* **Durability Policy**: Set to `Transient Local`.