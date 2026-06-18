# manipulator_ws

ROS 2 Humble workspace for a 4-DOF RRRR desktop manipulator controlled via an ESP32 (micro-ROS).

## Robot overview

| Joint | Type | Axis | Range | Hardware |
|---|---|---|---|---|
| J1 — base yaw | Continuous | Z | velocity-only (no position limit) | 360° servo |
| J2 — shoulder | Revolute | Y | ±90° | Standard servo |
| J3 — elbow | Revolute | Y | ±90° | Standard servo |
| J4 — wrist | Revolute | Y | ±90° | MG90S servo |
| Gripper | Prismatic (×2) | Y | 0–13 mm | Servo |

**Link geometry** (matches `arm_ik_2d.py`):

```
Shoulder origin:  0.09 m above ground  (base 0.07 m + turntable 0.02 m)
L1  shoulder → elbow:  0.10 m
L2  elbow → wrist:     0.09 m
L3  wrist → EE tip:    0.16 m  (gripper_base 0.03 m + fingers 0.13 m)
```

## Packages

| Package | Type | Purpose |
|---|---|---|
| `manipulator_description` | ament_cmake | URDF/xacro, RViz config |
| `manipulator_control` | ament_python | Keyboard teleop, click-to-target, arm commander, 2D IK |
| `manipulator_kinematics` | ament_python | 3D IK node with TF-based trajectory visualization |

## Prerequisites

- **ROS 2 Humble**
- **setuptools 58.2.0** — required for colcon to build `ament_python` packages (newer versions dropped `setup.py develop --editable`):

```bash
pip3 install setuptools==58.2.0
```

- Standard ROS tools (already in a desktop install):

```bash
sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-xacro \
                 ros-humble-tf2-ros
```

## Build

```bash
cd ~/manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the source line to your shell if you don't want to run it each session:

```bash
echo "source ~/manipulator_ws/install/setup.bash" >> ~/.bashrc
```

## Usage modes

### 1. Visualize only (joint sliders)

Shows the robot in RViz with a GUI slider for every joint. Useful for checking the model.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manipulator_description display.launch.py
```

### 2. Keyboard teleop

Opens RViz and lets you drive each joint from the keyboard. Run the launch file in one terminal and the teleop node in a second terminal (it requires raw key input on its own TTY).

**Terminal 1:**
```bash
ros2 launch manipulator_control teleop.launch.py
```

**Terminal 2:**
```bash
ros2 run manipulator_control keyboard_teleop
```

Key bindings:

| Key | Action |
|---|---|
| `q` / `a` | base spin left / right (velocity, keeps spinning) |
| `space` | base stop |
| `w` / `s` | J2 shoulder +/− |
| `e` / `d` | J3 elbow +/− |
| `r` / `f` | J4 wrist +/− |
| `t` / `g` | gripper close / open |
| `z` | home (all joints to 0, base stops) |
| `x` | quit |

J1 is velocity-controlled: press `q` or `a` to start spinning, `space` to stop. J2–J4 step 0.05 rad (~3°) per keypress; gripper steps 2 mm.

### 3. Click-to-move (IK)

Click anywhere in the RViz viewport to send the arm to that position using the IK solver. The EE trajectory is drawn as an orange line.

```bash
ros2 launch manipulator_control click_move.launch.py
```

In RViz, select the **Publish Point** tool from the top toolbar, then click on the ground grid. The IK node smoothly interpolates the arm to the target over ~1.5 s and reports the final position error in the terminal.

### 4. Base + gripper teleop (separate node)

Velocity control for the continuous-rotation base servo and direct gripper control:

```bash
ros2 run manipulator_control base_gripper_teleop
```

| Key | Action |
|---|---|
| `j` | base rotate left |
| `l` | base rotate right |
| `k` | base stop |
| `o` | gripper open |
| `p` | gripper close |
| `x` | quit |

### 5. Real hardware

See [Hardware control](#hardware-control) below.

## ROS topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `keyboard_teleop` or `ik_node` | `robot_state_publisher` |
| `/target_pose` | `geometry_msgs/Point` | `click_to_target` | `ik_node`, `arm_commander` |
| `/base_cmd` | `std_msgs/Float32` | `base_vel_gui`, `base_gripper_teleop`, `keyboard_teleop` | `arm_commander` |
| `/gripper_cmd` | `std_msgs/Float32` | `base_gripper_teleop` | `arm_commander` |
| `/arm_command` | `std_msgs/Float32MultiArray` | `arm_commander` | ESP32 (micro-ROS) |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz | `click_to_target` |
| `/click_marker` | `visualization_msgs/Marker` | `click_to_target` | RViz |
| `/ee_trajectory` | `visualization_msgs/Marker` | `ik_node` | RViz |

### `/arm_command` payload

Five floats sent to the ESP32 at 50 Hz:

```
[0]  base_velocity   -1.0 .. +1.0
[1]  shoulder_angle  rad
[2]  elbow_angle     rad
[3]  wrist_angle     rad
[4]  gripper         0.0 open .. 1.0 closed
```

The base joint uses velocity (not position) because the hardware is a continuous-rotation servo with no angle feedback.

## IK solvers

Two independent IK implementations are included:

**`arm_ik_2d.py`** (used by `arm_commander`) — 2D geometric IK in the vertical arm plane. Tries to keep the EE pointing straight down first, then sweeps outward in 5° steps if unreachable. This is what drives the real hardware.

**`ik_node.py`** (used by `click_move.launch.py`) — 3D IK for the full arm (adds base yaw from the click X/Y). Interpolates joint motion over 50 steps, reads actual EE position from the TF tree for trajectory drawing.

## Hardware control

### One-time firmware setup

**1. Fill in your network details** at the top of `esp32_microros.ino`:

```cpp
#define WIFI_SSID   "your-network"
#define WIFI_PASS   "your-password"
#define AGENT_IP    "192.168.x.x"   // run: hostname -I | awk '{print $1}'
#define AGENT_PORT  8888
```

**2. Flash** via Arduino IDE (board: *ESP32 Dev Module*, upload speed: 921600).

**3. Verify servo zero positions** — at startup all servos receive 1500 µs (center pulse). The arm should sit with each link pointing **horizontal** (shoulder/elbow/wrist all at 0 rad in `arm_ik_2d` convention). If a link points up or down instead, rotate the servo horn one spline and reflash.

---

### Every session

**Terminal 1 — micro-ROS agent** (keep running):

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

Wait for the line `[1] [RTPS Participant matched]` — this confirms the ESP32 connected.

**Terminal 2 — hardware stack**:

```bash
source ~/manipulator_ws/install/setup.bash
ros2 launch manipulator_control hardware.launch.py
```

This starts: `robot_state_publisher`, `arm_commander`, `click_to_target`, `base_vel_gui`, and RViz.

---

### Operating procedure

**Step 1 — aim the base**

The base (J1) is a continuous-rotation servo — it has no position feedback, so IK cannot aim it automatically. Use the floating **Base Velocity** window that opens alongside RViz:

| Button | Effect |
|---|---|
| ◄ Left | Base spins left at 0.5 rad/s |
| Stop | Base stops |
| Right ► | Base spins right at 0.5 rad/s |

Rotate until the arm plane faces your target, then press **Stop**.

**Step 2 — click to position**

In RViz, select the **Publish Point** tool from the top toolbar, then click on the ground grid where you want the end-effector to go. `arm_commander` runs the 2D IK and sends the result to the ESP32 immediately.

**Reachable workspace** (gripper pointing down, at ground level):

```
0.115 m ≤ horizontal distance from base axis ≤ 0.160 m
```

Clicks outside this ring produce no motion — the IK returns no solution and the arm holds its last position.

**Step 3 — gripper**

Run in a separate terminal:

```bash
ros2 run manipulator_control base_gripper_teleop
```

Press `o` to open, `p` to close.

Or publish directly:

```bash
ros2 topic pub --once /gripper_cmd std_msgs/Float32 "{data: 1.0}"  # close
ros2 topic pub --once /gripper_cmd std_msgs/Float32 "{data: 0.0}"  # open
```

---

### Failsafe behaviour

| Condition | Response |
|---|---|
| No `/arm_command` received for > 1 s | Base servo stops; position servos hold last angle |
| micro-ROS agent unreachable for > 5 s | ESP32 reboots and reconnects automatically |

> **Note:** RViz shows the URDF at home pose — it does not mirror the real arm position. `arm_commander` does not publish `/joint_states`. Use RViz for click targeting only.

---

### Servo calibration

All tuning constants are at the top of `esp32_microros.ino`:

| Constant | Default | Adjust when… |
|---|---|---|
| `BASE_SPEED_RANGE` | 200 µs | Base spins too fast or too slow |
| `gripperToPulse` span (`400`) | 400 µs | Gripper doesn't fully open/close, or closes the wrong way (flip sign) |
| `PULSE_MIN` / `PULSE_MAX` | 1000 / 2000 µs | Servos don't reach full range (widen carefully — never below 600 / above 2400) |
