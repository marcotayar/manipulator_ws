/*
  ESP32 — micro-ROS servo controller
  ====================================

  Subscribes to /arm_command and drives 5 servos.
  Joystick control is handled on the PC via joy_to_arm ROS node.

  /arm_command payload (Float32MultiArray, 5 elements):
    [0]  base_velocity   -1.0 .. +1.0   (continuous-rotation servo)
    [1]  shoulder_angle  rad
    [2]  elbow_angle     rad
    [3]  wrist_angle     rad
    [4]  gripper         -1.0 open .. 0.0 stop .. +1.0 close

  PC side (run all three):
    docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
    ros2 run joy joy_node
    ros2 run manipulator_control joy_to_arm
      -- or for RViz click-to-target --
    ros2 launch manipulator_control hardware.launch.py

  Wiring:
    Base     (MG996, 360°) → GPIO 13
    Shoulder (MG996)       → GPIO 12
    Elbow    (MG996)       → GPIO 14
    Wrist    (SG90)        → GPIO 27
    Gripper  (MG90S)       → GPIO 26

  Libraries needed:
    ESP32Servo, micro_ros_arduino (Humble branch)
*/

#include <ESP32Servo.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ── WiFi / agent ──────────────────────────────────────────
#define WIFI_SSID   "S25 Ultra de Marco"
#define WIFI_PASS   "8852966258"
#define AGENT_IP    "10.137.171.113"
#define AGENT_PORT  8888

// ── Servo pins ────────────────────────────────────────────
const int PIN_BASE  = 13;
const int PIN_SHLDR = 12;
const int PIN_ELBOW = 14;
const int PIN_WRIST = 27;
const int PIN_GRIP  = 26;

Servo servo_base, servo_shldr, servo_elbow, servo_wrist, servo_grip;

// ── Pulse widths (µs) ─────────────────────────────────────
const int PULSE_MIN = 1000;   // velocity servos (base/gripper): range around 1500
const int PULSE_MAX = 2000;
const int PULSE_MID = 1500;
const int BASE_SPEED_RANGE   = 200;  // tune if base is too fast/slow
const int GRIPPER_SPEED_RANGE = 200;  // tune if gripper is too fast/slow

// Positional servos (shoulder/elbow/wrist) use write(degrees): the attach
// range must match the servo's FULL travel or 0-180° comes out squashed.
// 500-2500 µs is the typical full range; write(0)->500µs->true 0°.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// ── Per-joint calibration ─────────────────────────────────
// Each positional servo has its own mounting, so each gets its own map:
//   servo_deg = HOME_DEG + DIR * degrees(ik_angle)
//   HOME_DEG : servo angle when that joint is at IK zero (link straight)
//   DIR      : +1 or -1 — flip if the joint moves the WRONG way
const float SH_HOME = 0.0f,   SH_DIR = +1.0f;   // shoulder: horizontal = 0°
const float EL_HOME = 90.0f,  EL_DIR = -1.0f;   // elbow:    straight   = 90°
const float WR_HOME = 90.0f,  WR_DIR = +1.0f;   // wrist:    straight   = 90°

// ── micro-ROS objects ─────────────────────────────────────
rcl_subscription_t  sub_arm_cmd;
std_msgs__msg__Float32MultiArray arm_cmd_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// ── Timing ────────────────────────────────────────────────
unsigned long last_cmd_ms  = 0;
unsigned long last_ping_ms = 0;
const unsigned long CMD_TIMEOUT_MS   = 1000;
const unsigned long PING_INTERVAL_MS = 5000;

// ── Servo helpers ─────────────────────────────────────────
// SAFETY: positional servos are clamped well inside their travel so the gears
// never slam into the mechanical end stops. Never widen past ~5/175.
const float SERVO_SAFE_MIN = 10.0f;
const float SERVO_SAFE_MAX = 170.0f;

int jointToDeg(float rad, float home, float dir) {
  float deg = home + dir * rad * 57.2958f;
  return (int)constrain(deg, SERVO_SAFE_MIN, SERVO_SAFE_MAX);
}

int baseVelToPulse(float v) {
  return PULSE_MID + (int)(constrain(v, -1.0f, 1.0f) * BASE_SPEED_RANGE);
}

int gripperToPulse(float v) {
  // v = -1.0 open, 0.0 stop, +1.0 close.
  // Flip sign of v in arm_cmd_callback if direction is wrong.
  return PULSE_MID + (int)(constrain(v, -1.0f, 1.0f) * GRIPPER_SPEED_RANGE);
}

// ── /arm_command callback ─────────────────────────────────
void arm_cmd_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * m =
      (const std_msgs__msg__Float32MultiArray *) msgin;
  if (m->data.size < 5) return;

  servo_base.writeMicroseconds(baseVelToPulse(m->data.data[0]));
  servo_shldr.write(jointToDeg(m->data.data[1], SH_HOME, SH_DIR));
  servo_elbow.write(jointToDeg(m->data.data[2], EL_HOME, EL_DIR));
  servo_wrist.write(jointToDeg(m->data.data[3], WR_HOME, WR_DIR));
  servo_grip.writeMicroseconds(gripperToPulse(m->data.data[4]));

  last_cmd_ms = millis();
}

// ── micro-ROS init ────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }

void error_loop() {
  while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

void microros_init() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_arm", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_arm_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_command"));

  arm_cmd_msg.data.capacity = 8;
  arm_cmd_msg.data.data = (float*) malloc(8 * sizeof(float));
  arm_cmd_msg.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm_cmd, &arm_cmd_msg,
      &arm_cmd_callback, ON_NEW_DATA));

  last_cmd_ms  = millis();
  last_ping_ms = millis();
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_base.setPeriodHertz(50);  servo_base.attach(PIN_BASE,   PULSE_MIN,    PULSE_MAX);
  servo_shldr.setPeriodHertz(50); servo_shldr.attach(PIN_SHLDR, SERVO_MIN_US, SERVO_MAX_US);
  servo_elbow.setPeriodHertz(50); servo_elbow.attach(PIN_ELBOW, SERVO_MIN_US, SERVO_MAX_US);
  servo_wrist.setPeriodHertz(50); servo_wrist.attach(PIN_WRIST, SERVO_MIN_US, SERVO_MAX_US);
  servo_grip.setPeriodHertz(50);  servo_grip.attach(PIN_GRIP,   PULSE_MIN,    PULSE_MAX);

  // Safe start pose — gripper hovers ABOVE ground, low shoulder load.
  // shoulder=+90° (tucked up), elbow=-60°, wrist=-80°  -> tip ~11 cm above ground.
  servo_base.writeMicroseconds(PULSE_MID);                       // stop
  servo_shldr.write(jointToDeg( 1.5708f, SH_HOME, SH_DIR));      // shoulder up
  servo_elbow.write(jointToDeg(-1.0472f, EL_HOME, EL_DIR));      // elbow folded
  servo_wrist.write(jointToDeg(-1.3963f, WR_HOME, WR_DIR));      // gripper down-ish
  servo_grip.writeMicroseconds(PULSE_MID);                       // stop

  set_microros_wifi_transports(
      (char*) WIFI_SSID, (char*) WIFI_PASS,
      (char*) AGENT_IP,  AGENT_PORT);
  delay(2000);

  microros_init();
  Serial.println("Ready.");
}

// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Stop base if /arm_command goes silent
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    servo_base.writeMicroseconds(PULSE_MID);
  }

  // Reboot if agent becomes unreachable
  if (now - last_ping_ms > PING_INTERVAL_MS) {
    last_ping_ms = now;
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      esp_restart();
    }
  }

  delay(10);
}
