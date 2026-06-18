/*
  ESP32 — micro-ROS Servo Controller
  ===================================

  The ESP32 is a micro-ROS node that subscribes to /arm_command
  (std_msgs/Float32MultiArray, 5 elements) and drives the servos.

  Command array layout:
    [0] base_velocity   -1.0 .. +1.0   (continuous 360 servo: 0 = stop)
    [1] shoulder_angle  radians        (position servo)
    [2] elbow_angle     radians        (position servo)
    [3] wrist_angle     radians        (position servo)
    [4] gripper         0.0 = open, 1.0 = closed

  Transport: WiFi UDP to the micro-ROS agent running on the PC.

  ── Required libraries ────────────────────────────────────────
    micro_ros_arduino (branch matching your ROS 2 distro, e.g. 'humble'):
      https://github.com/micro-ROS/micro_ros_arduino
    ESP32Servo

  ── Run the agent on the PC ───────────────────────────────────
    docker run -it --rm --net=host microros/micro-ros-agent:humble \
        udp4 --port 8888

  Wiring:
    Base  (MG996, 360) → GPIO 13
    Shoulder (MG996)   → GPIO 12
    Elbow (MG996)      → GPIO 14
    Wrist (MG90S)      → GPIO 27
    Gripper (SG90)     → GPIO 26
*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <WiFi.h>
#include <ESP32Servo.h>

// ── WiFi / agent ──────────────────────────────────────────
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"
#define AGENT_IP      "192.168.0.100"   // PC running the micro-ROS agent
#define AGENT_PORT    8888

// ── micro-ROS objects ─────────────────────────────────────
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ── Servo pins ────────────────────────────────────────────
const int PIN_BASE  = 13;
const int PIN_SHLDR = 12;
const int PIN_ELBOW = 14;
const int PIN_WRIST = 27;
const int PIN_GRIP  = 26;

Servo servo_base;
Servo servo_shldr;
Servo servo_elbow;
Servo servo_wrist;
Servo servo_grip;

// ── Pulse widths (µs) ─────────────────────────────────────
// Standard range (1000–2000) safe for MG996R / MG90S / SG90.
// Do NOT use 500–2500 — it can physically damage the gears.
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;
const int PULSE_MID = 1500;

// Base continuous-rotation: 1500 = stop, ±200 µs for speed.
// Tune BASE_SPEED_RANGE if the base is too fast or too slow.
const int BASE_SPEED_RANGE = 200;

// ── Position-servo joint limits (rad) ─────────────────────
const float J_MIN = -1.5708f;   // -π/2
const float J_MAX =  1.5708f;   // +π/2

// ── Failsafe / reconnect ──────────────────────────────────
unsigned long last_cmd_ms    = 0;
unsigned long last_ping_ms   = 0;
const unsigned long CMD_TIMEOUT_MS  = 1000;   // stop base after 1 s silence
const unsigned long PING_INTERVAL_MS = 5000;  // check agent every 5 s

// ── Helpers ───────────────────────────────────────────────
int radToPulse(float rad) {
  if (rad < J_MIN) rad = J_MIN;
  if (rad > J_MAX) rad = J_MAX;
  float t = (rad - J_MIN) / (J_MAX - J_MIN);   // 0..1
  return (int)(PULSE_MIN + t * (PULSE_MAX - PULSE_MIN));
}

int baseVelToPulse(float v) {
  if (v < -1.0f) v = -1.0f;
  if (v >  1.0f) v =  1.0f;
  return PULSE_MID + (int)(v * BASE_SPEED_RANGE);
}

int gripperToPulse(float g) {
  if (g < 0.0f) g = 0.0f;
  if (g > 1.0f) g = 1.0f;
  // open = 1500 µs, closed = 1100 µs.
  // Flip the sign or change 400 if your gripper closes the wrong way.
  return PULSE_MID - (int)(g * 400);
}

// ── Subscription callback ─────────────────────────────────
void cmd_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * m =
      (const std_msgs__msg__Float32MultiArray *) msgin;

  if (m->data.size < 5) return;

  float base_v   = m->data.data[0];
  float shoulder = m->data.data[1];
  float elbow    = m->data.data[2];
  float wrist    = m->data.data[3];
  float gripper  = m->data.data[4];

  servo_base.writeMicroseconds(baseVelToPulse(base_v));
  servo_shldr.writeMicroseconds(radToPulse(shoulder));
  servo_elbow.writeMicroseconds(radToPulse(elbow));
  servo_wrist.writeMicroseconds(radToPulse(wrist));
  servo_grip.writeMicroseconds(gripperToPulse(gripper));

  last_cmd_ms = millis();
}

// ── Error loop (blinks LED) ───────────────────────────────
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(120);
  }
}

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFT(fn)  { rcl_ret_t rc = fn; (void) rc; }

// ── micro-ROS init (called from setup and on reconnect) ───
void microros_init() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_arm", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_command"));

  // Pre-allocate the message buffer (CRITICAL for micro-ROS)
  cmd_msg.data.capacity = 8;
  cmd_msg.data.data = (float*) malloc(8 * sizeof(float));
  cmd_msg.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &cmd_msg,
      &cmd_callback, ON_NEW_DATA));

  last_cmd_ms  = millis();
  last_ping_ms = millis();
}

void setup() {
  // ESP32Servo: allocate hardware timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_base.setPeriodHertz(50);
  servo_shldr.setPeriodHertz(50);
  servo_elbow.setPeriodHertz(50);
  servo_wrist.setPeriodHertz(50);
  servo_grip.setPeriodHertz(50);

  servo_base.attach(PIN_BASE,   PULSE_MIN, PULSE_MAX);
  servo_shldr.attach(PIN_SHLDR, PULSE_MIN, PULSE_MAX);
  servo_elbow.attach(PIN_ELBOW, PULSE_MIN, PULSE_MAX);
  servo_wrist.attach(PIN_WRIST, PULSE_MIN, PULSE_MAX);
  servo_grip.attach(PIN_GRIP,   PULSE_MIN, PULSE_MAX);

  // Safe startup state
  servo_base.writeMicroseconds(PULSE_MID);
  servo_shldr.writeMicroseconds(PULSE_MID);
  servo_elbow.writeMicroseconds(PULSE_MID);
  servo_wrist.writeMicroseconds(PULSE_MID);
  servo_grip.writeMicroseconds(PULSE_MID);

  pinMode(LED_BUILTIN, OUTPUT);

  set_microros_wifi_transports(
      (char*) WIFI_SSID,
      (char*) WIFI_PASS,
      (char*) AGENT_IP,
      AGENT_PORT);

  delay(2000);

  microros_init();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));

  unsigned long now = millis();

  // Failsafe: stop base servo if commands stop arriving
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    servo_base.writeMicroseconds(PULSE_MID);
  }

  // Reconnect watchdog: ping agent every PING_INTERVAL_MS.
  // If unreachable, reboot — setup() re-establishes the connection.
  if (now - last_ping_ms > PING_INTERVAL_MS) {
    last_ping_ms = now;
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      esp_restart();
    }
  }
}
