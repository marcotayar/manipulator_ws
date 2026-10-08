/*
  ESP32 — shoulder servo potentiometer feedback
  ==============================================

  Reads the shoulder servo's INTERNAL potentiometer (tapped off the wiper
  wire inside the servo case) and publishes the resulting angle to ROS2 as
  std_msgs/Float32 on /shoulder_feedback.

  Angle convention matches arm_ik_2d.py's shoulder joint: radians measured
  from horizontal, same value esp32_microros.ino's jointToDeg() expects
  (SH_HOME=0, SH_DIR=+1 -> servo_deg == shoulder_rad in degrees).

  Only one servo for now (shoulder). Run this sketch standalone, separate
  from esp32_microros.ino, while testing the feedback loop.

  Wiring: see README section below / chat instructions.
    Potentiometer wiper -> voltage divider -> GPIO34 (ADC1_CH6)
    Potentiometer / servo GND -> ESP32 GND (shared ground, required)

  Calibration (REQUIRED before trusting the published angle):
    1. Upload with CALIBRATE_MODE 1, open Serial Monitor @115200.
    2. Move the shoulder by hand to its minimum safe angle (10 deg,
       matches SH_MIN in arm_ik_2d.py), note the printed raw ADC value.
    3. Move it to the maximum safe angle (90 deg, SH_MAX), note that value.
    4. Fill RAW_AT_MIN_DEG / RAW_AT_MAX_DEG below, set CALIBRATE_MODE 0,
       re-upload.

  Libraries needed: micro_ros_arduino (Humble branch)
*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ── WiFi / agent ──────────────────────────────────────────
#define WIFI_SSID   "S25 Ultra de Marco"
#define WIFI_PASS   "8852966258"
#define AGENT_IP    "10.137.171.113"
#define AGENT_PORT  8888

// ── Potentiometer ─────────────────────────────────────────
const int POT_PIN = 34;   // ADC1_CH6 — input-only pin, safe with WiFi on

// Set to 1 to print raw ADC values for calibration, skip ROS publishing.
#define CALIBRATE_MODE 0

// Raw ADC readings at two known shoulder angles (fill in after calibrating).
const int   RAW_AT_MIN_DEG = 600;    // raw value when shoulder == MIN_DEG
const int   RAW_AT_MAX_DEG = 3000;   // raw value when shoulder == MAX_DEG
const float MIN_DEG = 10.0f;         // must match SH_MIN in arm_ik_2d.py
const float MAX_DEG = 90.0f;         // must match SH_MAX in arm_ik_2d.py

const int   SAMPLES = 16;   // averaged per reading, reduces ADC noise

// ── micro-ROS objects ─────────────────────────────────────
rcl_publisher_t      pub_feedback;
std_msgs__msg__Float32 feedback_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     timer;

const unsigned int PUBLISH_PERIOD_MS = 50;   // 20 Hz

// ── Helpers ───────────────────────────────────────────────
int readPotRaw() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) sum += analogRead(POT_PIN);
  return (int)(sum / SAMPLES);
}

float rawToShoulderRad(int raw) {
  float deg = MIN_DEG + (float)(raw - RAW_AT_MIN_DEG) *
              (MAX_DEG - MIN_DEG) / (float)(RAW_AT_MAX_DEG - RAW_AT_MIN_DEG);
  deg = constrain(deg, MIN_DEG, MAX_DEG);
  return deg * 0.0174533f;   // deg -> rad
}

// ── Timer callback: read pot, publish angle ──────────────
void publish_timer_callback(rcl_timer_t * t, int64_t last_call_time) {
  (void) last_call_time;
  if (t == NULL) return;

  int raw = readPotRaw();
  feedback_msg.data = rawToShoulderRad(raw);
  rcl_publish(&pub_feedback, &feedback_msg, NULL);
}

// ── micro-ROS init ────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }

void error_loop() {
  while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

void microros_init() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "shoulder_pot_feedback", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_feedback, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/shoulder_feedback"));

  RCCHECK(rclc_timer_init_default(&timer, &support,
      RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(12);          // 0-4095
  analogSetAttenuation(ADC_11db);    // full 0-3.3V range

#if CALIBRATE_MODE
  while (true) {
    Serial.println(readPotRaw());
    delay(200);
  }
#endif

  set_microros_wifi_transports(
      (char*) WIFI_SSID, (char*) WIFI_PASS,
      (char*) AGENT_IP,  AGENT_PORT);
  delay(2000);

  microros_init();
  Serial.println("Ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
