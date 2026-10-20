/*
  ESP32 — calibrated shoulder PID position control, TB6612FNG channel B
  ==============================================

  Reads the shoulder servo's INTERNAL potentiometer (tapped off the wiper
  wire inside the servo case) and publishes the resulting angle to ROS2 as
  std_msgs/Float32 on /shoulder_feedback.
  /shoulder_target_deg (Float32): target in degrees, refreshed within 1 second.
  /shoulder_pid_enable (Bool): explicit enable; starts disabled.
  /estop (Bool, transient-local): stops PID; release never automatically enables.
  /shoulder_pid_gains (Float32MultiArray): [kp, ki, kd], disabled/non-estop only.
  /shoulder_pid_telemetry (Float32MultiArray): 14 finite values at 20 Hz,
  including applied gains, signed quantized output duty, and latched faults.

  Angle convention matches arm_ik_2d.py's shoulder joint: radians measured
  from horizontal, same value esp32_microros.ino's jointToDeg() expects
  (SH_HOME=0, SH_DIR=+1 -> servo_deg == shoulder_rad in degrees).

  Run standalone, separate from esp32_microros.ino. Keep the arm supported:
  standby coasts and does not hold the shoulder against gravity.

  Wiring: see README section below / chat instructions.
    Potentiometer wiper -> voltage divider -> GPIO34 (ADC1_CH6)
    Potentiometer / motor supply GND -> ESP32 GND (shared ground)
    TB6612FNG: PWMB=32, BIN1=14, BIN2=13, STBY=25, AIN1=26, AIN2=27
    Motor -> BO1/BO2; VCC -> ESP32 3V3; VM -> rated motor supply

  Calibration (REQUIRED before trusting the published angle):
    1. Upload with CALIBRATE_MODE 1, open Serial Monitor @115200.
    2. Hold measured angles from 0 to 75 deg in 15 deg steps, recording raw
       medians. Repeat some points in the opposite direction.
    3. Update SHOULDER_CALIBRATION with increasing raw counts and angles.
       Do not include indistinguishable points (75 and 90 deg overlap here).
    4. Set CALIBRATE_MODE 0 and re-upload. Command limits are independent.

  Libraries needed: micro_ros_arduino (Humble branch)
*/

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ── WiFi / agent ──────────────────────────────────────────
#include "wifi_config.h"

// ── Potentiometer ─────────────────────────────────────────
const int POT_PIN = 34;   // ADC1_CH6 — input-only pin, safe with WiFi on
const int SHOULDER_PWM_PIN = 32;
const int SHOULDER_PWM_CHANNEL = 0;
const unsigned int SHOULDER_PWM_HZ = 20000;
const unsigned int SHOULDER_PWM_BITS = 8;
const int MOTOR_BIN1_PIN = 14;
const int MOTOR_BIN2_PIN = 13;
const int MOTOR_STBY_PIN = 25;
const int MOTOR_AIN1_PIN = 26;
const int MOTOR_AIN2_PIN = 27;
// Initial gains only: duty percent per degree, per degree-second, and per deg/s.
float pid_kp = 0.6f;
float pid_ki = 0.1f;
float pid_kd = 0.04f;
const float PID_MAX_DUTY = 15.0f;
const float PID_MIN_ANGLE = 5.0f;
const float PID_MAX_ANGLE = 70.0f;  // Stay inside the calibrated 0..75 degree range.
const unsigned long TARGET_TIMEOUT_MS = 1000;
const unsigned long CONTROL_TIMEOUT_MS = 150;
bool pid_enabled = false;
bool estop_active = false;
bool target_valid = false;
bool measurement_valid = false;
float target_deg = 0.0f;
float measured_deg = 0.0f;
float integral_duty = 0.0f;
float previous_measurement = 0.0f;
float filtered_velocity = 0.0f;
unsigned long last_target_ms = 0;
unsigned long last_measurement_ms = 0;
unsigned long last_control_ms = 0;
float proportional_duty = 0.0f;
float derivative_duty = 0.0f;
float output_duty = 0.0f;
enum PidFault {
  PID_FAULT_NONE = 0,
  PID_FAULT_TARGET = 1,
  PID_FAULT_FEEDBACK = 2,
  PID_FAULT_CONTROL_TIMEOUT = 3,
  PID_FAULT_ESTOP = 4
};
unsigned int pid_fault = PID_FAULT_NONE;

// Set to 1 to print raw ADC values for calibration, skip ROS publishing.
#define CALIBRATE_MODE 0

// Measured descending sweep with the installed divider. Values above the last
// anchor saturate at 75 degrees: this sensor cannot distinguish 75 from 90.
struct ShoulderCalibrationPoint {
  int raw;
  float degrees;
};
constexpr ShoulderCalibrationPoint SHOULDER_CALIBRATION[] = {
  {2319, 0.0f},
  {2525, 15.0f},
  {2687, 30.0f},
  {2874, 45.0f},
  {3037, 60.0f},
  {3100, 75.0f},
};
constexpr unsigned int CALIBRATION_COUNT =
    sizeof(SHOULDER_CALIBRATION) / sizeof(SHOULDER_CALIBRATION[0]);

const int   SAMPLES = 16;   // averaged per reading, reduces ADC noise

// ── micro-ROS objects ─────────────────────────────────────
rcl_publisher_t pub_feedback, pub_telemetry;
std_msgs__msg__Float32 feedback_msg;
rcl_subscription_t sub_target, sub_enable, sub_estop, sub_gains;
std_msgs__msg__Float32 target_msg;
std_msgs__msg__Bool enable_msg, estop_msg;
// All sequence backing storage is static. Oversized gain sequences (or a
// nonempty layout.dim sequence) are rejected by micro-ROS deserialization,
// never copied past these capacities or applied by the callback.
float gains_storage[3] = {0.0f, 0.0f, 0.0f};
float telemetry_storage[14] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.6f, 0.1f, 0.04f, PID_MAX_DUTY, 0.0f
};
std_msgs__msg__Float32MultiArray gains_msg = {};
std_msgs__msg__Float32MultiArray telemetry_msg = {};

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
  constexpr unsigned int count = CALIBRATION_COUNT;
  constexpr float radiansPerDegree = 0.0174533f;
  if (raw <= SHOULDER_CALIBRATION[0].raw) {
    return SHOULDER_CALIBRATION[0].degrees * radiansPerDegree;
  }
  for (unsigned int i = 1; i < count; ++i) {
    const auto &upper = SHOULDER_CALIBRATION[i];
    if (raw <= upper.raw) {
      const auto &lower = SHOULDER_CALIBRATION[i - 1];
      const float fraction = (float)(raw - lower.raw) /
                             (float)(upper.raw - lower.raw);
      return (lower.degrees + fraction * (upper.degrees - lower.degrees)) *
             radiansPerDegree;
    }
  }
  return SHOULDER_CALIBRATION[count - 1].degrees * radiansPerDegree;
}

void stop_shoulder_motor() {
  digitalWrite(MOTOR_STBY_PIN, LOW);
  ledcWrite(SHOULDER_PWM_CHANNEL, 0);
  digitalWrite(MOTOR_BIN1_PIN, LOW);
  output_duty = 0.0f;
  digitalWrite(MOTOR_BIN2_PIN, LOW);
}

void disable_pid() {
  pid_enabled = false;
  proportional_duty = 0.0f;
  integral_duty = 0.0f;
  derivative_duty = 0.0f;
  filtered_velocity = 0.0f;
  stop_shoulder_motor();
}

void fault_pid(unsigned int fault) {
  pid_fault = fault;
  disable_pid();
}

void apply_motor_duty(float duty) {
  duty = constrain(duty, -PID_MAX_DUTY, PID_MAX_DUTY);
  const unsigned int pwm = (unsigned int)(fabsf(duty) * 2.55f + 0.5f);
  if (pwm == 0) {
    stop_shoulder_motor();
    return;
  }
  digitalWrite(MOTOR_STBY_PIN, LOW);
  digitalWrite(MOTOR_BIN1_PIN, duty > 0.0f ? HIGH : LOW);
  digitalWrite(MOTOR_BIN2_PIN, duty > 0.0f ? LOW : HIGH);
  ledcWrite(SHOULDER_PWM_CHANNEL, pwm);
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  output_duty = (duty > 0.0f ? 1.0f : -1.0f) *
                (float)pwm * (100.0f / 255.0f);
}

void target_callback(const void *msgin) {
  const float value = static_cast<const std_msgs__msg__Float32 *>(msgin)->data;
  if (estop_active) {
    target_valid = false;
    fault_pid(PID_FAULT_ESTOP);
    return;
  }
  if (!isfinite(value) || value < PID_MIN_ANGLE || value > PID_MAX_ANGLE) {
    target_valid = false;
    fault_pid(PID_FAULT_TARGET);
    return;
  }
  target_deg = value;
  last_target_ms = millis();
  target_valid = true;
  // Target refresh never resets an active controller or clears other faults.
  if (pid_fault == PID_FAULT_TARGET) pid_fault = PID_FAULT_NONE;
}

void gains_callback(const void *msgin) {
  const auto *msg = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);
  if (pid_enabled || estop_active || msg->data.size != 3) return;
  const float kp = msg->data.data[0];
  const float ki = msg->data.data[1];
  const float kd = msg->data.data[2];
  if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
      kp < 0.0f || kp > 5.0f || ki < 0.0f || ki > 2.0f ||
      kd < 0.0f || kd > 1.0f) return;
  pid_kp = kp;
  pid_ki = ki;
  pid_kd = kd;
}

void enable_callback(const void *msgin) {
  const bool requested = static_cast<const std_msgs__msg__Bool *>(msgin)->data;
  const unsigned long now = millis();
  if (!requested) {
    disable_pid();  // An ordinary disable does not erase a diagnostic fault.
    return;
  }
  if (estop_active) {
    fault_pid(PID_FAULT_ESTOP);
    return;
  }
  if (!measurement_valid) {
    fault_pid(PID_FAULT_FEEDBACK);
    return;
  }
  if (!target_valid || now - last_target_ms >= TARGET_TIMEOUT_MS) {
    target_valid = false;
    fault_pid(PID_FAULT_TARGET);
    return;
  }
  if (now - last_measurement_ms >= CONTROL_TIMEOUT_MS) {
    fault_pid(PID_FAULT_CONTROL_TIMEOUT);
    return;
  }
  // Repeated enable messages must not keep resetting an active controller.
  if (!pid_enabled) {
    proportional_duty = 0.0f;
    integral_duty = 0.0f;
    derivative_duty = 0.0f;
    filtered_velocity = 0.0f;
    previous_measurement = measured_deg;
    last_control_ms = now;
    pid_enabled = true;
  }
  pid_fault = PID_FAULT_NONE;
}

void estop_callback(const void *msgin) {
  estop_active = static_cast<const std_msgs__msg__Bool *>(msgin)->data;
  if (estop_active) {
    target_valid = false;
    fault_pid(PID_FAULT_ESTOP);
  }
}

// 20 Hz PID with measured dt, derivative on measurement, low-pass velocity,
// conditional integration anti-windup, and a bounded signed duty output.
void publish_timer_callback(rcl_timer_t *t, int64_t last_call_time) {
  (void)last_call_time;
  if (t == NULL) return;
  const int raw = readPotRaw();
  const float shoulder_rad = rawToShoulderRad(raw);
  const float angle_deg = shoulder_rad / 0.0174533f;
  const unsigned long now = millis();
  last_measurement_ms = now;
  measurement_valid = raw >= SHOULDER_CALIBRATION[0].raw &&
                      raw <= SHOULDER_CALIBRATION[CALIBRATION_COUNT - 1].raw &&
                      isfinite(shoulder_rad) && isfinite(angle_deg) &&
                      angle_deg >= PID_MIN_ANGLE && angle_deg <= PID_MAX_ANGLE;
  // Keep published values finite even if calibration is accidentally invalid.
  measured_deg = isfinite(angle_deg) ? angle_deg : 0.0f;
  feedback_msg.data = isfinite(shoulder_rad) ? shoulder_rad : 0.0f;
  const unsigned long elapsed_ms = now - last_control_ms;
  if (estop_active) {
    fault_pid(PID_FAULT_ESTOP);
  } else if (!measurement_valid) {
    fault_pid(PID_FAULT_FEEDBACK);
  } else if (target_valid && now - last_target_ms >= TARGET_TIMEOUT_MS) {
    target_valid = false;
    // Do not erase a latched feedback, control-timeout, or released-estop fault.
    if (pid_enabled || pid_fault == PID_FAULT_NONE) {
      fault_pid(PID_FAULT_TARGET);
    }
  } else if (pid_enabled && !target_valid) {
    fault_pid(PID_FAULT_TARGET);
  } else if (pid_enabled && elapsed_ms >= CONTROL_TIMEOUT_MS) {
    fault_pid(PID_FAULT_CONTROL_TIMEOUT);
  } else if (pid_enabled && elapsed_ms != 0) {
    last_control_ms = now;
    const float dt = elapsed_ms * 0.001f;
    const float velocity = (measured_deg - previous_measurement) / dt;
    previous_measurement = measured_deg;
    filtered_velocity += dt / (0.1f + dt) * (velocity - filtered_velocity);
    const float error = target_deg - measured_deg;
    proportional_duty = pid_kp * error;
    derivative_duty = -pid_kd * filtered_velocity;
    const float candidate_integral = constrain(
        integral_duty + pid_ki * error * dt, -PID_MAX_DUTY, PID_MAX_DUTY);
    const float proportional_derivative = proportional_duty + derivative_duty;
    const float candidate_output = proportional_derivative + candidate_integral;
    if (!((candidate_output > PID_MAX_DUTY && error > 0.0f) ||
          (candidate_output < -PID_MAX_DUTY && error < 0.0f))) {
      integral_duty = candidate_integral;
    }
    apply_motor_duty(proportional_derivative + integral_duty);
  }

  // Safety decisions above stop the motor before either publication.
  telemetry_storage[0] = target_deg;
  telemetry_storage[1] = measured_deg;
  telemetry_storage[2] = target_deg - measured_deg;
  telemetry_storage[3] = proportional_duty;
  telemetry_storage[4] = integral_duty;
  telemetry_storage[5] = derivative_duty;
  telemetry_storage[6] = output_duty;
  telemetry_storage[7] = pid_enabled ? 1.0f : 0.0f;
  telemetry_storage[8] = estop_active ? 1.0f : 0.0f;
  telemetry_storage[9] = pid_kp;
  telemetry_storage[10] = pid_ki;
  telemetry_storage[11] = pid_kd;
  telemetry_storage[12] = PID_MAX_DUTY;
  telemetry_storage[13] = (float)pid_fault;
  rcl_publish(&pub_feedback, &feedback_msg, NULL);
  rcl_publish(&pub_telemetry, &telemetry_msg, NULL);
}

// ── micro-ROS init ────────────────────────────────────────
#define RCCHECK(fn) do { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
    Serial.printf("micro-ROS error %d at %s\n", (int) rc, #fn); \
    error_loop(); \
  } \
} while (0)

void error_loop() {
  disable_pid();
  while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

void microros_init() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "shoulder_pot_feedback", "", &support));
  gains_msg.data.data = gains_storage;
  gains_msg.data.size = 0;
  gains_msg.data.capacity = 3;
  telemetry_msg.data.data = telemetry_storage;
  telemetry_msg.data.size = 14;
  telemetry_msg.data.capacity = 14;

  RCCHECK(rclc_publisher_init_default(&pub_feedback, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/shoulder_feedback"));
  RCCHECK(rclc_publisher_init_default(&pub_telemetry, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/shoulder_pid_telemetry"));

  RCCHECK(rclc_timer_init_default(&timer, &support,
      RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_timer_callback));

  RCCHECK(rclc_subscription_init_default(&sub_target, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/shoulder_target_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_enable, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/shoulder_pid_enable"));
  RCCHECK(rclc_subscription_init_default(&sub_gains, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/shoulder_pid_gains"));
  rmw_qos_profile_t estop_qos = rmw_qos_profile_default;
  estop_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  estop_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  RCCHECK(rclc_subscription_init(&sub_estop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/estop", &estop_qos));
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  // Process targets and safety state before advancing the control timer.
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_estop, &estop_msg, estop_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_target, &target_msg, target_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_enable, &enable_msg, enable_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_gains, &gains_msg, gains_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, LOW);
  pinMode(MOTOR_BIN1_PIN, OUTPUT);
  pinMode(MOTOR_BIN2_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  ledcSetup(SHOULDER_PWM_CHANNEL, SHOULDER_PWM_HZ, SHOULDER_PWM_BITS);
  ledcAttachPin(SHOULDER_PWM_PIN, SHOULDER_PWM_CHANNEL);
  stop_shoulder_motor();
  analogReadResolution(12);          // 0-4095
  analogSetAttenuation(ADC_11db);    // full 0-3.3V range

#if CALIBRATE_MODE
  while (true) {
    Serial.println(readPotRaw());
    delay(200);
  }
#endif

  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int attempts = 0;
       WiFi.status() != WL_CONNECTED && attempts < 40;
       attempts++) {
    Serial.print('.');
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\nWiFi failed, status=%d. Check SSID, password, and 2.4 GHz.\n",
                  (int) WiFi.status());
    error_loop();
  }
  Serial.print("\nWiFi connected, ESP32 IP: ");
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports(
      (char*) WIFI_SSID, (char*) WIFI_PASS,
      (char*) AGENT_IP, AGENT_PORT);
  delay(2000);

  Serial.printf("Initializing micro-ROS agent at %s:%d\n", AGENT_IP, AGENT_PORT);
  microros_init();
  Serial.println("Ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  const unsigned long now = millis();
  if (pid_enabled) {
    if (now - last_target_ms >= TARGET_TIMEOUT_MS) {
      target_valid = false;
      fault_pid(PID_FAULT_TARGET);
    } else if (now - last_control_ms >= CONTROL_TIMEOUT_MS) {
      fault_pid(PID_FAULT_CONTROL_TIMEOUT);
    }
  }
  delay(10);
}
