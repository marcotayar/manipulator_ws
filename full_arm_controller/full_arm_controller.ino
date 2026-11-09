/*
  ESP32 full-arm controller: shoulder/elbow/wrist position PID + base/gripper servos
  ==================================================================================

  Hardware map (docs/ligacao_pontes_h_esp32.pdf, docs/full_arm_wiring.html):
    Shoulder: TB6612 #1 A, PWM=19, IN1=18, IN2=5,  pot wiper -> GPIO34
    Base:     TB6612 #1 B, PWM=17, IN1=16, IN2=15  (held LOW: base not controlled yet)
    Elbow:    TB6612 #2 B, PWM=32, IN1=14, IN2=13, pot wiper -> GPIO39 (VN)
    Wrist:    TB6612 #2 A, PWM=23, IN1=22, IN2=21, pot wiper -> GPIO36 (VP)
    Both TB6612 STBY pins: GPIO4
    Gripper positional servo signal: GPIO26

  Potentiometers are 5 V referenced and wired directly to the ADC pins. Only the
  mechanical joint limits keep each wiper below 3.3 V; there is no electrical
  overvoltage protection. Calibration tables below are valid only for the
  current pots, mounting, and wiring (see full_arm_controller/*.csv).

  Angles are PHYSICAL joint angles in degrees (90 = home pose, arm vertical).
  They are not the arm_ik_2d.py IK frame; /arm_command joint angles are ignored.

  ROS interface (micro-ROS over Wi-Fi, see wifi_config.example.h):
    /arm_joint_targets_deg (Float32MultiArray): exactly [shoulder, elbow, wrist],
        each 85..175 deg, refreshed at least every 1 s.
    /arm_pid_enable (Int32): bitmask, bit0 shoulder, bit1 elbow, bit2 wrist.
        0 disables all. Boots disabled. Enabling requires fresh valid targets,
        valid feedback, and no e-stop.
    /arm_pid_gains (Float32MultiArray): exactly 12 values
        [Kp,Ki,Kd,G shoulder, Kp,Ki,Kd,G elbow, Kp,Ki,Kd,G wrist]; accepted only
        while all joints are disabled and e-stop is released. G is the gravity
        feedforward in percent duty with the link horizontal (0..50, default 0):
        feedforward = -G * sin(lean), lean = sum of (angle - 90) from the
        shoulder out to that joint. (Separate topic impossible: the prebuilt
        micro-ROS library allows at most 5 subscriptions.)
    /estop (Bool, transient-local): true disables all joints.
    /arm_command (Float32MultiArray, existing arm_commander contract): only
        [4] gripper 0 open..1 closed is used.
    /arm_joint_telemetry (Float32MultiArray, 20 Hz): 16 values
        [shoulder_deg, elbow_deg, wrist_deg,
         shoulder_raw, elbow_raw, wrist_raw,
         shoulder_target, elbow_target, wrist_target,
         shoulder_duty, elbow_duty, wrist_duty,
         enabled_mask, fault_code, fault_joint, estop]
        Angles are -1 when the reading is outside the calibrated table.
        Fault codes: 0 none, 1 invalid/stale target, 2 feedback invalid or
        outside 80..178 deg, 3 control timeout, 4 e-stop, 5 runaway (joint
        moved 4 deg opposite to the applied push: wrong direction or sag),
        6 stall (output at the duty cap for 1 s with under 1 deg of motion).
        fault_joint: 0 none/all, 1 shoulder, 2 elbow, 3 wrist.

  Set CALIBRATE_MODE 1 to print raw ADC CSV over USB with every actuator
  disabled and Wi-Fi off.
*/

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>

#include "wifi_config.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define CALIBRATE_MODE 0

// ── Calibration (raw ADC -> physical degrees) ──────────────────────────────
struct CalPoint {
  int raw;
  float deg;
};
// ── Joints ──────────────────────────────────────────────────────────────────
struct Joint {
  int pwm_pin, in1_pin, in2_pin, adc_pin, pwm_channel;
  const CalPoint *cal;
  size_t cal_count;
  // +1: IN1 HIGH/IN2 LOW increases the measured angle. Verify per joint during
  // commissioning; runaway detection faults if the sign is wrong.
  float direction;
  float kp, ki, kd;
  int raw;
  float measured_deg;
  bool measurement_valid;
  float target_deg;
  float integral, previous_deg, filtered_velocity;
  float p_term, d_term, output;
  float setpoint_deg;     // ramps toward target_deg at SETPOINT_RATE_DEG_S
  float runaway_ref_deg;  // angle when the current push direction started
  int runaway_sign;       // +1/-1 push direction, 0 idle
  float stall_ref_deg;    // angle when output reached the duty cap
  unsigned long stall_start_ms;  // 0 = output below the cap
  float gravity_ff;       // feedforward duty holding this joint's distal links
};

// Shoulder: midpoint of ascending/descending passes (shoulder_calibration_table.csv).
constexpr CalPoint SHOULDER_CAL[] = {
    {1639, 75.0f}, {2025, 90.0f}, {2406, 105.0f}, {2687, 120.0f},
    {2947, 135.0f}, {3160, 150.0f}, {3493, 165.0f}, {3892, 180.0f},
};
// Elbow: standardized ascending pass (elbow_calibration_capture.csv).
constexpr CalPoint ELBOW_CAL[] = {
    {272, 75.0f}, {584, 85.0f}, {750, 90.0f}, {1133, 105.0f}, {1462, 120.0f},
    {1931, 135.0f}, {2395, 150.0f}, {2868, 165.0f}, {3050, 175.0f}, {3277, 180.0f},
};
// Wrist: decreasing raw with angle (wrist_calibration_capture.csv).
constexpr CalPoint WRIST_CAL[] = {
    {3469, 75.0f}, {3304, 85.0f}, {3010, 90.0f}, {2500, 105.0f}, {1902, 120.0f},
    {1656, 135.0f}, {1164, 150.0f}, {801, 165.0f}, {508, 175.0f}, {332, 180.0f},
};

// Piecewise-linear interpolation; works for increasing or decreasing tables.
// Returns NAN outside the measured table (never extrapolates).
float rawToDegrees(int raw, const CalPoint *cal, size_t count) {
  for (size_t i = 0; i + 1 < count; ++i) {
    const CalPoint &a = cal[i];
    const CalPoint &b = cal[i + 1];
    const int lo = min(a.raw, b.raw);
    const int hi = max(a.raw, b.raw);
    if (raw >= lo && raw <= hi) {
      const float fraction = (float)(raw - a.raw) / (float)(b.raw - a.raw);
      return a.deg + fraction * (b.deg - a.deg);
    }
  }
  return NAN;
}

// ── Limits and timing ───────────────────────────────────────────────────────
constexpr float TARGET_MIN_DEG = 85.0f;
constexpr float TARGET_MAX_DEG = 175.0f;
constexpr float SAFE_MIN_DEG = 80.0f;   // measured angle outside -> fault
constexpr float SAFE_MAX_DEG = 178.0f;
constexpr float DEADBAND_DEG = 1.5f;    // calibration/backlash is ~2-3 deg
constexpr float RUNAWAY_DEG = 4.0f;
constexpr float STALL_MOVE_DEG = 1.0f;          // less motion than this at the cap = stalled
constexpr unsigned long STALL_TIME_MS = 1000;   // protects TB6612 (1.2 A continuous)
constexpr float SETPOINT_RATE_DEG_S = 15.0f;  // target jumps become a gentle ramp
constexpr float SETPOINT_MAX_LAG_DEG = 6.0f;  // ramp waits for a joint held by friction
constexpr float MAX_DUTY = 50.0f;       // percent; joints need >20% to break stiction
constexpr unsigned long TARGET_TIMEOUT_MS = 1000;
constexpr unsigned long CONTROL_TIMEOUT_MS = 300;  // micro-ROS over Wi-Fi jitters >150 ms
constexpr unsigned int CONTROL_PERIOD_MS = 50;  // 20 Hz

constexpr int MOTOR_STBY_PIN = 4;
constexpr unsigned int MOTOR_PWM_HZ = 20000;
constexpr unsigned int MOTOR_PWM_BITS = 8;

// Base DC motor channel: held LOW (coast) until base control is implemented.
constexpr int BASE_PWM_PIN = 17;
constexpr int BASE_IN1_PIN = 16;
constexpr int BASE_IN2_PIN = 15;

// Gripper servo on LEDC timer 2 (channel 4) at 50 Hz; motors use timers 0/1.
constexpr int GRIPPER_SERVO_PIN = 26;
constexpr int GRIPPER_SERVO_CHANNEL = 4;
constexpr unsigned int SERVO_HZ = 50;
constexpr unsigned int SERVO_BITS = 16;
constexpr float GRIP_OPEN_DEG = 80.0f;
constexpr float GRIP_CLOSED_DEG = 140.0f;

enum Fault : unsigned int {
  FAULT_NONE = 0,
  FAULT_TARGET = 1,
  FAULT_FEEDBACK = 2,
  FAULT_CONTROL_TIMEOUT = 3,
  FAULT_ESTOP = 4,
  FAULT_RUNAWAY = 5,
  FAULT_STALL = 6,
};


Joint joints[3] = {
    {19, 18, 5, 34, 0, SHOULDER_CAL, sizeof(SHOULDER_CAL) / sizeof(CalPoint),
     -1.0f, 2.0f, 0.3f, 0.04f},  // verified: IN1 HIGH lowers the shoulder
    {32, 14, 13, 39, 1, ELBOW_CAL, sizeof(ELBOW_CAL) / sizeof(CalPoint),
     +1.0f, 4.0f, 1.0f, 0.04f},  // verified: IN1 HIGH raises the elbow
    {23, 22, 21, 36, 2, WRIST_CAL, sizeof(WRIST_CAL) / sizeof(CalPoint),
     +1.0f, 4.0f, 1.0f, 0.04f},  // verified: IN1 HIGH raises the wrist
};
constexpr int JOINT_COUNT = 3;

unsigned int enabled_mask = 0;
unsigned int fault_code = FAULT_NONE;
unsigned int fault_joint = 0;
bool estop_active = false;
bool targets_valid = false;
unsigned long last_target_ms = 0;
unsigned long last_control_ms = 0;
float gripper_closure = 0.0f;

// Control runs in its own FreeRTOS task so Wi-Fi/micro-ROS stalls never stall
// the motor loop. The mutex guards all control state shared with callbacks.
SemaphoreHandle_t state_mutex = nullptr;
struct StateLock {
  StateLock() { xSemaphoreTake(state_mutex, portMAX_DELAY); }
  ~StateLock() { xSemaphoreGive(state_mutex); }
};
float telemetry_snapshot[16];

// ── micro-ROS objects ───────────────────────────────────────────────────────
rcl_publisher_t pub_telemetry;
rcl_subscription_t sub_targets, sub_enable, sub_gains, sub_estop, sub_arm_command;
std_msgs__msg__Int32 enable_msg;
std_msgs__msg__Bool estop_msg;
// Static backing storage; oversized sequences are rejected by deserialization.
float targets_storage[3];
float gains_storage[12];
// Percent duty with the link horizontal. Wrist 25 verified on hardware (holds at
// 118 deg, returns to 99 deg); shoulder/elbow not yet identified.
float gravity_gain[3] = {0.0f, 0.0f, 25.0f};
float arm_command_storage[8];
float telemetry_storage[16];
std_msgs__msg__Float32MultiArray targets_msg = {};
std_msgs__msg__Float32MultiArray gains_msg = {};
std_msgs__msg__Float32MultiArray arm_command_msg = {};
std_msgs__msg__Float32MultiArray telemetry_msg = {};
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ── ADC ─────────────────────────────────────────────────────────────────────
int readSettledRaw(int pin) {
  analogRead(pin);  // discard the first conversion after switching channels
  delayMicroseconds(500);
  long sum = 0;
  for (int i = 0; i < 32; ++i) sum += analogRead(pin);
  return (int)(sum / 32);
}

void measureJoint(Joint &j) {
  j.raw = readSettledRaw(j.adc_pin);
  const float deg = rawToDegrees(j.raw, j.cal, j.cal_count);
  j.measurement_valid = isfinite(deg);
  j.measured_deg = j.measurement_valid ? deg : -1.0f;
}

bool jointInSafeRange(const Joint &j) {
  return j.measurement_valid && j.measured_deg >= SAFE_MIN_DEG &&
         j.measured_deg <= SAFE_MAX_DEG;
}

// ── Motor output ────────────────────────────────────────────────────────────
void coastJoint(Joint &j) {
  ledcWrite(j.pwm_channel, 0);
  digitalWrite(j.in1_pin, LOW);
  digitalWrite(j.in2_pin, LOW);
  j.output = 0.0f;
}

void resetPid(Joint &j) {
  j.integral = 0.0f;
  j.filtered_velocity = 0.0f;
  j.p_term = 0.0f;
  j.d_term = 0.0f;
  j.previous_deg = j.measured_deg;
  j.setpoint_deg = j.measured_deg;
  j.runaway_ref_deg = j.measured_deg;
  j.runaway_sign = 0;
  j.stall_start_ms = 0;
}

void updateStandby() {
  digitalWrite(MOTOR_STBY_PIN, enabled_mask != 0 ? HIGH : LOW);
}

void disableAll() {
  enabled_mask = 0;
  digitalWrite(MOTOR_STBY_PIN, LOW);
  for (Joint &j : joints) {
    coastJoint(j);
    resetPid(j);
  }
}

void fault(unsigned int code, unsigned int joint_number) {
  fault_code = code;
  fault_joint = joint_number;
  disableAll();
}

void applyDuty(Joint &j, float duty) {
  duty = constrain(duty, -MAX_DUTY, MAX_DUTY);
  const unsigned int pwm = (unsigned int)(fabsf(duty) * 2.55f + 0.5f);
  if (pwm == 0) {
    coastJoint(j);
    return;
  }
  const bool forward = duty * j.direction > 0.0f;
  digitalWrite(j.in1_pin, forward ? HIGH : LOW);
  digitalWrite(j.in2_pin, forward ? LOW : HIGH);
  ledcWrite(j.pwm_channel, pwm);
  j.output = (duty > 0.0f ? 1.0f : -1.0f) * (float)pwm * (100.0f / 255.0f);
}

// ── Servos ──────────────────────────────────────────────────────────────────
void writeServoMicroseconds(int channel, int microseconds) {
  const uint32_t period_us = 1000000UL / SERVO_HZ;
  const uint32_t duty = (uint32_t)((uint64_t)microseconds * 65535ULL / period_us);
  ledcWrite(channel, duty);
}

void writeGripper(float closure) {
  closure = constrain(closure, 0.0f, 1.0f);
  const float deg = GRIP_OPEN_DEG + closure * (GRIP_CLOSED_DEG - GRIP_OPEN_DEG);
  writeServoMicroseconds(GRIPPER_SERVO_CHANNEL, 500 + (int)(deg / 180.0f * 2000.0f));
}

// ── Callbacks ───────────────────────────────────────────────────────────────
void targetsCallback(const void *msgin) {
  StateLock lock;
  const auto *msg = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);
  if (estop_active) {
    targets_valid = false;
    fault(FAULT_ESTOP, 0);
    return;
  }
  if (msg->data.size != JOINT_COUNT) {
    targets_valid = false;
    fault(FAULT_TARGET, 0);
    return;
  }
  for (int i = 0; i < JOINT_COUNT; ++i) {
    const float value = msg->data.data[i];
    if (!isfinite(value) || value < TARGET_MIN_DEG || value > TARGET_MAX_DEG) {
      targets_valid = false;
      fault(FAULT_TARGET, i + 1);
      return;
    }
  }
  for (int i = 0; i < JOINT_COUNT; ++i) {
    Joint &j = joints[i];
    const float value = msg->data.data[i];
    j.target_deg = value;
  }
  last_target_ms = millis();
  targets_valid = true;
  if (fault_code == FAULT_TARGET) {
    fault_code = FAULT_NONE;
    fault_joint = 0;
  }
}

void enableCallback(const void *msgin) {
  StateLock lock;
  const int32_t requested = static_cast<const std_msgs__msg__Int32 *>(msgin)->data;
  const unsigned long now = millis();
  if (requested <= 0) {
    disableAll();  // ordinary disable keeps the last diagnostic fault visible
    return;
  }
  const unsigned int mask = (unsigned int)requested & 0x7u;
  if (mask == 0) {
    disableAll();
    return;
  }
  if (estop_active) {
    fault(FAULT_ESTOP, 0);
    return;
  }
  if (!targets_valid || now - last_target_ms >= TARGET_TIMEOUT_MS) {
    targets_valid = false;
    fault(FAULT_TARGET, 0);
    return;
  }
  for (int i = 0; i < JOINT_COUNT; ++i) {
    if ((mask & (1u << i)) && !jointInSafeRange(joints[i])) {
      fault(FAULT_FEEDBACK, i + 1);
      return;
    }
  }
  for (int i = 0; i < JOINT_COUNT; ++i) {
    const unsigned int bit = 1u << i;
    if (mask & bit) {
      if (!(enabled_mask & bit)) resetPid(joints[i]);
    } else {
      coastJoint(joints[i]);
      resetPid(joints[i]);
    }
  }
  if (enabled_mask == 0) last_control_ms = now;
  enabled_mask = mask;
  fault_code = FAULT_NONE;
  fault_joint = 0;
  updateStandby();
}

void gainsCallback(const void *msgin) {
  StateLock lock;
  const auto *msg = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);
  if (enabled_mask != 0 || estop_active || msg->data.size != 12) return;
  for (int i = 0; i < JOINT_COUNT; ++i) {
    const float kp = msg->data.data[4 * i];
    const float ki = msg->data.data[4 * i + 1];
    const float kd = msg->data.data[4 * i + 2];
    const float g = msg->data.data[4 * i + 3];
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(g) ||
        kp < 0.0f || kp > 5.0f || ki < 0.0f || ki > 2.0f || kd < 0.0f || kd > 1.0f ||
        g < 0.0f || g > MAX_DUTY) {
      return;  // reject the whole message; never apply a partial update
    }
  }
  for (int i = 0; i < JOINT_COUNT; ++i) {
    joints[i].kp = msg->data.data[4 * i];
    joints[i].ki = msg->data.data[4 * i + 1];
    joints[i].kd = msg->data.data[4 * i + 2];
    gravity_gain[i] = msg->data.data[4 * i + 3];
  }
}

// Gravity pulls each link further from vertical. Positive duty increases the
// angle, so the hold is -G * sin(lean). Lean accumulates outward from the shoulder
// (assumes every joint's angle increases in the same rotational sense). A joint
// whose lean is unknown (invalid proximal reading) gets no feedforward.
void updateGravityFeedforward() {
  float lean_deg = 0.0f;
  bool lean_valid = true;
  for (int i = 0; i < JOINT_COUNT; ++i) {
    lean_valid = lean_valid && joints[i].measurement_valid;
    lean_deg += joints[i].measured_deg - 90.0f;
    joints[i].gravity_ff =
        lean_valid ? -gravity_gain[i] * sinf(lean_deg * 0.0174533f) : 0.0f;
  }
}

void estopCallback(const void *msgin) {
  StateLock lock;
  estop_active = static_cast<const std_msgs__msg__Bool *>(msgin)->data;
  if (estop_active) {
    targets_valid = false;
    fault(FAULT_ESTOP, 0);
  }
}

void armCommandCallback(const void *msgin) {
  const auto *msg = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);
  if (msg->data.size < 5) return;
  const float grip = msg->data.data[4];
  if (!isfinite(grip) || estop_active) return;
  gripper_closure = grip;
  writeGripper(gripper_closure);
}

// ── 20 Hz control ───────────────────────────────────────────────────────────
void controlJoint(Joint &j, int index, float dt, unsigned long now) {
  const float velocity = (j.measured_deg - j.previous_deg) / dt;
  j.previous_deg = j.measured_deg;
  j.filtered_velocity += dt / (0.1f + dt) * (velocity - j.filtered_velocity);
  // Ramp the internal setpoint so a large target jump never becomes a large error.
  const float max_step = SETPOINT_RATE_DEG_S * dt;
  j.setpoint_deg += constrain(j.target_deg - j.setpoint_deg, -max_step, max_step);
  j.setpoint_deg = constrain(j.setpoint_deg, j.measured_deg - SETPOINT_MAX_LAG_DEG,
                             j.measured_deg + SETPOINT_MAX_LAG_DEG);
  const float error = j.setpoint_deg - j.measured_deg;
  const float abs_error = fabsf(error);

  if (abs_error <= DEADBAND_DEG && fabsf(j.target_deg - j.setpoint_deg) < 0.01f) {
    // Inside calibration/backlash uncertainty: stop correcting, but keep applying
    // the frozen integral so a gravity-loaded joint holds instead of falling.
    j.p_term = 0.0f;
    j.d_term = 0.0f;
    applyDuty(j, j.gravity_ff + j.integral);
  } else {
    j.p_term = j.kp * error;
    j.d_term = -j.kd * j.filtered_velocity;
    const float candidate_integral =
        constrain(j.integral + j.ki * error * dt, -MAX_DUTY, MAX_DUTY);
    const float pd = j.p_term + j.d_term;
    const float candidate_output = j.gravity_ff + pd + candidate_integral;
    // Conditional integration anti-windup.
    if (!((candidate_output > MAX_DUTY && error > 0.0f) ||
          (candidate_output < -MAX_DUTY && error < 0.0f))) {
      j.integral = candidate_integral;
    }
    applyDuty(j, j.gravity_ff + pd + j.integral);
  }

  // Runaway: the joint moved RUNAWAY_DEG opposite to the push being applied
  // (reversed motor wiring/direction, or the load overpowering the motor).
  // Overshoot is not runaway: it moves the way the motor was pushing.
  const int sign = j.output > 1.0f ? 1 : (j.output < -1.0f ? -1 : 0);
  if (sign != j.runaway_sign) {
    j.runaway_sign = sign;
    j.runaway_ref_deg = j.measured_deg;
  } else if (sign != 0 && (j.measured_deg - j.runaway_ref_deg) * sign < -RUNAWAY_DEG) {
    fault(FAULT_RUNAWAY, index + 1);
    return;
  }

  // Stall: output pinned at the cap without motion overheats the bridge/motor.
  if (fabsf(j.output) >= 0.95f * MAX_DUTY) {
    if (j.stall_start_ms == 0 || fabsf(j.measured_deg - j.stall_ref_deg) >= STALL_MOVE_DEG) {
      j.stall_start_ms = now;
      j.stall_ref_deg = j.measured_deg;
    } else if (now - j.stall_start_ms >= STALL_TIME_MS) {
      fault(FAULT_STALL, index + 1);
    }
  } else {
    j.stall_start_ms = 0;
  }
}

// One 20 Hz control step. Caller holds state_mutex.
void controlStep() {
  for (Joint &j : joints) measureJoint(j);
  const unsigned long now = millis();
  const unsigned long elapsed_ms = now - last_control_ms;

  if (estop_active) {
    if (enabled_mask != 0) fault(FAULT_ESTOP, 0);
  } else if (targets_valid && now - last_target_ms >= TARGET_TIMEOUT_MS) {
    targets_valid = false;
    if (enabled_mask != 0 || fault_code == FAULT_NONE) fault(FAULT_TARGET, 0);
  } else if (enabled_mask != 0) {
    for (int i = 0; i < JOINT_COUNT; ++i) {
      if ((enabled_mask & (1u << i)) && !jointInSafeRange(joints[i])) {
        fault(FAULT_FEEDBACK, i + 1);
        break;
      }
    }
    if (enabled_mask != 0 && elapsed_ms >= CONTROL_TIMEOUT_MS) {
      fault(FAULT_CONTROL_TIMEOUT, 0);
    } else if (enabled_mask != 0 && elapsed_ms != 0) {
      last_control_ms = now;
      const float dt = elapsed_ms * 0.001f;
      updateGravityFeedforward();
      for (int i = 0; i < JOINT_COUNT && enabled_mask != 0; ++i) {
        if (enabled_mask & (1u << i)) controlJoint(joints[i], i, dt, now);
      }
    }
  }

  for (int i = 0; i < JOINT_COUNT; ++i) {
    const Joint &j = joints[i];
    telemetry_snapshot[i] = j.measured_deg;
    telemetry_snapshot[3 + i] = (float)j.raw;
    telemetry_snapshot[6 + i] = j.target_deg;
    telemetry_snapshot[9 + i] = j.output;
  }
  telemetry_snapshot[12] = (float)enabled_mask;
  telemetry_snapshot[13] = (float)fault_code;
  telemetry_snapshot[14] = (float)fault_joint;
  telemetry_snapshot[15] = estop_active ? 1.0f : 0.0f;
}

void controlTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    {
      StateLock lock;
      controlStep();
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// Executor timer: publishes the latest control snapshot only.
void timerCallback(rcl_timer_t *t, int64_t last_call_time) {
  (void)last_call_time;
  if (t == NULL) return;
  {
    StateLock lock;
    memcpy(telemetry_storage, telemetry_snapshot, sizeof(telemetry_storage));
  }
  rcl_publish(&pub_telemetry, &telemetry_msg, NULL);
}

// ── micro-ROS init ──────────────────────────────────────────────────────────
void errorLoop() {
  disableAll();
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

#define RCCHECK(fn)                                                   \
  do {                                                                \
    rcl_ret_t rc = fn;                                                \
    if (rc != RCL_RET_OK) {                                           \
      Serial.printf("micro-ROS error %d at %s\n", (int)rc, #fn);     \
      errorLoop();                                                    \
    }                                                                 \
  } while (0)

void microrosInit() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "full_arm_controller", "", &support));

  targets_msg.data = {targets_storage, 0, 3};
  gains_msg.data = {gains_storage, 0, 12};
  arm_command_msg.data = {arm_command_storage, 0, 8};
  telemetry_msg.data = {telemetry_storage, 16, 16};

  RCCHECK(rclc_publisher_init_default(&pub_telemetry, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_joint_telemetry"));
  RCCHECK(rclc_timer_init_default(&timer, &support,
      RCL_MS_TO_NS(CONTROL_PERIOD_MS), timerCallback));
  RCCHECK(rclc_subscription_init_default(&sub_targets, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_joint_targets_deg"));
  RCCHECK(rclc_subscription_init_default(&sub_enable, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/arm_pid_enable"));
  RCCHECK(rclc_subscription_init_default(&sub_gains, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_pid_gains"));
  RCCHECK(rclc_subscription_init_default(&sub_arm_command, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/arm_command"));
  rmw_qos_profile_t estop_qos = rmw_qos_profile_default;
  estop_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  estop_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  RCCHECK(rclc_subscription_init(&sub_estop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/estop", &estop_qos));

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  // Safety state and targets are processed before telemetry publication.
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_estop, &estop_msg, estopCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_targets, &targets_msg, targetsCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_enable, &enable_msg, enableCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_gains, &gains_msg, gainsCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_arm_command, &arm_command_msg, armCommandCallback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ── Setup / loop ────────────────────────────────────────────────────────────
void setupOutputsSafe() {
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, LOW);
  for (const int pin : {BASE_PWM_PIN, BASE_IN1_PIN, BASE_IN2_PIN}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (Joint &j : joints) {
    pinMode(j.in1_pin, OUTPUT);
    pinMode(j.in2_pin, OUTPUT);
    digitalWrite(j.in1_pin, LOW);
    digitalWrite(j.in2_pin, LOW);
    ledcSetup(j.pwm_channel, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    ledcAttachPin(j.pwm_pin, j.pwm_channel);
    ledcWrite(j.pwm_channel, 0);
  }
}

void setup() {
  setupOutputsSafe();
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
  delay(500);

#if CALIBRATE_MODE
  Serial.println("Calibration mode; all actuators disabled.");
  Serial.println("milliseconds,shoulder_gpio34,elbow_gpio39,wrist_gpio36");
  while (true) {
    digitalWrite(MOTOR_STBY_PIN, LOW);
    Serial.printf("%lu,%d,%d,%d\n", millis(), readSettledRaw(34),
                  readSettledRaw(39), readSettledRaw(36));
    delay(200);
  }
#endif

  for (Joint &j : joints) {
    measureJoint(j);
    j.target_deg = 90.0f;  // home pose; never used until targets arrive
    resetPid(j);
  }
  state_mutex = xSemaphoreCreateMutex();

  ledcSetup(GRIPPER_SERVO_CHANNEL, SERVO_HZ, SERVO_BITS);
  ledcAttachPin(GRIPPER_SERVO_PIN, GRIPPER_SERVO_CHANNEL);
  writeGripper(0.0f);

  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int attempts = 0; WiFi.status() != WL_CONNECTED && attempts < 40; ++attempts) {
    Serial.print('.');
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\nWiFi failed, status=%d. Check SSID, password, and 2.4 GHz.\n",
                  (int)WiFi.status());
    errorLoop();
  }
  Serial.print("\nWiFi connected, ESP32 IP: ");
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASS,
                               (char *)AGENT_IP, AGENT_PORT);
  delay(2000);
  Serial.printf("Initializing micro-ROS agent at %s:%d\n", AGENT_IP, AGENT_PORT);
  microrosInit();
  // Core 1 above loop() priority; Wi-Fi runs on core 0.
  xTaskCreatePinnedToCore(controlTask, "arm_control", 4096, nullptr, 2, nullptr, 1);
  Serial.println("Ready. All joints disabled.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(5);
}
