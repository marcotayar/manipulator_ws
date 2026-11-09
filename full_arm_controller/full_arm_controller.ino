/*
  ESP32 full-arm hardware calibration
  ===================================

  Hardware map from docs/ligacao_pontes_h_esp32.pdf:
    Shoulder (Cotovelo 2): TB6612 #1 A, PWM=19, IN1=18, IN2=5, pot=34
    Elbow   (Cotovelo 1): TB6612 #1 B, PWM=17, IN1=16, IN2=15, pot=39
    Wrist   (Pulso):      TB6612 #2 B, PWM=32, IN1=14, IN2=13, pot=36
    Both TB6612 STBY pins: GPIO4
    Base 360-degree servo signal: GPIO25
    Gripper positional servo signal: GPIO26

  This sketch is intentionally calibration-only. It never enables either
  TB6612 and never attaches the base or gripper servo signal. Closed-loop
  elbow and wrist control cannot safely be enabled until their potentiometer
  ADC counts have been recorded at known physical angles.

  The existing SHOULDER_CALIBRATION belongs specifically to the shoulder
  potentiometer/divider now connected to GPIO34. Never copy it to GPIO39 or
  GPIO36: each potentiometer and divider needs its own measured calibration.

  Upload, open Serial Monitor at 115200 baud, place one joint at a measured
  angle while keeping the arm supported, and record the corresponding median.
  The output is CSV:

    milliseconds,shoulder_gpio34,elbow_gpio39,wrist_gpio36
*/

#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// TB6612 #1, channel A: shoulder (document label "Cotovelo 2").
constexpr int SHOULDER_PWM_PIN = 19;
constexpr int SHOULDER_IN1_PIN = 18;
constexpr int SHOULDER_IN2_PIN = 5;
constexpr int SHOULDER_POT_PIN = 34;

// TB6612 #1, channel B: elbow (document label "Cotovelo 1").
constexpr int ELBOW_PWM_PIN = 17;
constexpr int ELBOW_IN1_PIN = 16;
constexpr int ELBOW_IN2_PIN = 15;
constexpr int ELBOW_POT_PIN = 39;

// TB6612 #2, channel B: wrist (document label "Pulso").
constexpr int WRIST_PWM_PIN = 32;
constexpr int WRIST_IN1_PIN = 14;
constexpr int WRIST_IN2_PIN = 13;
constexpr int WRIST_POT_PIN = 36;

// The two physical STBY inputs are wired together.
constexpr int MOTOR_STBY_PIN = 4;

// Reserved for the production controller; deliberately inactive here.
constexpr int BASE_SERVO_PIN = 25;
constexpr int GRIPPER_SERVO_PIN = 26;

constexpr unsigned int ADC_SAMPLES = 31;
constexpr unsigned long REPORT_PERIOD_MS = 200;

static_assert(SHOULDER_POT_PIN == 34,
              "Shoulder calibration is valid only for the GPIO34 wiring");

int compare_ints(const void *left, const void *right) {
  const int a = *static_cast<const int *>(left);
  const int b = *static_cast<const int *>(right);
  return (a > b) - (a < b);
}

int read_adc_median(int pin) {
  int samples[ADC_SAMPLES];
  for (unsigned int i = 0; i < ADC_SAMPLES; ++i) {
    samples[i] = analogRead(pin);
  }
  qsort(samples, ADC_SAMPLES, sizeof(samples[0]), compare_ints);
  return samples[ADC_SAMPLES / 2];
}

void force_all_outputs_safe() {
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, LOW);

  const int motor_outputs[] = {
      SHOULDER_PWM_PIN, SHOULDER_IN1_PIN, SHOULDER_IN2_PIN,
      ELBOW_PWM_PIN, ELBOW_IN1_PIN, ELBOW_IN2_PIN,
      WRIST_PWM_PIN, WRIST_IN1_PIN, WRIST_IN2_PIN,
  };
  for (const int pin : motor_outputs) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // INPUT leaves both servo signal wires undriven during calibration.
  pinMode(BASE_SERVO_PIN, INPUT);
  pinMode(GRIPPER_SERVO_PIN, INPUT);
}

void setup() {
  force_all_outputs_safe();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Full-arm calibration mode; all actuators disabled.");
  Serial.println("milliseconds,shoulder_gpio34,elbow_gpio39,wrist_gpio36");
}

void loop() {
  // Reassert standby every cycle so calibration cannot energize a motor.
  digitalWrite(MOTOR_STBY_PIN, LOW);

  const int shoulder_raw = read_adc_median(SHOULDER_POT_PIN);
  const int elbow_raw = read_adc_median(ELBOW_POT_PIN);
  const int wrist_raw = read_adc_median(WRIST_POT_PIN);

  Serial.printf("%lu,%d,%d,%d\n", millis(), shoulder_raw, elbow_raw, wrist_raw);
  delay(REPORT_PERIOD_MS);
}
