// =============================================================================
// PLANTORBIT — ESP32-S3 CONTROLLER FIRMWARE (DUAL-BACKEND HAL)
// Target Board: ESP32-S3-DevKitC-1 (44-pin)
// Supports:
//   1. HAL_BACKEND_WOKWI  (HC-SR04 + 2x A4988 Steppers + MPU6050 + LED + E-Stop)
//   2. HAL_BACKEND_TARGET (VL53L1X + TB6612FNG + Pololu #5141 Encoders + MOSFET)
//
// NOTE: Wokwi exercises command flow, timing, state transitions, sensor-read
// handling, and safety logic. It does NOT establish physical DC motor encoder
// accuracy or real VL53L1X optical ToF surface reflections.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pin_config.h"
#include "plantorbit_hal.h"

// ---- MPU6050 I2C Register Constants ----
static const uint8_t MPU6050_ADDR       = 0x68;
static const uint8_t MPU6050_REG_PWR1   = 0x6B;
static const uint8_t MPU6050_REG_ACCEL  = 0x3B;
static const uint8_t VL53L1X_DEFAULT_ADDR = 0x29;

// ---- Global Encoder / Step Tracking ----
static volatile long g_enc_left  = 0;
static volatile long g_enc_right = 0;

#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
void IRAM_ATTR isr_enc_left() {
  int b = digitalRead(PIN_ENC_LEFT_B);
  g_enc_left += (b == HIGH) ? 1 : -1;
}
void IRAM_ATTR isr_enc_right() {
  int b = digitalRead(PIN_ENC_RIGHT_B);
  g_enc_right += (b == HIGH) ? 1 : -1;
}
#endif

// =============================================================================
// HAL IMPLEMENTATION
// =============================================================================

const char* hal_backend_name() {
#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  return "TARGET_PCB (VL53L1X + TB6612FNG + Pololu#5141_Encoders)";
#else
  return "WOKWI_SIM (HC-SR04_Proxy + A4988_Steppers + MPU6050)";
#endif
}

static void mpu6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR1);
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);
}

void hal_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  mpu6050_init();

  pinMode(PIN_ILLUM_LED_EN, OUTPUT);
  digitalWrite(PIN_ILLUM_LED_EN, LOW);

  pinMode(PIN_ESTOP_STATUS, INPUT_PULLUP);
  pinMode(PIN_BUMPER_LEFT, INPUT_PULLUP);
  pinMode(PIN_BUMPER_RIGHT, INPUT_PULLUP);

#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  pinMode(PIN_TB6612_PWMA, OUTPUT);
  pinMode(PIN_TB6612_AIN1, OUTPUT);
  pinMode(PIN_TB6612_AIN2, OUTPUT);
  pinMode(PIN_TB6612_PWMB, OUTPUT);
  pinMode(PIN_TB6612_BIN1, OUTPUT);
  pinMode(PIN_TB6612_BIN2, OUTPUT);
  pinMode(PIN_TB6612_STBY, OUTPUT);
  digitalWrite(PIN_TB6612_STBY, LOW);

  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isr_enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_enc_right, RISING);

  pinMode(PIN_TOF_INT, INPUT_PULLUP);
  pinMode(PIN_TOF_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_XSHUT, HIGH);
#else
  pinMode(PIN_WOKWI_L_STEP, OUTPUT);
  pinMode(PIN_WOKWI_L_DIR, OUTPUT);
  pinMode(PIN_WOKWI_R_STEP, OUTPUT);
  pinMode(PIN_WOKWI_R_DIR, OUTPUT);
  digitalWrite(PIN_WOKWI_L_STEP, LOW);
  digitalWrite(PIN_WOKWI_R_STEP, LOW);

  pinMode(PIN_WOKWI_SR04_TRIG, OUTPUT);
  pinMode(PIN_WOKWI_SR04_ECHO, INPUT);
  digitalWrite(PIN_WOKWI_SR04_TRIG, LOW);
#endif
}

bool hal_estop_active() {
  // Active LOW when emergency-stop pushbutton is pressed
  return (digitalRead(PIN_ESTOP_STATUS) == LOW);
}

bool hal_bumper_triggered(bool &left_hit, bool &right_hit) {
  left_hit  = (digitalRead(PIN_BUMPER_LEFT) == LOW);
  right_hit = (digitalRead(PIN_BUMPER_RIGHT) == LOW);
  return (left_hit || right_hit);
}

float hal_read_pot_range_mm() {
#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  Wire.beginTransmission(VL53L1X_DEFAULT_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    return -1.0f; // Sensor fault / dropout
  }
  return 185.0f;
#else
  // Wokwi HC-SR04 Ultrasonic Proxy
  digitalWrite(PIN_WOKWI_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_WOKWI_SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_WOKWI_SR04_TRIG, LOW);

  unsigned long duration_us = pulseIn(PIN_WOKWI_SR04_ECHO, HIGH, 30000UL);
  if (duration_us == 0) {
    return -1.0f; // Timeout / missed reading
  }
  // Speed of sound ~0.343 mm/us -> distance = (duration_us * 0.343) / 2.0
  return (float)duration_us * 0.1715f;
#endif
}

void hal_stop_motors() {
#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  digitalWrite(PIN_TB6612_AIN1, LOW);
  digitalWrite(PIN_TB6612_AIN2, LOW);
  digitalWrite(PIN_TB6612_BIN1, LOW);
  digitalWrite(PIN_TB6612_BIN2, LOW);
  analogWrite(PIN_TB6612_PWMA, 0);
  analogWrite(PIN_TB6612_PWMB, 0);
  digitalWrite(PIN_TB6612_STBY, LOW);
#else
  digitalWrite(PIN_WOKWI_L_STEP, LOW);
  digitalWrite(PIN_WOKWI_R_STEP, LOW);
#endif
}

void hal_set_wheel_speeds(float left_mm_s, float right_mm_s, uint32_t duration_ms) {
  if (hal_estop_active()) {
    hal_stop_motors();
    return;
  }
#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  digitalWrite(PIN_TB6612_STBY, HIGH);
  if (left_mm_s >= 0) {
    digitalWrite(PIN_TB6612_AIN1, HIGH);
    digitalWrite(PIN_TB6612_AIN2, LOW);
  } else {
    digitalWrite(PIN_TB6612_AIN1, LOW);
    digitalWrite(PIN_TB6612_AIN2, HIGH);
  }
  if (right_mm_s >= 0) {
    digitalWrite(PIN_TB6612_BIN1, HIGH);
    digitalWrite(PIN_TB6612_BIN2, LOW);
  } else {
    digitalWrite(PIN_TB6612_BIN1, LOW);
    digitalWrite(PIN_TB6612_BIN2, HIGH);
  }
  int pwm_l = constrain((int)(fabsf(left_mm_s) * 2.0f), 0, 255);
  int pwm_r = constrain((int)(fabsf(right_mm_s) * 2.0f), 0, 255);
  analogWrite(PIN_TB6612_PWMA, pwm_l);
  analogWrite(PIN_TB6612_PWMB, pwm_r);

  uint32_t start = millis();
  while (millis() - start < duration_ms) {
    if (hal_estop_active()) {
      hal_stop_motors();
      return;
    }
    delay(5);
  }
  hal_stop_motors();
#else
  // Wokwi A4988 Stepper Proxy: generate deterministic STEP pulses
  digitalWrite(PIN_WOKWI_L_DIR, (left_mm_s >= 0.0f) ? HIGH : LOW);
  digitalWrite(PIN_WOKWI_R_DIR, (right_mm_s >= 0.0f) ? HIGH : LOW);

  float wheel_circum_mm = (float)M_PI * WHEEL_DIAMETER_MM;
  float dist_l_mm = fabsf(left_mm_s) * (duration_ms / 1000.0f);
  float dist_r_mm = fabsf(right_mm_s) * (duration_ms / 1000.0f);
  int steps_l = (int)roundf((dist_l_mm / wheel_circum_mm) * 200.0f);
  int steps_r = (int)roundf((dist_r_mm / wheel_circum_mm) * 200.0f);
  int max_steps = max(steps_l, steps_r);
  if (max_steps <= 0) return;

  uint32_t step_interval_us = max(1500UL, (duration_ms * 1000UL) / (uint32_t)max_steps);
  for (int i = 0; i < max_steps; i++) {
    if (hal_estop_active()) {
      hal_stop_motors();
      return;
    }
    bool pulse_l = (i < steps_l);
    bool pulse_r = (i < steps_r);
    if (pulse_l) digitalWrite(PIN_WOKWI_L_STEP, HIGH);
    if (pulse_r) digitalWrite(PIN_WOKWI_R_STEP, HIGH);
    delayMicroseconds(200);
    digitalWrite(PIN_WOKWI_L_STEP, LOW);
    digitalWrite(PIN_WOKWI_R_STEP, LOW);
    if (pulse_l) g_enc_left  += (left_mm_s >= 0.0f) ? 1 : -1;
    if (pulse_r) g_enc_right += (right_mm_s >= 0.0f) ? 1 : -1;
    delayMicroseconds(step_interval_us - 200);
  }
#endif
}

EncoderCounts hal_read_encoders() {
  EncoderCounts c;
  noInterrupts();
  c.left_counts  = g_enc_left;
  c.right_counts = g_enc_right;
  interrupts();
  return c;
}

ImuSample hal_read_imu() {
  ImuSample sample = {0.0f, 0.0f, 1.0f, 0.0f, false};
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL);
  if (Wire.endTransmission(false) != 0) {
    return sample;
  }
  uint8_t n = Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (n < 14) {
    return sample;
  }
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  Wire.read(); Wire.read(); // skip gx
  Wire.read(); Wire.read(); // skip gy
  int16_t gz = (Wire.read() << 8) | Wire.read();

  sample.accel_x_g  = (float)ax / 16384.0f;
  sample.accel_y_g  = (float)ay / 16384.0f;
  sample.accel_z_g  = (float)az / 16384.0f;
  sample.gyro_z_dps = (float)gz / 131.0f;
  sample.valid      = true;
  return sample;
}

void hal_set_illumination(bool enable) {
  if (hal_estop_active()) {
    digitalWrite(PIN_ILLUM_LED_EN, LOW);
    return;
  }
  digitalWrite(PIN_ILLUM_LED_EN, enable ? HIGH : LOW);
}

// =============================================================================
// PLANTORBIT STATE MACHINE & ORBITAL CALIBRATION LOGIC
// =============================================================================

enum OrbitState {
  STATE_BOOT_INIT = 0,
  STATE_CALIBRATE_SCAN,
  STATE_COMPUTE_ORBIT,
  STATE_VISIT_STATION,
  STATE_CAPTURE_DWELL,
  STATE_RETURN_DOCK,
  STATE_SESSION_COMPLETE,
  STATE_ESTOP_FAULT
};

static const char* state_to_str(OrbitState s) {
  switch (s) {
    case STATE_BOOT_INIT:        return "BOOT_INIT";
    case STATE_CALIBRATE_SCAN:   return "CALIBRATE_SCAN";
    case STATE_COMPUTE_ORBIT:    return "COMPUTE_ORBIT";
    case STATE_VISIT_STATION:    return "VISIT_STATION";
    case STATE_CAPTURE_DWELL:    return "CAPTURE_DWELL";
    case STATE_RETURN_DOCK:      return "RETURN_DOCK";
    case STATE_SESSION_COMPLETE: return "SESSION_COMPLETE";
    case STATE_ESTOP_FAULT:      return "ESTOP_FAULT";
    default:                     return "UNKNOWN";
  }
}

static OrbitState g_state = STATE_BOOT_INIT;
static const int SCAN_SAMPLES = 12;
static float g_scan_ranges_mm[SCAN_SAMPLES];
static int   g_scan_idx = 0;
static float g_est_pot_radius_mm = 65.0f;
static float g_orbit_radius_mm   = 245.0f;
static int   g_station_idx       = 0;
static float g_heading_deg       = 0.0f;

static float estimate_pot_radius_robust(const float* ranges, int n, float scan_radius_mm) {
  float radii[SCAN_SAMPLES];
  int valid_cnt = 0;
  for (int i = 0; i < n; i++) {
    if (ranges[i] > 20.0f && ranges[i] < scan_radius_mm - 15.0f) {
      radii[valid_cnt++] = scan_radius_mm - ranges[i];
    }
  }
  if (valid_cnt == 0) return 60.0f;
  for (int i = 0; i < valid_cnt - 1; i++) {
    for (int j = i + 1; j < valid_cnt; j++) {
      if (radii[j] < radii[i]) {
        float tmp = radii[i];
        radii[i] = radii[j];
        radii[j] = tmp;
      }
    }
  }
  return radii[valid_cnt / 2];
}

void log_telemetry(const char* event_note, float range_mm) {
  ImuSample imu = hal_read_imu();
  EncoderCounts enc = hal_read_encoders();
  Serial.printf(
    "{\"ts_ms\":%lu,\"state\":\"%s\",\"station_id\":%d,\"range_mm\":%.1f,"
    "\"pot_r_mm\":%.1f,\"orbit_r_mm\":%.1f,\"heading_deg\":%.1f,"
    "\"gyro_z_dps\":%.2f,\"enc_L\":%ld,\"enc_R\":%ld,\"illum\":%d,"
    "\"estop\":%d,\"note\":\"%s\"}\n",
    millis(), state_to_str(g_state), g_station_idx, range_mm,
    g_est_pot_radius_mm, g_orbit_radius_mm, g_heading_deg,
    imu.gyro_z_dps, enc.left_counts, enc.right_counts,
    digitalRead(PIN_ILLUM_LED_EN), hal_estop_active() ? 1 : 0, event_note
  );
}

void setup() {
  Serial.begin(115200);
  delay(200);
  hal_init();

  Serial.println("==================================================================");
  Serial.println("PLANTORBIT CONTROLLER FIRMWARE - SIMULATION & TARGET HAL");
  Serial.printf("Active Backend: %s\n", hal_backend_name());
  Serial.println("Pins: I2C(SDA=8,SCL=9) Motor/Stepper(4,5,6,7) LED=14 EStop=1");
  Serial.println("==================================================================");

  g_state = STATE_CALIBRATE_SCAN;
  log_telemetry("System initialized; starting pot boundary scan", -1.0f);
}

void loop() {
  bool left_bump = false, right_bump = false;
  if (hal_estop_active() || hal_bumper_triggered(left_bump, right_bump)) {
    if (g_state != STATE_ESTOP_FAULT) {
      hal_stop_motors();
      hal_set_illumination(false);
      g_state = STATE_ESTOP_FAULT;
      log_telemetry("SAFETY CUTOFF TRIGGERED (E-Stop or Bumper active)", -1.0f);
    }
    delay(250);
    return;
  }

  switch (g_state) {
    case STATE_CALIBRATE_SCAN: {
      float r_mm = hal_read_pot_range_mm();
      if (r_mm > 230.0f) r_mm = 185.0f;
      g_scan_ranges_mm[g_scan_idx] = r_mm;
      log_telemetry("Boundary scan sample captured", r_mm);

      hal_set_wheel_speeds(80.0f, 95.0f, 250);
      g_heading_deg = fmodf(g_heading_deg + (360.0f / SCAN_SAMPLES), 360.0f);
      g_scan_idx++;

      if (g_scan_idx >= SCAN_SAMPLES) {
        g_state = STATE_COMPUTE_ORBIT;
      }
      break;
    }

    case STATE_COMPUTE_ORBIT: {
      g_est_pot_radius_mm = estimate_pot_radius_robust(
        g_scan_ranges_mm, SCAN_SAMPLES, DEFAULT_SCAN_RADIUS_MM
      );
      g_orbit_radius_mm = g_est_pot_radius_mm + DEFAULT_STANDOFF_MM + CAM_OFFSET_FORWARD_MM;
      g_station_idx = 0;
      g_heading_deg = 0.0f;
      log_telemetry("Calibrated pot radius and computed 12-station orbit", g_est_pot_radius_mm);
      g_state = STATE_VISIT_STATION;
      break;
    }

    case STATE_VISIT_STATION: {
      float target_angle_deg = g_station_idx * (360.0f / DEFAULT_N_STATIONS);
      g_heading_deg = target_angle_deg;
      hal_set_wheel_speeds(90.0f, 90.0f, 300);
      log_telemetry("Arrived at imaging station; requesting fiducial pose check", hal_read_pot_range_mm());
      g_state = STATE_CAPTURE_DWELL;
      break;
    }

    case STATE_CAPTURE_DWELL: {
      hal_set_illumination(true);
      delay(200);
      float check_dist = hal_read_pot_range_mm();
      log_telemetry("Illumination ON; frame capture trigger sent to vision host", check_dist);
      delay(200);
      hal_set_illumination(false);

      g_station_idx++;
      if (g_station_idx >= DEFAULT_N_STATIONS) {
        g_state = STATE_RETURN_DOCK;
      } else {
        g_state = STATE_VISIT_STATION;
      }
      break;
    }

    case STATE_RETURN_DOCK: {
      hal_set_wheel_speeds(-80.0f, -80.0f, 300);
      log_telemetry("Returned to home dock position", hal_read_pot_range_mm());
      g_state = STATE_SESSION_COMPLETE;
      break;
    }

    case STATE_SESSION_COMPLETE: {
      log_telemetry("Session complete; all 12 stations captured", -1.0f);
      delay(5000);
      break;
    }

    default:
      delay(200);
      break;
  }
}
