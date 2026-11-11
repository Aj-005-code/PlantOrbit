#ifndef PLANTORBIT_HAL_H
#define PLANTORBIT_HAL_H

#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"

struct ImuSample {
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_z_dps;
  bool  valid;
};

struct EncoderCounts {
  long left_counts;
  long right_counts;
};

// Unified Hardware Abstraction Layer (HAL) API
// Keeps the PlantOrbit state machine identical across Target PCB and Wokwi Simulation
void hal_init();
const char* hal_backend_name();
bool hal_estop_active();
bool hal_bumper_triggered(bool &left_hit, bool &right_hit);
float hal_read_pot_range_mm();
void hal_set_wheel_speeds(float left_mm_s, float right_mm_s, uint32_t duration_ms);
void hal_stop_motors();
EncoderCounts hal_read_encoders();
ImuSample hal_read_imu();
void hal_set_illumination(bool enable);

#endif // PLANTORBIT_HAL_H
