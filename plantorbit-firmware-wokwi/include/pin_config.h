#ifndef PLANTORBIT_PIN_CONFIG_H
#define PLANTORBIT_PIN_CONFIG_H

// =============================================================================
// PLANTORBIT PIN MAPPING CONFIGURATION (ESP32-S3-DevKitC-1, 44-pin)
// Matches Section 4 of Locked Architecture Plan (Plan.pdf)
// Select backend via build flag: -DHAL_BACKEND_WOKWI=1 or -DHAL_BACKEND_TARGET=1
// Default in Wokwi browser simulator: HAL_BACKEND_WOKWI
// =============================================================================

#if !defined(HAL_BACKEND_WOKWI) && !defined(HAL_BACKEND_TARGET)
#define HAL_BACKEND_WOKWI 1
#endif

// ---- Shared Pins Across Both Backends ----
#define PIN_I2C_SDA          8   // GPIO8:  I2C SDA (MPU6050 0x68 + Target VL53L1X 0x29)
#define PIN_I2C_SCL          9   // GPIO9:  I2C SCL (MPU6050 0x68 + Target VL53L1X 0x29)
#define PIN_ILLUM_LED_EN    14   // GPIO14: White LED-ring MOSFET gate (Target) / Sim LED (Wokwi)
#define PIN_ESTOP_STATUS     1   // GPIO1:  E-Stop auxiliary sense contact (Active LOW w/ pullup)
#define PIN_BUMPER_LEFT      2   // GPIO2:  Left collision bumper microswitch (Active LOW)
#define PIN_BUMPER_RIGHT    39   // GPIO39: Right collision bumper microswitch (Active LOW)

// ---- Backend-Specific Pin Assignments ----
#if defined(HAL_BACKEND_TARGET) && (HAL_BACKEND_TARGET == 1)
  // === ACTUAL TARGET HARDWARE BACKEND (KiCad Carrier PCB) ===
  // Motor Driver: Toshiba/SparkFun TB6612FNG Dual H-Bridge
  #define PIN_TB6612_PWMA    4   // GPIO4:  Left motor PWM
  #define PIN_TB6612_AIN1    5   // GPIO5:  Left motor direction input 1
  #define PIN_TB6612_AIN2    6   // GPIO6:  Left motor direction input 2
  #define PIN_TB6612_PWMB    7   // GPIO7:  Right motor PWM
  #define PIN_TB6612_BIN1   15   // GPIO15: Right motor direction input 1
  #define PIN_TB6612_BIN2   16   // GPIO16: Right motor direction input 2
  #define PIN_TB6612_STBY   17   // GPIO17: TB6612FNG standby/enable (Active HIGH)

  // Quadrature Encoders: Pololu #5141 (150.58:1 MP 6V with 12 CPR encoder, 3.3V Vcc)
  #define PIN_ENC_LEFT_A    10   // GPIO10: Left encoder channel A
  #define PIN_ENC_LEFT_B    11   // GPIO11: Left encoder channel B
  #define PIN_ENC_RIGHT_A   12   // GPIO12: Right encoder channel A
  #define PIN_ENC_RIGHT_B   13   // GPIO13: Right encoder channel B

  // Range Sensor: ST/Adafruit VL53L1X ToF (I2C 0x29)
  #define PIN_TOF_INT       18   // GPIO18: Optional VL53L1X GPIO1 interrupt (Active LOW)
  #define PIN_TOF_XSHUT     21   // GPIO21: Optional VL53L1X XSHUT shutdown control
#else
  // === WOKWI SIMULATION SUBSTITUTE BACKEND ===
  // Drive Proxy: Two A4988 Stepper Drivers + Two Bipolar Stepper Motors
  #define PIN_WOKWI_L_STEP   4   // GPIO4:  Left A4988 STEP
  #define PIN_WOKWI_L_DIR    5   // GPIO5:  Left A4988 DIR
  #define PIN_WOKWI_R_STEP   6   // GPIO6:  Right A4988 STEP
  #define PIN_WOKWI_R_DIR    7   // GPIO7:  Right A4988 DIR

  // Range Proxy: HC-SR04 Ultrasonic Sensor (exercises calibration state machine)
  // NOTE: On physical hardware HC-SR04 ECHO is 5V and requires a level shifter;
  // in Wokwi it is a simulation-only substitute for the 3.3V I2C VL53L1X.
  #define PIN_WOKWI_SR04_ECHO 18 // GPIO18: HC-SR04 ECHO input
  #define PIN_WOKWI_SR04_TRIG 21 // GPIO21: HC-SR04 TRIG output
#endif

// ---- Robot Geometric & Kinematic Constants (Reference Design) ----
#define WHEEL_DIAMETER_MM       45.0f   // 40-50 mm rubber wheel nominal diameter
#define WHEEL_TRACK_MM         190.0f   // Differential-drive wheel track width
#define POLOLU_5141_CPR         12.0f   // Motor shaft counts per revolution
#define POLOLU_5141_GEAR_RATIO 150.58f  // Exact gear ratio: (25*32*34*35*38)/(12*11*14*13*10)
#define COUNTS_PER_WHEEL_REV   (POLOLU_5141_CPR * POLOLU_5141_GEAR_RATIO) // ~1806.96 counts/rev

#define DEFAULT_SCAN_RADIUS_MM 250.0f
#define DEFAULT_STANDOFF_MM    180.0f
#define DEFAULT_N_STATIONS      12
#define CAM_OFFSET_FORWARD_MM   35.0f   // Rigid inward-facing camera optical center offset

#endif // PLANTORBIT_PIN_CONFIG_H
