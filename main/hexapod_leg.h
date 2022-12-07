#pragma once

#include "pca9685_handler.h"
#include <array>

#define SERVO_MIN_PULSEWIDTH_BITS \
  100  // Minimum Duty pulse width in microsecond, 0.5, 500uS
#define SERVO_MAX_PULSEWIDTH_BITS \
  500  // Maximum Duty pulse width in microsecond, 2.5, 2500 uS
#define SERVO_MIN_DEGREE -90  // Minimum angle
#define SERVO_MAX_DEGREE 90   // Maximum angle

struct HexapodLegConfig {
  // {coxa, femur, tibia}
  std::array<float, 3> initial_servo_angles;
  std::array<uint32_t, 3> servo_pines;

  uint32_t pca_i2c_addr;
  uint32_t i2c_master_sda_pin;
  uint32_t i2c_master_scl_pin;
  uint32_t i2c_master_freq;
  uint32_t servo_pwm_freq;
};

class HexapodLeg {
public:
  enum Joint {
    coxa = 0,
    femur = 1,
    tibia = 2
  };

  HexapodLeg(const HexapodLegConfig& config);

  void moveFemurToAngle(const int angle);
  void moveCoxaToAngle(const int angle);
  void moveTibiaToAngle(const int angle);
  void moveLegToAngle(const int angle, const int joint);

private:
  uint32_t angleToPWMDuty(int angle);

  PCA9685Handler pca_handler;
  
  std::array<float, 3> servos_init_angles_;
  std::array<uint32_t, 3> joints_pines_;

};
