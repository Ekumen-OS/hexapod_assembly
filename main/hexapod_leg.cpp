#include "hexapod_leg.h"


uint32_t HexapodLeg::angleToPWMDuty(int angle) {
  return (angle - SERVO_MIN_DEGREE) *
             (SERVO_MAX_PULSEWIDTH_BITS - SERVO_MIN_PULSEWIDTH_BITS) /
             (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
         SERVO_MIN_PULSEWIDTH_BITS;
}


HexapodLeg::HexapodLeg(const HexapodLegConfig& config): pca_handler(config.pca_i2c_addr,
  config.i2c_master_sda_pin,
  config.i2c_master_scl_pin,
  config.i2c_master_freq,
  config.servo_pwm_freq), 
  servos_init_angles_(config.initial_servo_angles),
  joints_pines_(config.servo_pines) {}


void HexapodLeg::moveFemurToAngle(const int angle) {
  moveLegToAngle(angle, Joint::femur);
}

void HexapodLeg::moveCoxaToAngle(const int angle) {
  moveLegToAngle(angle, Joint::coxa);
}

void HexapodLeg::moveTibiaToAngle(const int angle) {
  moveLegToAngle(angle, Joint::tibia);
}

void HexapodLeg::moveLegToAngle(const int angle, const int joint){
  pca_handler.setPWM(joints_pines_[joint], 0, angleToPWMDuty(angle + servos_init_angles_[joint]));
}