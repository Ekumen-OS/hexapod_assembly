#include "hexapod.h"

#include "pca9685_handler.h"

Hexapod::Hexapod()
    : legs{{
      {{kInitServoAngles[Leg::front_left], kServoPCA9685Pines[Leg::front_left], I2C_ADDRESS_1, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
      {{kInitServoAngles[Leg::mid_left], kServoPCA9685Pines[Leg::mid_left], I2C_ADDRESS_1, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
      {{kInitServoAngles[Leg::back_left], kServoPCA9685Pines[Leg::back_left], I2C_ADDRESS_2, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
      {{kInitServoAngles[Leg::back_right], kServoPCA9685Pines[Leg::back_right], I2C_ADDRESS_2, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
      {{kInitServoAngles[Leg::mid_right], kServoPCA9685Pines[Leg::mid_right], I2C_ADDRESS_3, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
      {{kInitServoAngles[Leg::front_right], kServoPCA9685Pines[Leg::front_right], I2C_ADDRESS_3, I2C_MASTER_SDA, I2C_MASTER_SCL,
                     I2C_MASTER_FREQ_HZ, SERVO_PWM_FREQ}},
    }}
 {}

void Hexapod::moveForward() {

  // Probably these are going to be calibrated
  #define TARGET_ANGLE_MOVE_UP 90.0
  #define TARGET_ANGLE_MOVE_DOWN 20.0
  #define DELAY_MOVE_LEGS_mSECONDS 600.0
  #define TARGET_ANGLE_MOVEMENT_COXA_UP 75.0
  #define TARGET_ANGLE_MOVEMENT_COXA_DOWN -30.0
  #define TARGET_ANGLE_STABLE_FEMUR 0

  // This routine should start with the hexapod in its
  // "RESTING" position

  // Complete sequence to move with the left side
  moveAllFemursLeft(TARGET_ANGLE_MOVE_UP);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));

  moveAllCoxasLeft(TARGET_ANGLE_MOVEMENT_COXA_UP);
  moveAllCoxasRight(TARGET_ANGLE_MOVEMENT_COXA_DOWN);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));

  moveAllFemursLeft(TARGET_ANGLE_MOVE_DOWN);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));

  // Complete sequence to move with the right side
  moveAllFemursRight(TARGET_ANGLE_MOVE_UP);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));

  moveAllCoxasRight(TARGET_ANGLE_MOVEMENT_COXA_UP);
  moveAllCoxasLeft(TARGET_ANGLE_MOVEMENT_COXA_DOWN);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));

  moveAllFemursRight(TARGET_ANGLE_MOVE_DOWN);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MOVE_LEGS_mSECONDS));
}

void Hexapod::moveAllFemursLeft(float angle){
  // Femurs, Coxas, and Tibias from the right side
  // Require to multiply by -1 to go in the expected direction

  legs[Leg::front_left].moveFemurToAngle(angle);
  legs[Leg::back_left].moveFemurToAngle(angle);
  legs[Leg::mid_right].moveFemurToAngle( (-1.0 * angle) );
}
void Hexapod::moveAllFemursRight(float angle){
  legs[Leg::front_right].moveFemurToAngle((-1.0 * angle));
  legs[Leg::back_right].moveFemurToAngle((-1.0 * angle));
  legs[Leg::mid_left].moveFemurToAngle(angle);
}

void Hexapod::moveAllCoxasLeft(float angle){
  legs[Leg::front_left].moveCoxaToAngle(angle);
  legs[Leg::back_left].moveCoxaToAngle(angle);
  legs[Leg::mid_right].moveCoxaToAngle( -1* angle);
}

void Hexapod::moveAllCoxasRight(float angle){
  legs[Leg::front_right].moveCoxaToAngle((-1.0 * angle));
  legs[Leg::back_right].moveCoxaToAngle((-1.0 * angle));
  legs[Leg::mid_left].moveCoxaToAngle(angle);
}

void Hexapod::moveBackwards() {
  legs[Leg::front_left].moveTibiaToAngle(-90);
}

void Hexapod::rotateRight() {}

void Hexapod::rotateLeft() {}
