#include "hexapod.h"

#include "pca9685_handler.h"
#include <esp_log.h>

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

void Hexapod::calibrateJoint() {
  int angle = 0;
  int step = 2;

  while (1) {
    ESP_LOGI("TAG", "Angle of rotation: %d", angle);
    legs[Leg::mid_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * angle);

    // Add delay, since it takes time for servo to rotate, usually
    // 200ms/60degree rotation under 5V power supply
    vTaskDelay(pdMS_TO_TICKS(100));

    if ((angle + step) > 45 || (angle + step) < -45) {
      step *= -1;
    }
    angle += step;
  }

}
void Hexapod::stand() {

  legs[Leg::front_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::mid_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::front_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);
  legs[Leg::mid_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);
  legs[Leg::back_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::back_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);

  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  legs[Leg::mid_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  legs[Leg::front_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);
  legs[Leg::mid_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);
  legs[Leg::back_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  legs[Leg::back_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);

  legs[Leg::front_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * 0);
  legs[Leg::mid_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * 0);
  legs[Leg::front_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * 0);
  legs[Leg::mid_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * 0);
  legs[Leg::back_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * 0);
  legs[Leg::back_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * 0);
}

void Hexapod::standUp() {
  legs[Leg::front_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::mid_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::front_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);
  legs[Leg::mid_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);
  legs[Leg::back_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  legs[Leg::back_right].moveCoxaToAngle(RIGHT_COXA_SIGN * 0);
  vTaskDelay(pdMS_TO_TICKS(500));

  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
  legs[Leg::mid_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
  legs[Leg::front_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
  legs[Leg::mid_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
  legs[Leg::back_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
  legs[Leg::back_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
  vTaskDelay(pdMS_TO_TICKS(500));

  legs[Leg::front_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
  legs[Leg::mid_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
  legs[Leg::front_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
  legs[Leg::mid_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
  legs[Leg::back_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
  legs[Leg::back_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
  vTaskDelay(pdMS_TO_TICKS(500));

  int angle_tibia = -20;
  int angle_femur = 60;
  int step_angle_tibia = 1;
  int step_angle_femur = 3;
  for(angle_tibia = -20, angle_femur = 60; angle_tibia < 0; angle_tibia += step_angle_tibia, angle_femur += step_angle_femur) {
    legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
    legs[Leg::mid_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
    legs[Leg::front_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
    legs[Leg::mid_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
    legs[Leg::back_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
    legs[Leg::back_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 60);
    vTaskDelay(pdMS_TO_TICKS(50));

    legs[Leg::front_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
    legs[Leg::mid_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
    legs[Leg::front_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
    legs[Leg::mid_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
    legs[Leg::back_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * -20);
    legs[Leg::back_right].moveTibiaToAngle(RIGHT_TIBIA_SIGN * -20);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void Hexapod::sayHi() {
  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 60);
  legs[Leg::front_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * 15);
  vTaskDelay(pdMS_TO_TICKS(500));

  for(int i = 0; i < 2; ++i) {
    legs[Leg::front_left].moveCoxaToAngle(LEFT_COXA_SIGN * 45);
    vTaskDelay(pdMS_TO_TICKS(500));
    legs[Leg::front_left].moveCoxaToAngle(LEFT_COXA_SIGN * -45);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  legs[Leg::front_left].moveCoxaToAngle(LEFT_COXA_SIGN * 0);
  vTaskDelay(pdMS_TO_TICKS(500));
  legs[Leg::front_left].moveTibiaToAngle(LEFT_TIBIA_SIGN * 0);
  vTaskDelay(pdMS_TO_TICKS(500));
  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  vTaskDelay(pdMS_TO_TICKS(500));
}

void Hexapod::leftUpDown() {

  int angle = 40;

  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * angle);
  legs[Leg::mid_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * angle);
  legs[Leg::back_left].moveFemurToAngle(LEFT_FEMUR_SIGN * angle);

  vTaskDelay(pdMS_TO_TICKS(1000));

  legs[Leg::front_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  legs[Leg::mid_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);
  legs[Leg::back_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
}

void Hexapod::rightUpDown() {

  int angle = 40;

  legs[Leg::mid_left].moveFemurToAngle(LEFT_FEMUR_SIGN * angle);
  legs[Leg::front_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * angle);
  legs[Leg::back_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * angle);

  vTaskDelay(pdMS_TO_TICKS(1000));

  legs[Leg::mid_left].moveFemurToAngle(LEFT_FEMUR_SIGN * 0);
  legs[Leg::front_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);
  legs[Leg::back_right].moveFemurToAngle(RIGHT_FEMUR_SIGN * 0);
}

void Hexapod::moveForward() {

  // Probably these are going to be calibrated
  #define TARGET_ANGLE_MOVE_UP 40.0
  #define TARGET_ANGLE_MOVE_DOWN 0.0
  #define DELAY_MOVE_LEGS_mSECONDS 600.0
  #define TARGET_ANGLE_MOVEMENT_COXA_UP 15.0
  #define TARGET_ANGLE_MOVEMENT_COXA_DOWN -10.0
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
