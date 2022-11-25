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
  legs[Leg::front_left].moveTibiaToAngle(90);
}

void Hexapod::moveBackwards() {
  legs[Leg::front_left].moveTibiaToAngle(-90);
}

void Hexapod::rotateRight() {}

void Hexapod::rotateLeft() {}