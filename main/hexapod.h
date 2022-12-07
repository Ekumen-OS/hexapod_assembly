#pragma once

#include <array>
#include "pca9685_handler.h"
#include "hexapod_leg.h"

// Initial angle of the servos in the hexapod legs. Legs are disposed as shown in the following diagram.
//--Front--
//  0 ___ 5
// 1 / F \ 4
//   \___/
//   2   3
// --Back--
// 
// Initial angles for each leg.
// {coxa, femur, tibia}
// *THESE VALUES MUST BE UPDATED IF A SERVO IS CHANGED FROM THE ROBOT*
const std::array<std::array<float, 3>, 6> kInitServoAngles {{
  { 0, 35, -80},
  {-30, 60, -60},
  {-40, 20, -60},
  { 15, -30, -30},
  { 50, -30, 30},
  {-45, 0, 30}
}};

// Pines where the servos are placed for each leg.
//--Front--
//  0 ___ 5
// 1 / F \ 4
//   \___/
//   2   3
// --Back--
// {coxa, femur, tibia}
const std::array<std::array<uint32_t, 3>, 6> kServoPCA9685Pines {{
  {0, 1, 2},
  {12, 13, 14},
  {0, 1, 2},
  {12, 13, 14},
  {0, 1, 2},
  {12, 13, 14}
}};

#define SERVO_MIN_PULSEWIDTH_BITS \
  100  // Minimum Duty pulse width in microsecond, 0.5, 500uS
#define SERVO_MAX_PULSEWIDTH_BITS \
  500  // Maximum Duty pulse width in microsecond, 2.5, 2500 uS
#define SERVO_MIN_DEGREE -90  // Minimum angle.
#define SERVO_MAX_DEGREE 90   // Maximum angle.
#define SERVO_PWM_FREQ 50     // Hz. Period of 20ms.

#define I2C_MASTER_SDA 21         /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL 22         /*!< gpio number for I2C master clock  */
#define I2C_MASTER_FREQ_HZ 10000 /*!< I2C master clock frequency */

#define LEFT_COXA_SIGN 1
#define RIGHT_COXA_SIGN -1
#define LEFT_FEMUR_SIGN 1
#define RIGHT_FEMUR_SIGN -1
#define LEFT_TIBIA_SIGN 1
#define RIGHT_TIBIA_SIGN -1

// Hexapod legs are controlled with 3 PCA9685. Each one controls 2 legs as
// shown in the following diagram.
//
//--Front--
//  40 ___ 43
// 40 / F \ 43
//    \___/
//   41  41
// --Back--
#define I2C_ADDRESS_1 0x40 /*!< slave address for first PCA9685 */
#define I2C_ADDRESS_2 0x41 /*!< slave address for second PCA9685 */
#define I2C_ADDRESS_3 0x43 /*!< slave address for third PCA9685 */

class Hexapod {
 public:
  //--Front--
  //  0 ___ 5
  // 1 / F \ 4
  //   \___/
  //   2   3
  // --Back--
  enum Leg {
    front_left = 0,
    mid_left = 1,
    back_left = 2,
    back_right = 3,
    mid_right = 4,
    front_right = 5
  };

  Hexapod();

  void calibrateJoint();

  void stand();
  void standUp();
  void sayHi();

  void leftUpDown();
  void rightUpDown();

  void moveForward();
  void moveBackwards();

  void rotateRight();
  void rotateLeft();

 private:

  // Moves left front, left back and right middle femurs
  // To the target direction
  void moveAllFemursLeft(float angle);
  void moveAllFemursRight(float angle);

  // Moves left front, left back and right middle coxas
  // to the target direction. Right middle coxa inverted direction
  void moveAllCoxasLeft(float angle);
  void moveAllCoxasRight(float angle);

  std::array<HexapodLeg, 6> legs;
};
