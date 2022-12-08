/*************************************************** 
  This code has been take from here: https://github.com/brainelectronics/esp32-pca9685 and then following changes were made:
  - Adapt code from C to C++.

  You can find these changes here: https://gitlab.com/ekumenlabs/ekumembers/ekuthon2022/hexapod_assembly/-/commit/541c326250f34d0bebe1a63bf07030a959bde528

  This is a library for the PCA9685 LED PWM Driver
  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include "esp_err.h"

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

// PCA9685 registers. Got from datasheet.
#define MODE1 0x00       /*!< Mode register 1 */
#define MODE2 0x01       /*!< Mode register 2 */
#define SUBADR1 0x02     /*!< I2C-bus subaddress 1 */
#define SUBADR2 0x03     /*!< I2C-bus subaddress 2 */
#define SUBADR3 0x04     /*!< I2C-bus subaddress 3 */
#define ALLCALLADR 0x05  /*!< LED All Call I2C-bus address */
#define LED0 0x6         /*!< LED0 start register */
#define LED0_ON_L 0x6    /*!< LED0 output and brightness control byte 0 */
#define LED0_ON_H 0x7    /*!< LED0 output and brightness control byte 1 */
#define LED0_OFF_L 0x8   /*!< LED0 output and brightness control byte 2 */
#define LED0_OFF_H 0x9   /*!< LED0 output and brightness control byte 3 */
#define LED_MULTIPLYER 4 /*!< For the other 15 channels */
#define ALLLED_ON_L \
  0xFA /*!< load all the LEDn_ON registers, byte 0 (turn 0-7 channels on) */
#define ALLLED_ON_H \
  0xFB /*!< load all the LEDn_ON registers, byte 1 (turn 8-15 channels on) */
#define ALLLED_OFF_L \
  0xFC /*!< load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off) */
#define ALLLED_OFF_H                                                         \
  0xFD /*!< load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off) \
        */
#define PRE_SCALE 0xFE        /*!< prescaler for output frequency */
#define CLOCK_FREQ 25000000.0 /*!< 25MHz default osc clock */

class PCA9685Handler {
 public:
  PCA9685Handler(uint32_t i2c_address, uint32_t i2c_master_sda,
                 uint32_t i2c_master_scl, uint32_t i2c_master_freq_hz,
                 uint32_t output_freq);

  /**
   * @brief      Sets the pwm of the pin
   *
   * @param[in]  num   The pin number
   * @param[in]  on    On time
   * @param[in]  off   Off time
   *
   * @return     result of command
   */
  esp_err_t setPWM(uint8_t num, uint16_t on, uint16_t off);

 private:
  /**
   * @brief      Write a 8 bit value to a register on an i2c device
   *
   * @param[in]  regaddr  The register address
   * @param[in]  value    The value
   *
   * @return     result of command
   */
  esp_err_t genericWriteI2cRegister(uint8_t regaddr, uint8_t value);

  /**
   * @brief      Write two 16 bit values to the same register on an i2c device
   *
   * @param[in]  regaddr   The register address
   * @param[in]  valueOn   The value on
   * @param[in]  valueOff  The value off
   *
   * @return     result of command
   */
  esp_err_t genericWriteI2cRegisterTwoWords(uint8_t regaddr, uint16_t valueOn,
                                            uint16_t valueOff);

  /**
   * @brief i2c master initialization
   */
  void i2cMasterInit(uint32_t i2c_master_sda, uint32_t i2c_master_scl,
                     uint32_t i2c_master_freq_hz);

  /**
   * @brief      Reset the PCA9685
   *
   * @return     result of command
   */
  esp_err_t resetPCA9685(void);

  /**
   * @brief      Sets the frequency of PCA9685
   *
   * @param[in]  freq  The frequency
   *
   * @return     result of command
   */
  esp_err_t setFrequencyPCA9685(uint16_t freq);

  uint32_t i2c_address_;
};
