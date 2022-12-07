/*************************************************** 
  This code has been take from here: https://github.com/brainelectronics/esp32-pca9685 and then following changes were made:
  - Adapt code from C to C++.

  You can find these changes here: https://gitlab.com/ekumenlabs/ekumembers/ekuthon2022/hexapod_assembly/-/commit/541c326250f34d0bebe1a63bf07030a959bde528

  This is a library for the PCA9685 LED PWM Driver
  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "pca9685_handler.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdint.h>

#include <iostream>
#include <string>

#include "esp_err.h"

const static char tag[] = "PCA9685Handler";

#define I2C_MASTER_SDA 21         /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL 22         /*!< gpio number for I2C master clock  */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

PCA9685Handler::PCA9685Handler(uint32_t i2c_address, uint32_t i2c_master_sda,
                               uint32_t i2c_master_scl,
                               uint32_t i2c_master_freq_hz,
                               uint32_t output_freq)
    : i2c_address_(i2c_address) {
  // I2C protocol needs to be initialized only once. So if a second instance of
  // PCA9685Handler is created, we don't want it initialize I2C again.
  const static bool started = [this, i2c_master_sda, i2c_master_scl,
                               i2c_master_freq_hz]() {
    i2cMasterInit(i2c_master_sda, i2c_master_scl, i2c_master_freq_hz);
    return true;
  }();
  resetPCA9685();
  setFrequencyPCA9685(output_freq);
}

void PCA9685Handler::i2cMasterInit(uint32_t i2c_master_sda,
                                   uint32_t i2c_master_scl,
                                   uint32_t i2c_master_freq_hz) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = i2c_master_sda;
  conf.scl_io_num = i2c_master_scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = i2c_master_freq_hz;
  conf.clk_flags = 0;

  i2c_port_t i2c_master_port = I2C_NUM_0;
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
}

esp_err_t PCA9685Handler::genericWriteI2cRegister(uint8_t regaddr,
                                                  uint8_t value) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t PCA9685Handler::genericWriteI2cRegisterTwoWords(uint8_t regaddr,
                                                          uint16_t valueOn,
                                                          uint16_t valueOff) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, valueOn & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueOn >> 8, NACK_VAL);
  i2c_master_write_byte(cmd, valueOff & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueOff >> 8, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t PCA9685Handler::resetPCA9685(void) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address_ << 1) | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);  // 0x0 = "Mode register 1"
  i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);   // 0x80 = "Reset"
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
  i2c_cmd_link_delete(cmd);

  vTaskDelay(pdMS_TO_TICKS(50));

  return ret;
}

esp_err_t PCA9685Handler::setFrequencyPCA9685(uint16_t freq) {
  esp_err_t ret;

  // Send to sleep
  ret = genericWriteI2cRegister(MODE1, 0x10);
  if (ret != ESP_OK) {
    return ret;
  }

  // Set prescaler
  // calculation on page 25 of datasheet
  uint8_t prescale_val = round((CLOCK_FREQ / (4096 * freq)) - 1);
  ret = genericWriteI2cRegister(PRE_SCALE, prescale_val);

  printf("prescale_val: %i\n", prescale_val);

  if (ret != ESP_OK) {
    return ret;
  }

  // reset again
  resetPCA9685();

  // Send to sleep again
  ret = genericWriteI2cRegister(MODE1, 0x10);
  if (ret != ESP_OK) {
    return ret;
  }

  // wait
  vTaskDelay(pdMS_TO_TICKS(5));

  // Write 0xa0 for auto increment LED0_x after received cmd
  ret = genericWriteI2cRegister(MODE1, 0xa0);
  if (ret != ESP_OK) {
    return ret;
  }

  return ret;
}

esp_err_t PCA9685Handler::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  esp_err_t ret;

  uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * num;
  ret = genericWriteI2cRegisterTwoWords(pinAddress & 0xff, on, off);

  vTaskDelay(pdMS_TO_TICKS(1));  

  return ret;
}
