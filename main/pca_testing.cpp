#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "pca9685_handler.h"

#define SERVO_MIN_PULSEWIDTH_BITS \
  100  // Minimum Duty pulse width in microsecond, 0.5, 500uS
#define SERVO_MAX_PULSEWIDTH_BITS \
  500  // Maximum Duty pulse width in microsecond, 2.5, 2500 uS
#define SERVO_MIN_DEGREE -90  // Minimum angle
#define SERVO_MAX_DEGREE 90   // Maximum angle

#define I2C_MASTER_SDA 21         /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL 22         /*!< gpio number for I2C master clock  */
#define I2C_MASTER_FREQ_HZ 10000 /*!< I2C master clock frequency */

#define I2C_ADDRESS_1 0x40 /*!< slave address for first PCA9685 */
#define I2C_ADDRESS_2 0x41 /*!< slave address for first PCA9685 */
#define I2C_ADDRESS_3 0x43 /*!< slave address for first PCA9685 */

static inline uint32_t angle_to_compare(int angle) {
  return (angle - SERVO_MIN_DEGREE) *
             (SERVO_MAX_PULSEWIDTH_BITS - SERVO_MIN_PULSEWIDTH_BITS) /
             (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
         SERVO_MIN_PULSEWIDTH_BITS;
}

extern "C" {
void app_main();
}

void app_main() {
  printf("Starting app_main()\n");
  PCA9685Handler pca1(I2C_ADDRESS_1, I2C_MASTER_SDA, I2C_MASTER_SCL,
                      I2C_MASTER_FREQ_HZ, 50);
  PCA9685Handler pca2(I2C_ADDRESS_2, I2C_MASTER_SDA, I2C_MASTER_SCL,
                      I2C_MASTER_FREQ_HZ, 50);
  PCA9685Handler pca3(I2C_ADDRESS_2, I2C_MASTER_SDA, I2C_MASTER_SCL,
                      I2C_MASTER_FREQ_HZ, 50);

  int angle = 0;
  int step = 2;

  while (1) {
    ESP_LOGI("TAG", "Angle of rotation: %d", angle);
    ESP_LOGI("TAG", "PWM value: %d", angle_to_compare(angle));
    pca1.setPWM(3, 0, angle_to_compare(angle));

    // Add delay, since it takes time for servo to rotate, usually
    // 200ms/60degree rotation under 5V power supply
    vTaskDelay(pdMS_TO_TICKS(100));

    if ((angle + step) > 90 || (angle + step) < -90) {
      step *= -1;
    }
    angle += step;
  }
}
