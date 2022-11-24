#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "pca9685.h"

#define I2C_MASTER_SDA 21    /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL 22     /*!< gpio number for I2C master clock  */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define I2C_ADDRESS     0x40    /*!< slave address for first PCA9685 */

static char tag[] = "PCA9685";
static uint8_t PCA9685_ADDR = I2C_ADDRESS;

/**
 * @brief      Write a 8 bit value to a register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param[in]  value    The value
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register(uint8_t regaddr, uint8_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief      Write two 16 bit values to the same register on an i2c device
 *
 * @param[in]  regaddr   The register address
 * @param[in]  valueOn   The value on
 * @param[in]  valueOff  The value off
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
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

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    int i2c_master_port = I2C_NUM_0;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

/**
 * @brief      Reset the PCA9685
 * 
 * @return     result of command
 */
esp_err_t resetPCA9685(void)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);   // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);    // 0x80 = "Reset"
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(50));

    return ret;
}

/**
 * @brief      Sets the frequency of PCA9685
 *
 * @param[in]  freq  The frequency
 * 
 * @return     result of command
 */
esp_err_t setFrequencyPCA9685(uint16_t freq)
{
    esp_err_t ret;

    // Send to sleep
    ret = generic_write_i2c_register(MODE1, 0x10);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set prescaler
    // calculation on page 25 of datasheet
    uint8_t prescale_val = round((CLOCK_FREQ / (4096 * freq)) - 1);
    ret = generic_write_i2c_register(PRE_SCALE, prescale_val);

    printf("prescale_val: %i\n", prescale_val);

    if (ret != ESP_OK) {
        return ret;
    }

    // reset again
    resetPCA9685();

    // Send to sleep again
    ret = generic_write_i2c_register(MODE1, 0x10);
    if (ret != ESP_OK) {
        return ret;
    }

    // wait
    vTaskDelay(pdMS_TO_TICKS(5));

    // Write 0xa0 for auto increment LED0_x after received cmd
    ret = generic_write_i2c_register(MODE1, 0xa0);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

/**
 * @brief      Sets the pwm of the pin
 *
 * @param[in]  num   The pin number
 * @param[in]  on    On time
 * @param[in]  off   Off time
 * 
 * @return     result of command
 */
esp_err_t setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    esp_err_t ret;

    uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * num;
    ret = generic_write_i2c_register_two_words(pinAddress & 0xff, on, off);

    return ret;
}

void app_main() {
    printf("Starting app_main()\n");

    i2c_example_master_init();
    resetPCA9685();
    setFrequencyPCA9685(50);  // 50 Hz
    printf("Finished setup, entering loop now\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Try different duty cycles in the fourth pin of the PCA 9685.
    printf("setPWM: 500\n");
    setPWM(3, 500, 4096-500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 1000\n");
    setPWM(3, 1000, 4096-1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 1500\n");
    setPWM(3, 1500, 4096-1500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 2000\n");
    setPWM(3, 2000, 4096-2000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 2500\n");
    setPWM(3, 2500, 4096-2500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 3000\n");
    setPWM(3, 3000, 4096-3000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 3500\n");
    setPWM(3, 3500, 4096-3500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("setPWM: 4000\n");
    setPWM(3, 4000, 4096-4000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    int i = 0;
    while(i < 10) {
        i++;
        printf("Blink all pins starting from 0\n");

        printf("for (uint8_t pin = 0; pin < 16; pin++)\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        printf("Turn LED on\n");
        setPWM(2, 2000, 4096-2000);
        printf("Move servo\n");
        setPWM(3, 2000, 4096-2000);

        vTaskDelay(pdMS_TO_TICKS(500));
        printf("Turn LED off\n");
        setPWM(2, 0, 4096);
        printf("Move servo\n");
        setPWM(3, 4000, 4096-4000);

        /*This for() sets a signal in every pin of the PCA9685*/
/*        for (uint8_t pin = 0; pin < 16; pin++)
        {
            printf("for (uint8_t pin = 0; pin < 16; pin++)\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            printf("Turn LED %d on\n", pin);
            setPWM(pin, 2000, 4096-2000);

            vTaskDelay((100));
            printf("Turn LED %d off\n", pin);
            setPWM(pin, 500, 4096-500);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
*/
    }
}
