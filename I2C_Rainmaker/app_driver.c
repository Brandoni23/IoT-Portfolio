
/*  Temperature Sensor demo implementation using RGB LED and timer

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include <app_reset.h>
#include "app_priv.h"
#include <math.h>
////////////////////////////////////////////////////
static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;
/////////////////////////////////////////////////////////////

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    19

static TimerHandle_t sensor_timer;

#define DEFAULT_SATURATION  100
#define DEFAULT_BRIGHTNESS  50

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10
/**
static uint16_t g_hue;
static uint16_t g_saturation = DEFAULT_SATURATION;
static uint16_t g_value = DEFAULT_BRIGHTNESS;
**/
static float g_temperature;
static float g_humidity;
uint8_t sensor_data_h1, sensor_data_h2, sensor_data_l1, sensor_data_l2;

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l, uint8_t *data_h2, uint8_t *data_l2)
{

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x27 << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("Sending failed1\n");
        return ret;
    }


    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x27 << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, ACK_VAL);
    i2c_master_read_byte(cmd, data_h2, ACK_VAL);
    i2c_master_read_byte(cmd, data_l2, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        // .clk_flags = 0,          
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static void app_sensor_update(TimerHandle_t handle)
{
    /*static float delta = 0.5;*/
    /*g_temperature += delta;*/
    /**if (g_temperature > 99) {
        delta = -0.5;
    } else if (g_temperature < 1) {
        delta = 0.5;
    }**/
    /*g_hue = (100 - g_temperature) * 2;*/
    /*ws2812_led_set_hsv(g_hue, g_saturation, g_value);*/

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(app_sensor_update, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(g_temperature));

    /**esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(app_sensor_update, ESP_RMAKER_PARAM_INTENSITY),
            esp_rmaker_float(g_humidity));**/
}

float app_get_current_temperature()
{
    return g_temperature;
}

/**float app_get_current_humidity()
{
    return g_humidity;
}**/

esp_err_t app_sensor_init(void)
{
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        return err;
    }

    /*g_temperature = DEFAULT_TEMPERATURE;*/
    sensor_timer = xTimerCreate("app_sensor_update_tm", (400 * 1000) / portTICK_PERIOD_MS,
                            pdTRUE, NULL, app_sensor_update);
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        /*g_hue = (100 - g_temperature) * 2;*/
        /*ws2812_led_set_hsv(g_hue, g_saturation, g_value);*/
        return ESP_OK;                                                                                                                                                                                                                                                                                                       
    }
    return ESP_FAIL;
}

static void i2c_test_task(void *arg)
{
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    int cnt = 0;
    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h1, &sensor_data_l1, &sensor_data_h2, &sensor_data_l2);
        g_temperature = ((sensor_data_h2 << 6) | (sensor_data_l2 >> 2))*165/(pow(2,14)-2)-40;
        g_humidity = (((sensor_data_h1 & 0x3F) << 8) | sensor_data_l1)*100/(pow(2,14)-2);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR( HoneyWell HIH6000 Series )\n", task_idx);
            printf("*******************\n");
            printf("data_h: %02x\n", sensor_data_h1);
            printf("data_l: %02x\n", sensor_data_l1);
            printf("Relative Humidity ");
            printf("[Percent]: %.02f \n", g_humidity);
            printf("data_h2: %02x\n", sensor_data_h2);
            printf("data_l2: %02x\n", sensor_data_l2);
            printf("Temperature ");
            printf("[Celsius]: %.02f \n", g_temperature);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip... speak louder cant hear you", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
    }
}

void app_driver_init()
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    app_sensor_init();
    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
}
