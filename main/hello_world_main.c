/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <mavlink.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

static const char *TAG = "example";

static const int RX_BUF_SIZE = 2048;



#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
// Define the GPIO pins for the servos
#define SERVO_GPIO_PIN_1 0
#define SERVO_GPIO_PIN_2 2

// Define MCPWM configurations for each servo
mcpwm_config_t pwm_config_1 = {
    .frequency = 50,             // Set PWM frequency to 50Hz for servo control
    .cmpr_a = 0,
    .cmpr_b = 0,
    .counter_mode = MCPWM_UP_COUNTER,
    .duty_mode = MCPWM_DUTY_MODE_0,
};

mcpwm_config_t pwm_config_2 = {
    .frequency = 50,             // Set PWM frequency to 50Hz for servo control
    .cmpr_a = 0,
    .cmpr_b = 0,
    .counter_mode = MCPWM_UP_COUNTER,
    .duty_mode = MCPWM_DUTY_MODE_0,
};

void init(void)
{

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t chan = 0;
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    unsigned int temp = 0;
    int i = 0;
    // Configure MCPWM for servo 1
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO_PIN_1);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_1);

    // Configure MCPWM for servo 2
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, SERVO_GPIO_PIN_2);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_2);


    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500); // Set position for 0 degrees
                        
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1500); // Set position for 0 degrees
   
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            // if (mavlink_parse_char(chan, data[0], &msg, &status))
            // {
            //     switch (msg.msgid)
            //     {
            //     case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // #70
            //     {
            //         mavlink_rc_channels_override_t ov_chs;
            //         mavlink_msg_rc_channels_override_decode(&msg, &ov_chs);
            //         ESP_LOGI(RX_TASK_TAG, "Overr. Roll: %d ", ov_chs.chan1_raw);
            //         ESP_LOGI(RX_TASK_TAG, "Overr. Roll: %d", ov_chs.chan2_raw);
            //     }
            //     }
            // }
            for (i = 0; i < rxBytes; ++i)
            {
                temp = data[i];
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status))
                {
                    // Packet received
                    // ESP_LOGI(RX_TASK_TAG, "\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                    switch (msg.msgid)
                    {
                    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // #70
                    {
                        mavlink_rc_channels_override_t ov_chs;
                        mavlink_msg_rc_channels_override_decode(&msg, &ov_chs);
                        ESP_LOGI(RX_TASK_TAG, "chan1: %d ", ov_chs.chan1_raw);
                        // ESP_LOGI(RX_TASK_TAG, "chan2: %d", ov_chs.chan2_raw);
                        // ESP_LOGI(RX_TASK_TAG, "chan3: %d", ov_chs.chan3_raw);
                        // ESP_LOGI(RX_TASK_TAG, "chan4: %d", ov_chs.chan4_raw);
                        ESP_LOGI(RX_TASK_TAG, "chan5: %d", ov_chs.chan5_raw);
                        int speed = ov_chs.chan5_raw;
                        if (speed < 1500 && speed > 1310)
                        {
                            speed = 1350;
                        }
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed); // Set position for 0 degrees
                        
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, ov_chs.chan1_raw); // Set position for 0 degrees
                        // int value =
                                        }
                    }
                }
            }

            memset(data, 0, RX_BUF_SIZE);
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}