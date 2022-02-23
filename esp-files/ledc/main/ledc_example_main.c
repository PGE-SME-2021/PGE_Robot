/* LEDC (LED Controller) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"



#define GPIO_1 15
#define GPIO_2 16


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (14) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 9 bits
#define LEDC_DUTY               (255) // Set duty to 80%. ((2 ** 9) - 1) * XX% =~440 
#define LEDC_FREQUENCY          (30000) // Frequency in Hertz. Set frequency at 20.3 kHz 
                                        // needs to be coherent with the resolution

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
gpio_reset_pin(GPIO_1);
gpio_reset_pin(GPIO_2);

gpio_set_direction(GPIO_1, GPIO_MODE_OUTPUT);
gpio_set_direction(GPIO_2, GPIO_MODE_OUTPUT);

    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    while(1){
        printf("Motor go forward\n");
        gpio_set_level(GPIO_1, 0);
        gpio_set_level(GPIO_2, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        printf("Motor stops \n");
        gpio_set_level(GPIO_1, 0);
        gpio_set_level(GPIO_2, 0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        printf("Motor go backward\n");
        gpio_set_level(GPIO_1, 1);
        gpio_set_level(GPIO_2, 0);
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
