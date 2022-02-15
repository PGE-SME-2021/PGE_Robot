#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
//********************************************************************************************************************
//********************************************************************************************************************
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
//********************************************************************************************************************
//********************************************************************************************************************

void gpio_init();
