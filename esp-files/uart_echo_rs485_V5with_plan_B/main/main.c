/*!
   \file main.c
   \brief Driver motor
   \author Aurélien BENOIT
   \date 06/01/2022
   \note Some pins on target chip cannot be assigned for UART communication.
        Please refer to documentation for selected board and target to configure pins using Kconfig.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
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

/*!
   \def TAG
   \brief String
*/
#define TAG "RS485_ECHO_APP"

/*!
   \def ECHO_TEST_TXD (CONFIG_ECHO_UART_TXD)
   \brief Pin number TXD
*/
#define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
/*!
   \def ECHO_TEST_RXD (CONFIG_ECHO_UART_RXD)
   \brief Pin number RXD
*/
#define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)

/*!
   \def ECHO_TEST_RTS  (CONFIG_ECHO_UART_RTS)
   \brief RTS for RS485 Half-Duplex Mode manages DE/~RE
*/
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

/*!
   \def ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)
   \brief CTS is not used in RS485 Half-Duplex Mode
*/
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)
/*!
   \def BUF_SIZE
   \brief Buffer of read data
*/
#define BUF_SIZE        (127)
/*!
   \def BAUD_RATE
   \brief Speed communication (bit/s)
*/
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)
/*!
   \def PACKET_READ_TICS
   \brief Read packet timeout
*/
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
/*!
   \def ECHO_TASK_STACK_SIZE
   \brief Size of the task
*/
#define ECHO_TASK_STACK_SIZE    (2048)
/*!
   \def ECHO_TASK_PRIO
   \brief Priority of the task
*/
#define ECHO_TASK_PRIO  (10)
/*!
   \def ECHO_UART_PORT
   \brief Number port
*/
#define ECHO_UART_PORT   (CONFIG_ECHO_UART_PORT_NUM)

/*!
   \def ECHO_READ_TOUT
   \brief Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin,
          3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
*/
#define ECHO_READ_TOUT   (3)
/*!
   \brief Led on pin
*/
#define LED_ON (26)
/*!
   \brief Led green pin
*/
#define LED_BLINK_GREEN (34)
/*!
   \brief Led red pin
*/
#define LED_BLINK_RED (35)
/*!
   \brief Right Forward pin
*/
#define DROITE_AV (36)
/*!
   \brief Right Back pin
*/
#define DROITE_AR (37)
/*!
   \brief Left Forward pin
*/
#define GAUCHE_AV (38)
/*!
   \brief Left Back pin
*/
#define GAUCHE_AR (39)
/*!
   \brief Led Right Forward pin
*/
#define LED_D_AV (40)
/*!
   \brief Led Right Back pin
*/
#define LED_D_AR (41)
/*!
   \brief Led Left Forward pin
*/
#define LED_G_AV (42)
/*!
   \brief Led Left Back pin
*/
#define LED_G_AR (45)

/*!
   \brief Timer number
*/
#define LEDC_TIMER              LEDC_TIMER_0
/*!
   \brief Speed mode
*/
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
/*!
   \brief Motor right speed pin
*/
#define VITESSE_DROITE          (1) // Define the output GPIO
/*!
   \brief Motor left speed pin
*/
#define VITESSE_GAUCHE          (2) // Define the output GPIO
/*!
   \brief Channel for Motor right speed
*/
#define M_CHANNEL_0             LEDC_CHANNEL_0
/*!
   \brief Channel for Motor left speed
*/
#define M_CHANNEL_1            LEDC_CHANNEL_1
/*!
   \brief Size of ledc_channel
*/
#define CH_NUM       (2)
/*!
   \brief Set duty resolution to 8 bits
*/
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
/*!
   \brief Set duty to %. ((2 ** 8) - 1) * %
*/
#define LEDC_DUTY               (255)
/*!
   \brief Frequency in Hertz
*/
#define LEDC_FREQUENCY          (30000)


uart_config_t uart_config = {
    .baud_rate = BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_APB,
};
ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .timer_num        = LEDC_TIMER,
    .duty_resolution  = LEDC_DUTY_RES,
    .freq_hz          = LEDC_FREQUENCY,
    .clk_cfg          = LEDC_AUTO_CLK
};
ledc_channel_config_t ledc_channel[CH_NUM] = {
      { //Vitesse Droite Arret
        .speed_mode     = LEDC_MODE,
        .channel        = M_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = VITESSE_DROITE,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
        },
        { //Vitesse gauche Arret
          .speed_mode     = LEDC_MODE,
          .channel        = M_CHANNEL_1,
          .timer_sel      = LEDC_TIMER,
          .intr_type      = LEDC_INTR_DISABLE,
          .gpio_num       = VITESSE_GAUCHE,
          .duty           = 0, // Set duty to 0%
          .hpoint         = 0
        },
    };
const int uart_num = ECHO_UART_PORT;



/*!
   \brief Initialize the uart communication,
          Initialize the PWM signals "ledc_channel" and the timer "ledc_timer
   \return void
*/
void echo_init(){
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
  for (int ch = 0; ch < CH_NUM; ch++) ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
  esp_log_level_set(TAG, ESP_LOG_INFO);                                                                 // Set UART log level
  ESP_LOGI(TAG, "Start RS485 application test and configure UART.");
  // Install UART driver (we don't need an event queue here)
  // In this example we don't even use a buffer for sending data.
  ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));                                            // Configure UART parameters
  ESP_LOGI(TAG, "UART set pins, mode and install driver.");
  ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));   // Set UART pins as per KConfig settings
  ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));                                 // Set RS485 half duplex mode
  ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));                                        // Set read timeout of UART TOUT feature

}

/*!
   \brief Initialize GPIO ports
   \return void
*/
void gpio_init(){
  gpio_reset_pin(LED_BLINK_GREEN);
  gpio_reset_pin(LED_BLINK_RED);
  gpio_reset_pin(LED_D_AV);
  gpio_reset_pin(LED_D_AR);
  gpio_reset_pin(LED_G_AV);
  gpio_reset_pin(LED_G_AR);
  gpio_reset_pin(LED_ON);
  gpio_reset_pin(DROITE_AV);
  gpio_reset_pin(DROITE_AR);
  gpio_reset_pin(GAUCHE_AV);
  gpio_reset_pin(GAUCHE_AR);
  gpio_set_direction(LED_BLINK_GREEN, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BLINK_RED, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_D_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_D_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_G_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_G_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(DROITE_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(DROITE_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GAUCHE_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(GAUCHE_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_ON, GPIO_MODE_OUTPUT);
}


/*!
   \brief Move the robot forward with the given speed
   \param tab[2]
   \return void
*/
void forward(float tab[2]){
  printf("Le robot avance \r\n");
  gpio_set_level(LED_D_AV, 1);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 1);
  gpio_set_level(LED_G_AR, 0);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);
  gpio_set_level(DROITE_AV, 1);
  gpio_set_level(DROITE_AR, 0);
  gpio_set_level(GAUCHE_AV, 1);
  gpio_set_level(GAUCHE_AR, 0);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 2; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");
}

/*!
   \brief Back up the robot with the given speed
   \param tab[2]
   \return void
*/
void back(float tab[2]){
  printf("Le robot recule \r\n");
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 1);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 1);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);
  gpio_set_level(DROITE_AV, 0);
  gpio_set_level(DROITE_AR, 1);
  gpio_set_level(GAUCHE_AV, 0);
  gpio_set_level(GAUCHE_AR, 1);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 2; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

/*!
   \brief Turn the robot to the right with the given speed
   \param tab[2]
   \return void
*/
void right(float tab[2]){
  printf("Le robot tourne à droite \r\n");
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 1);
  gpio_set_level(LED_G_AV, 1);
  gpio_set_level(LED_G_AR, 0);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);
  gpio_set_level(DROITE_AV, 0);
  gpio_set_level(DROITE_AR, 1);
  gpio_set_level(GAUCHE_AV, 1);
  gpio_set_level(GAUCHE_AR, 0);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 2; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

/*!
   \brief Turn the robot to the left with the given speed
   \param tab[2]
   \return void
*/
void left(float tab[2]){
  printf("Le robot tourne gauche \r\n");
  gpio_set_level(LED_D_AV, 1);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 1);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);
  gpio_set_level(DROITE_AV, 1);
  gpio_set_level(DROITE_AR, 0);
  gpio_set_level(GAUCHE_AV, 0);
  gpio_set_level(GAUCHE_AR, 1);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 2; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

/*!
   \brief Stop the robot
   \param[in] tab[2]
   \return void
*/
void stop(float tab[2]){
  printf("Le robot s'arrete \r\n");
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 0);
  gpio_set_level(LED_BLINK_GREEN, 0);
  gpio_set_level(LED_BLINK_RED, 1);
  gpio_set_level(DROITE_AV, 0);
  gpio_set_level(DROITE_AR, 0);
  gpio_set_level(GAUCHE_AV, 0);
  gpio_set_level(GAUCHE_AR, 0);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 2; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");


}

/*!
   \brief Managing the robot's movements
   \param[in] l
   \param[in] data
   \param[in] tab[4]
   \return void
*/
void commandMotor(int l, uint8_t * data, float tab[2]){
  uint8_t tabCommand[1];

    tabCommand[0] = data[2] ;
    printf("%d", tabCommand[0]);

  printf("\n");

  if(tabCommand[0] == 1) forward(tab);//avancer
  else if(tabCommand[0] == 2) back(tab);//reculer
  else if(tabCommand[0] == 3) right(tab); // droite
  else if(tabCommand[0] == 4) left(tab);// gauche
  else if(tabCommand[0] == 5) stop(tab);// stop
  else ESP_LOGE(TAG, "Error Data command - command should be < 6");
}

/*!
   \brief Receive motor's speed
   \param[in] arg
   \return void
*/
void echo_task(void *arg){
    printf("portTICK_RATE_MS : %d \n", portTICK_RATE_MS);
    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    uint8_t * data = (uint8_t*) malloc(BUF_SIZE);                                                            // Allocate buffers for UART
    float dataSpeed[2];
    int v = 0;
    while(1) {
        int len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len)); //vérifier le nombre d'octets disponibles dans le tampon Rx FIFO
        len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
        if (len > 0) { // if data
          ESP_LOGI(TAG, "Received %u bytes:", len);

            printf("[  ");
            for (v = 0; v < 2; v++) {
                dataSpeed[v] = data[v];
                printf("%d ", (uint8_t)dataSpeed[v]);
            }
            printf("] \n");

            commandMotor(len, data, dataSpeed);

        } else {
            printf(".");
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/*!
   \brief Main function
   \return void
*/
void app_main(void){
  echo_init();
  gpio_init();
  xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
}
