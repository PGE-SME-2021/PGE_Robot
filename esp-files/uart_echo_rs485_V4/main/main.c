
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


// **********************************************************************************************************************************
// **********************************************************************************************************************************

#define TAG "RS485_ECHO_APP"
// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
#define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)
// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)
// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)
#define BUF_SIZE        (127)
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)
// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
// **********************************************************************************************************************************
// **********************************************************************************************************************************
#define LED_ON (26)
#define LED_BLINK_GREEN (34)
#define LED_BLINK_RED (35)
#define LED_D_AV (40)
#define LED_D_AR (41)
#define LED_G_AV (42)
#define LED_G_AR (45)
//****************************************************************************************************************
//****************************************************************************************************************
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_DROITE_AVANT          (1) // Define the output GPIO
#define LEDC_OUTPUT_IO_DROITE_ARRIERE          (2) // Define the output GPIO
#define LEDC_OUTPUT_IO_GAUCHE_AVANT          (3) // Define the output GPIO
#define LEDC_OUTPUT_IO_GAUCHE_ARRIERE          (4) // Define the output GPIO
#define M_CHANNEL_0             LEDC_CHANNEL_0
#define M_CHANNEL_1            LEDC_CHANNEL_1
#define M_CHANNEL_2            LEDC_CHANNEL_2
#define M_CHANNEL_3            LEDC_CHANNEL_3
#define CH_NUM       (4)
#define LEDC_DUTY_RES           LEDC_TIMER_9_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (440) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (20300) // Frequency in Hertz. Set frequency at 5 kHz



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
    .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
    .clk_cfg          = LEDC_AUTO_CLK
};
ledc_channel_config_t ledc_channel[CH_NUM] = {
      { //Droite Avant Arret
        .speed_mode     = LEDC_MODE,
        .channel        = M_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_DROITE_AVANT,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
        },
        { //Droite Arriere Arret
          .speed_mode     = LEDC_MODE,
          .channel        = M_CHANNEL_1,
          .timer_sel      = LEDC_TIMER,
          .intr_type      = LEDC_INTR_DISABLE,
          .gpio_num       = LEDC_OUTPUT_IO_DROITE_ARRIERE,
          .duty           = 0, // Set duty to 0%
          .hpoint         = 0
        },
        { //Gauche Avant Arret
          .speed_mode     = LEDC_MODE,
          .channel        = M_CHANNEL_2,
          .timer_sel      = LEDC_TIMER,
          .intr_type      = LEDC_INTR_DISABLE,
          .gpio_num       = LEDC_OUTPUT_IO_GAUCHE_AVANT,
          .duty           = 0, // Set duty to 0%
          .hpoint         = 0
        },
        { //Gauche Arriere Arret
          .speed_mode     = LEDC_MODE,
          .channel        = M_CHANNEL_3,
          .timer_sel      = LEDC_TIMER,
          .intr_type      = LEDC_INTR_DISABLE,
          .gpio_num       = LEDC_OUTPUT_IO_GAUCHE_ARRIERE,
          .duty           = 0, // Set duty to 0%
          .hpoint         = 0
        },
    };
const int uart_num = ECHO_UART_PORT;

static void echo_send(const int port, const char* str, uint8_t length){
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

static void echo_init(){
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

void gpio_init(){
  gpio_reset_pin(LED_BLINK_GREEN);
  gpio_reset_pin(LED_BLINK_RED);
  gpio_reset_pin(LED_D_AV);
  gpio_reset_pin(LED_D_AR);
  gpio_reset_pin(LED_G_AV);
  gpio_reset_pin(LED_G_AR);
  gpio_reset_pin(LED_ON);
  gpio_set_direction(LED_BLINK_GREEN, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BLINK_RED, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_D_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_D_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_G_AV, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_G_AR, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_ON, GPIO_MODE_OUTPUT);

}

void forward(float tab[4]){
  printf("Le robot avance \r\n");
  gpio_set_level(LED_D_AV, 1);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 1);
  gpio_set_level(LED_G_AR, 0);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_2, (uint8_t)tab[2]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_2)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_3, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_3)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 4; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");


}

void back(float tab[4]){
  printf("Le robot recule \r\n");
  //echo_send(uart_num,"Le robot recule \r\n", 21);
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 1);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 1);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_2, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_2)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_3,(uint8_t)tab[3]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_3)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 4; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

void right(float tab[4]){
  printf("Le robot tourne à droite \r\n");
  //echo_send(uart_num,"Le robot tourne à droite \r\n", 30);
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 1);
  gpio_set_level(LED_G_AV, 1);
  gpio_set_level(LED_G_AR, 0);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,(uint8_t)tab[1]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_2, (uint8_t)tab[2]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_2)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_3,0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_3)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 4; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

void left(float tab[4]){
  printf("Le robot tourne gauche \r\n");
  //echo_send(uart_num,"Le robot tourne gauche \r\n", 27);
  gpio_set_level(LED_D_AV, 1);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 1);
  gpio_set_level(LED_BLINK_GREEN, 1);
  gpio_set_level(LED_BLINK_RED, 0);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, (uint8_t)tab[0]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1,0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_2, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_2)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_3,(uint8_t)tab[3]));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_3)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 4; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");

}

void stop(float tab[4]){
  printf("Le robot s'arrete \r\n");
  //echo_send(uart_num,"Le robot s'arrete \r\n", 23);
  gpio_set_level(LED_D_AV, 0);
  gpio_set_level(LED_D_AR, 0);
  gpio_set_level(LED_G_AV, 0);
  gpio_set_level(LED_G_AR, 0);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_0)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_1, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_1)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_2, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_2)); // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, M_CHANNEL_3, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, M_CHANNEL_3)); // Update duty to apply the new value
  printf(" duty_cycle [ ");
  for (int v = 0; v < 4; v++) printf("%d ", (uint8_t)tab[v]);
  printf("] \n");
  gpio_set_level(LED_BLINK_GREEN, 0);
  gpio_set_level(LED_BLINK_RED, 1);

}

void commandMotor(int l, uint8_t * data, float tab[4]){
  uint8_t tabCommand[1];
  for (int v = 0; v < l; v++) {
    tabCommand[v] = data[v] ;
    printf("%d", tabCommand[v]);
  }
  printf("\n");
  if(tabCommand[0] == 1) forward(tab);//avancer
  else if(tabCommand[0] == 2) back(tab);//reculer
  else if(tabCommand[0] == 3) right(tab); // droite
  else if(tabCommand[0] == 4) left(tab);// gauche
  else if(tabCommand[0] == 5) stop(tab);// stop
  else ESP_LOGE(TAG, "Error Data command - command should be < 6");
}

static void echo_task(void *arg){
    printf("portTICK_RATE_MS : %d \n", portTICK_RATE_MS);
    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    uint8_t * data = (uint8_t*) malloc(BUF_SIZE);                                                            // Allocate buffers for UART
    float dataSpeed[4];
    int v = 0;
    while(1) {
        int len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len)); //vérifier le nombre d'octets disponibles dans le tampon Rx FIFO
        len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
        if (len > 0) { // if data
          ESP_LOGI(TAG, "Received %u bytes:", len);
          if (len == 4 ){
            printf("[  ");
            for (v = 0; v < len; v++) {
                dataSpeed[v] = 511 * ((float)data[v] /100 );
                printf("%d ", (uint8_t)dataSpeed[v]);
            }
            printf("] \n");
          } else { //len = 1 // command motor

              commandMotor(len, data, dataSpeed);
          }
        } else {
            echo_send(uart_num, ".", 1);
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void){
  echo_init();
  gpio_init();
  xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);


}
