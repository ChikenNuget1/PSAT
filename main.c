// #include "gps.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "string.h"


#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 5  // ESP32 TX → GPS RX
#define GPS_RX_PIN 4  // ESP32 RX ← GPS TX
#define BUF_SIZE 1024