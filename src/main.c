// IMPORTANT
// If program doesn't compile successfully, firstly check Port (i.e COM3)
// Then check Baud rate.
//
// If Raw NMEA Data, is needed, uncomment line 71.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

// Define which UART ports to use (GPS)
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 5  // ESP32 TX → GPS RX
#define GPS_RX_PIN 4  // ESP32 RX ← GPS TX
#define BUF_SIZE 1024

// Define which UART ports to use (LoRa)
#define LORA_UART_NUM 2
#define LORA_TX_PIN 19 // ESP32 TX → LoRa RX
#define LORA_RX_PIN 18 // ESP32 RX ← LoRa TX
#define LORA_BAUD 115200



// Helper: convert NMEA lat/lon to decimal degrees
double nmea_to_decimal(const char *nmea, char direction) {
    // Convert String to Float
    double val = atof(nmea);
    // Seperate degrees
    int degrees = (int)(val / 100);
    // Seperate minutes
    double minutes = val - (degrees * 100);
    // Convert to decimal
    double decimal = degrees + minutes / 60.0;
    // If direction is South or West, then apply negative
    if(direction == 'S' || direction == 'W') decimal = -decimal;
    return decimal;
}

// AT Commands to configure LoRa module
// AT Commands are used to configure the LoRa registers, instead of us doing it manually
// It sends the string "cmd" to the LoRa module via UART, and the module's internal firmware handles which registers to set.
void send_lora_cmd(const char *cmd) {
    uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void lora_init() {
    uart_config_t lora_config = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(LORA_UART_NUM, &lora_config);
    uart_set_pin(LORA_UART_NUM, LORA_TX_PIN, LORA_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LORA_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Basic LoRa Module Setup
    send_lora_cmd("AT\r\n"); // IDK wtf this does
    send_lora_cmd("AT+FREQ=915000000\r\n"); // Set frequency to 915 MHz
    send_lora_cmd("AT+SF=7\r\n"); // Spreading Factor 7
    send_lora_cmd("AT+BW=125\r\n"); // Bandwidth 125 kHz
    send_lora_cmd("AT+PWR=20\r\n"); // Transmit Power 20 dBm
}

void app_main(void) {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART_NUM, &uart_config);

    // Set UART pins
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Send configuration commands to GPS
    const char *set_rate = "$PMTK220,200*2C\r\n"; // 5 Hz
    uart_write_bytes(GPS_UART_NUM, set_rate, strlen(set_rate));

    const char *set_sentences = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    uart_write_bytes(GPS_UART_NUM, set_sentences, strlen(set_sentences));

    // LoRa UART setup
    lora_init();

    // Read GPS data
    uint8_t data[BUF_SIZE]; // Ray Bytes from UART
    char line[BUF_SIZE]; // One line of NMEA Data
    int line_pos = 0; // Current position

    while(1) {
        // Every 100ms read data up to length of BUF_SIZE
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        // If Data exists
        if(len > 0) {
            // Process
            for(int i = 0; i < len; i++) {
                // putchar(data[i]); // Raw data
                char c = data[i];
                if(c == '\n') {
                    line[line_pos] = '\0'; // End of sentence
                    line_pos = 0;

                    // Check if this is a GNRMC sentence
                    if(strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
                        // Tokenize the sentence
                        char *token = strtok(line, ",");
                        int field = 0;
                        char *lat_str = NULL;
                        char *lat_dir = NULL;
                        char *lon_str = NULL;
                        char *lon_dir = NULL;

                        // Read through and process data from NMEA field
                        while(token) {
                            field++;
                            if(field == 4) lat_str = token;   // latitude
                            if(field == 5) lat_dir = token;   // N/S
                            if(field == 6) lon_str = token;   // longitude
                            if(field == 7) lon_dir = token;   // E/W
                            token = strtok(NULL, ",");
                        }
                        
                        // Print Latitude and Longitude
                        if(lat_str && lat_dir && lon_str && lon_dir) {
                            // Convert to decimal degrees
                            double latitude = nmea_to_decimal(lat_str, lat_dir[0]);
                            double longitude = nmea_to_decimal(lon_str, lon_dir[0]);
                            // Print to console
                            printf("Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);

                            // Prepare payload for LoRa
                            char payload[128];

                            // snprintf writes formatted text into a character array "payload" safely,
                            // payload size is limited to sizeof(payload) to prevent overflow.
                            // LAT and LON are formatted to 6 decimal places, and latitude and longitude values are inserted.
                            snprintf(payload, sizeof(payload), "LAT:%.6f,LON:%.6f", latitude, longitude);

                            //The RA‑08H-KIT handles:
                            //Timing, preamble, CRC, and spreading
                            //Turning the RF signal into real radio waves
                            char cmd[200];
                            snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%s\r\n", strlen(payload), payload); // AT SEND command: send length and payload
                            uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
                        }
                    }

                } else if(c != '\r') {
                    line[line_pos++] = c;
                    if(line_pos >= BUF_SIZE - 1) line_pos = BUF_SIZE - 2; // prevent overflow
                }
            }
        }
    }
}
