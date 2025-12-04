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

// Define which UART ports to use
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 5  // ESP32 TX → GPS RX
#define GPS_RX_PIN 4  // ESP32 RX ← GPS TX
#define BUF_SIZE 1024

// Define LoRa UART
#define LORA_UART_NUM UART_NUM_0
#define LORA_TX_PIN 17 // ESP32 TX -> LORA RX
#define LORA_RX_PIN 16 // ESP32 RX -> LORA TX

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

void app_main(void) {
    // Configure ESP32 UART parameters
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

    // Configure LoRa UART parameters
    uart_config_t lora_uart_config = {
        .baud_rate = 9600,  // adjust to your LoRa module
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(LORA_UART_NUM, &lora_uart_config);
    uart_set_pin(LORA_UART_NUM, LORA_TX_PIN, LORA_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LORA_UART_NUM, 2048, 0, 0, NULL, 0);

    // Send configuration commands to GPS
    const char *set_rate = "$PMTK220,200*2C\r\n"; // 5 Hz
    uart_write_bytes(GPS_UART_NUM, set_rate, strlen(set_rate));

    const char *set_sentences = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    uart_write_bytes(GPS_UART_NUM, set_sentences, strlen(set_sentences));

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
                            double latitude = nmea_to_decimal(lat_str, lat_dir[0]);
                            double longitude = nmea_to_decimal(lon_str, lon_dir[0]);
                            printf("Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);
                            
                            // Transmit Message to LoRa module
                            char message[64];
                            snprintf(message, sizeof(message), "%.6f,%.6f\n", latitude, longitude);
                            uart_write_bytes(LORA_UART_NUM, message, strlen(message));
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
