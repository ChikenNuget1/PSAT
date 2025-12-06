/* ESP-IDF example: forward console input to LoRa module over UART
   Targets ESP32-C6 (APIs same as other ESP32 chips).
   - Uses UART_NUM_1 for the GPS connection.
   - Uses UART_NUM_0 for the LoRa connection.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "esp_err.h"

static const char *TAG = "lora-probe";

// GPS Configuration
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 23 // ESP32 TX → GPS RX
#define GPS_RX_PIN 22  // ESP32 RX ← GPS TX
#define BUF_SIZE 1024

// LoRa Configuration
#define LORA_UART_NUM           UART_NUM_0   // UART used to talk to LoRa module
#define LORA_UART_TX_PIN        5            // ESP TX -> LoRa RX
#define LORA_UART_RX_PIN        4            // ESP RX <- LoRa TX (optional)
#define LORA_UART_RTS_PIN       UART_PIN_NO_CHANGE
#define LORA_UART_CTS_PIN       UART_PIN_NO_CHANGE

#define LORA_BAUD_RATE          115200         // Change if your LoRa uses a differsent baud
#define LORA_UART_BUF_SIZE      1024


// -------------------------------------
// -----------GPS functions-------------
// -------------------------------------

/**
 * @brief Converts NMEA coordinate format (DDDD.MMMM) to decimal degrees
 * * @param nmea NMEA coordinate string
 * @param direction Cardinal direction
 * @return double Decimal degree value
 */
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

/**
 * @brief initialises the GPS UART and sets parameters
 */
void gps_setup(){
    uart_config_t uart_config_gps = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART_NUM, &uart_config_gps);

    // Install UART driver
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0); 

    // Set UART pins
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Send configuration commands to GPS
    const char *set_rate = "$PMTK220,200*2C\r\n"; // 5 Hz
    uart_write_bytes(GPS_UART_NUM, set_rate, strlen(set_rate));

    const char *set_sentences = "$PMTK314,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; 
    uart_write_bytes(GPS_UART_NUM, set_sentences, strlen(set_sentences));
}
/**
 * @brief Reads data from the GPS UART, parses RMC, and GGA sentences and updates
 * a static formatted string containing location and satellite count.
 * @return const char* Pointer to the static location string buffer
 */
const char* read_gps(){
    // Static variables to maintain latest data and buffer calls.
    static char locationStr[64] = "No GPS fix yet\r\n"; 
    static char line[BUF_SIZE];
    static int line_pos = 0;
    static int satellites_in_use = 0;
    uint8_t data[BUF_SIZE];

    // Read available bytes
    int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 500 / portTICK_PERIOD_MS);

    if(len > 0) {
        for(int i = 0; i < len; i++) {
            char c = data[i];
            if(c == '\n') { // End of NMEA Sentence detected
                line[line_pos] = '\0';
                
                // Trim trailing \r (using the previous fix logic)
                if (line_pos > 0 && line[line_pos - 1] == '\r') {
                    line[line_pos - 1] = '\0';
                }
                
                line_pos = 0; // Reset buffer

                // --- GNRMC/GPRMC Parsing (Existing Location Logic) ---
                if(strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
                    char *token = strtok(line, ",");
                    int field = 0;
                    char *lat_str = NULL, *lat_dir = NULL, *lon_str = NULL, *lon_dir = NULL;

                    while(token) {
                        field++;
                        if(field == 4) lat_str = token; // Latitude
                        if(field == 5) lat_dir = token; // N/S Indicator
                        if(field == 6) lon_str = token; // Longitude
                        if(field == 7) lon_dir = token; // W/E Indicator
                        token = strtok(NULL, ",");
                    }

                    if(lat_str && lat_dir && lon_str && lon_dir) {
                        double latitude = nmea_to_decimal(lat_str, lat_dir[0]);
                        double longitude = nmea_to_decimal(lon_str, lon_dir[0]);
                        
                        // UPDATE locationStr to include latest location and satellite count
                        snprintf(locationStr, sizeof(locationStr), 
                                 "Lat: %.6f, Lon: %.6f, Sats: %d\r\n", 
                                 latitude, longitude, satellites_in_use);
                    }
                }
                
                // --- GNGGA/GPGGA Parsing for Satellites ---
                else if(strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
                    char temp_line[BUF_SIZE];
                    strncpy(temp_line, line, BUF_SIZE); // Use copy for strtok
                    
                    char *token = strtok(temp_line, ",");
                    int field = 0;
                    char *sat_count_str = NULL;

                    while(token) {
                        field++;
                        // Field 8 of GGA is the number of satellites in use
                        if(field == 8) sat_count_str = token; 
                        token = strtok(NULL, ",");
                    }

                    if(sat_count_str && *sat_count_str != '\0') {
                        satellites_in_use = atoi(sat_count_str); // Update satellite count
                    }
                }
                // --- END Parsing ---
                

            } else if(c != '\r') {
                // Buffer overflow protection
                line[line_pos++] = c;
                if(line_pos >= BUF_SIZE - 1) line_pos = BUF_SIZE - 2;
            }
        }
    }

    return locationStr;
}


// -------------------------------------
// -------------LoRa Tasks--------------
// -------------------------------------
/* --- Sends a sample packet periodically to LoRa --- */

/**
 * @brief Periodically send the latest GPS data via UART
 */
static void periodic_sender_task(void *arg)
{
    // Delay between transmissions
    const TickType_t delay = pdMS_TO_TICKS(1000); // 1 seconds
    uint32_t counter = 0;

    while (1)
    {
        // Retrieve latest GPS data
        const char *payload = read_gps();
        if(payload) {
            // Queue bytes for transmission
            uart_write_bytes(LORA_UART_NUM, payload, strlen(payload));
        }

        // Wait until all bytes are physically transmitted
        esp_err_t err = uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(500));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "TX physically finished");
        } else {
            ESP_LOGW(TAG, "TX not finished (timeout)");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

/**
 * Main Function
 */
void app_main(void)
{
    /* --- Configure LoRa UART (UART1) --- */
    uart_config_t uart_config = {
        .baud_rate = LORA_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Disable all ESP-IDF Logging to prevent interference with UART_0
    esp_log_level_set("*", ESP_LOG_NONE);
    
    // Set up GPS Configs
    gps_setup();

    // Install driver for LORA_UART_NUM: RX + TX buffers
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, LORA_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_UART_TX_PIN, LORA_UART_RX_PIN, LORA_UART_RTS_PIN, LORA_UART_CTS_PIN));

    /* --- (Optional) Reconfigure console UART if needed - usually already configured by system --- */
    // If you need to change console settings uncomment and adapt:
    // uart_config_t console_cfg = { .baud_rate = 115200, ... };
    // uart_param_config(CONSOLE_UART_NUM, &console_cfg);

    ESP_LOGI(TAG, "UART to LoRa initialized. TX pin=%d RX pin=%d baud=%d", LORA_UART_TX_PIN, LORA_UART_RX_PIN, LORA_BAUD_RATE);
    ESP_LOGI(TAG, "Wiring reminder: ESP TX -> LoRa RX, ESP RX -> LoRa TX, common GND");

      /* --- Start periodic sender task --- */
      xTaskCreate(periodic_sender_task, "lora-probe", 4096, NULL, 5, NULL);
}
