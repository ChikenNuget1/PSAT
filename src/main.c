/* ESP-IDF example: forward console input to LoRa module over UART
   Targets ESP32-C6 (APIs same as other ESP32 chips).
   - Uses UART_NUM_1 for the LoRa connection.
   - Uses UART_NUM_0 as the console (default).
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lora-probe";

/* --- User configurable pins / settings --- */
/* Set these to match your wiring */
#define LORA_UART_NUM           UART_NUM_1   // UART used to talk to LoRa module
#define LORA_UART_TX_PIN        5            // ESP TX -> LoRa RX
#define LORA_UART_RX_PIN        4            // ESP RX <- LoRa TX (optional)
#define LORA_UART_RTS_PIN       UART_PIN_NO_CHANGE
#define LORA_UART_CTS_PIN       UART_PIN_NO_CHANGE

#define LORA_BAUD_RATE          115200         // Change if your LoRa uses a differsent baud
#define LORA_UART_BUF_SIZE      1024

/* --- Sends a sample packet periodically to LoRa --- */
static void periodic_sender_task(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(1000); // 1 seconds
    uint32_t counter = 0;

    while (1)
    {
        const char *payload = "hello\r\n";      // <-- new payload 
        int len = strlen(payload);

        // Queue bytes to UART FIFO
        int written = uart_write_bytes(LORA_UART_NUM, payload, len);
        ESP_LOGI(TAG, "Periodic: sent %d bytes to LoRa", written);

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

static void lora_probe_task(void *arg)
{
    const char *probe = "AT\r\n";
    uint8_t rxbuf[256];
    esp_err_t err;

    // 1) send probe and get number queued
    int w = uart_write_bytes(LORA_UART_NUM, probe, strlen(probe));
    ESP_LOGI(TAG, "Probe: uart_write_bytes() returned %d (queued to driver/FIFO)", w);

    // 2) wait until TX is done physically (blocks until FIFO+ringbuf emptied or timeout)
    err = uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(500)); // 500 ms
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "uart_wait_tx_done: TX finished (data physically transmitted)");
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "uart_wait_tx_done: TIMEOUT waiting for TX to finish");
    } else {
        ESP_LOGW(TAG, "uart_wait_tx_done: err=%d", err);
    }

    // 3) show how much free space is in the tx buffer now (diagnostic)
    size_t free_sz = 0;
    if (uart_get_tx_buffer_free_size(LORA_UART_NUM, &free_sz) == ESP_OK) {
        ESP_LOGI(TAG, "uart tx buffer free size: %u", (unsigned)free_sz);
    }

    // 4) allow the module some time to respond
    vTaskDelay(pdMS_TO_TICKS(200));

    // 5) attempt to read anything module might have sent back
    int r = uart_read_bytes(LORA_UART_NUM, rxbuf, sizeof(rxbuf)-1, pdMS_TO_TICKS(500));
    if (r > 0) {
        rxbuf[r] = '\0';
        ESP_LOGI(TAG, "LoRa -> ESP (%d bytes): %s", r, (char*)rxbuf);
    } else {
        ESP_LOGI(TAG, "No response from module (read r=%d)", r);
    }

    // 6) Try sending a short payload and wait similarly
    const char *payload = "TEST\r\n";
    w = uart_write_bytes(LORA_UART_NUM, payload, strlen(payload));
    ESP_LOGI(TAG, "Payload: uart_write_bytes() returned %d", w);

    err = uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "After payload, uart_wait_tx_done returned %d", err);

    vTaskDelete(NULL);
}

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
