#include <stdio.h>
#include <string.h>
#include "radio.h"
#include "tremo_system.h"

// Define Configurations
#define REGION_AU915
#define RX_TIMEOUT_VALUE 3000
#define BUFFER_SIZE 64

// Set carrier frequency to 915mhz
#if defined(REGION_AU915)
#define RF_FREQUENCY 915000000
#endif

// Global variables
uint8_t Buffer[BUFFER_SIZE]; // Buffer to store received radio payload
uint16_t BufferSize = 0; // Actual size of received payload

volatile uint8_t ReceivedFlag = 0; // Flat set by the interrupt handler

static RadioEvents_t RadioEvents; // Structure to hold pointers to the event handler functions

/* --- Event Handlers --- */

/**
 * @brief Handler called when a transmission completes succesfully
 */
void OnTxDone(void) {
    Radio.Sleep(); // After successful transmission, put radio to lowest possible power-state
}

/**
 * @brief Handler called when a valid LoRa packet is received.
 * @param payload Pointer to the received data buffer
 * @param size Actual size of the received payload
 * @param rssi Received Signal Strength Indicator
 * @param snr Signal-to-Noise Ratio
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Copy received data safely
    BufferSize = (size > BUFFER_SIZE) ? BUFFER_SIZE : size;
    memcpy(Buffer, payload, BufferSize);

    ReceivedFlag = 1;    // Signal main loop to process
    Radio.Sleep();       // Stop RX to process
}

/**
 * @brief Handler called if a transmission times out (not relevant for RX-only mode, but required).
 */
void OnTxTimeout(void) { 
    Radio.Sleep(); 
}

/**
 * @brief Handler called if no packet is received within the RX_TIMEOUT_VALUE.
 */
void OnRxTimeout(void) { 
    Radio.Sleep(); 
    Radio.Rx(RX_TIMEOUT_VALUE); 
}

/**
 * @brief Handler called if a packet is detected but fails the CRC check (corrupt data).
 */
void OnRxError(void) { 
    Radio.Sleep(); 
    Radio.Rx(RX_TIMEOUT_VALUE); 
}

/* --- Main Application --- */
int app_start(void) {
    // Starting message
    printf("LoRa Receiver Start!\r\n");

    // Initialize Radio events
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    // Initialise radio hardware and register event handlers
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    // Configure LoRa receiver parameters
    Radio.SetRxConfig(
        MODEM_LORA,
        0,      // Bandwidth 125 kHz
        7,      // Spreading factor
        1,      // Coding rate
        0,      // AFC bandwidth
        8,      // Preamble length
        0,      // Symbol timeout
        false,  // Fixed length payload
        0,      // Payload length
        true,   // Cyclic Redundancy Check
        0,      // Frequency Hop
        0,      // Hop Period
        false,  // IQ Inversion
        true    // Continuous RX
    );

    // Start receiving data
    Radio.Rx(RX_TIMEOUT_VALUE);

    while(1) {
        // process radio interrupts
        // This function handles an interrupt request from the LoRa module.
        // An interrupt request, is a request that tells the chip that something needs attention.
        // This function pauses current tasks to service the radio's signal.
        Radio.IrqProcess();

        // Check flag set by event handler
        if (ReceivedFlag) {
            ReceivedFlag = 0; // Reset immediately

            // --- Data Processing ---
            // Print payload as string
            printf("Received (%d bytes): ", BufferSize);
            for (int i = 0; i < BufferSize; i++) {
                printf("%c", Buffer[i]);
            }
            printf("\r\n");

            // Restart RX for next packet
            Radio.Rx(RX_TIMEOUT_VALUE);
        }

        // Small delay to avoid starving UART printf
        delay_ms(1);
    }
}