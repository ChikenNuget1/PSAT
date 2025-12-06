#include <stdio.h>
#include <string.h>
#include "radio.h"
#include "tremo_system.h"

#define REGION_AU915
#define RX_TIMEOUT_VALUE 3000
#define BUFFER_SIZE 64

#if defined(REGION_AU915)
#define RF_FREQUENCY 915000000
#endif

uint8_t Buffer[BUFFER_SIZE];
uint16_t BufferSize = 0;

volatile uint8_t ReceivedFlag = 0;

static RadioEvents_t RadioEvents;

/* --- Event Handlers --- */
void OnTxDone(void) {
    Radio.Sleep();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Copy received data safely
    BufferSize = (size > BUFFER_SIZE) ? BUFFER_SIZE : size;
    memcpy(Buffer, payload, BufferSize);

    ReceivedFlag = 1;    // Signal main loop to process
    Radio.Sleep();       // Stop RX to process
}

void OnTxTimeout(void) { Radio.Sleep(); }
void OnRxTimeout(void) { Radio.Sleep(); Radio.Rx(RX_TIMEOUT_VALUE); }
void OnRxError(void) { Radio.Sleep(); Radio.Rx(RX_TIMEOUT_VALUE); }

/* --- Main Application --- */
int app_start(void) {
    printf("LoRa Receiver Start!\r\n");

    // Initialize Radio
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    // Configure LoRa RX
    Radio.SetRxConfig(
        MODEM_LORA,
        0,      // BW 125 kHz
        7,      // SF7
        1,      // CR 4/5
        0,      // AFC bandwidth
        8,      // Preamble
        0,      // Symbol timeout
        false,  // Fixed length payload
        0,      // Payload length ignored
        true,   // CRC
        0, 0, false, true // hop/IQ/continuous
    );

    // Start RX
    Radio.Rx(RX_TIMEOUT_VALUE);

    while(1) {
        Radio.IrqProcess(); // process radio interrupts

        if (ReceivedFlag) {
            ReceivedFlag = 0;

            // Print payload as string
            printf("Received (%d bytes): ", BufferSize);
            for (int i = 0; i < BufferSize; i++) {
                printf("%c", Buffer[i]);
            }
            printf("\r\n");

            // Optional: print hex for debugging
            // printf("Hex: ");
            // for(int i=0;i<BufferSize;i++) printf("%02X ", Buffer[i]);
            // printf("\r\n");

            // Restart RX
            Radio.Rx(RX_TIMEOUT_VALUE);
        }

        // Small delay to avoid starving UART printf
        delay_ms(1);
    }
}