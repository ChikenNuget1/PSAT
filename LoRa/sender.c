/*!
 * \file      main.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"
#include "tremo_uart.h"

// --- UART Defines --- 
#define UART_RX_BUFFER_SIZE 255 // Max size for buffer of incoming UART data
uint8_t uartBuffer[UART_RX_BUFFER_SIZE];
uint16_t uartDataLen = 0; // Length of valid data

// --- Frequency Band Selection ---
#define REGION_AU915
#define USE_MODEM_LORA

// Regions
#if defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                470000000 // Hz

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 // Hz

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_US915_HYBRID )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

// LoRa Configuration Parameters
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )
// FSK Configuration
#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000   // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           83333   // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

// Statemachine definition
typedef enum
{
    LOWPOWER,
    RX,             // Packet Received Successfully
    RX_TIMEOUT,     // RX Operation time out
    RX_ERROR,       // RX Operation received corrupt packet
    TX,             // Packet transmission complete
    TX_TIMEOUT,     // TX Operation timeout
    UART_TO_RADIO   // UART Data ready to be sent over radio
}States_t;

#define RX_TIMEOUT_VALUE                            1800 // RX Timeout in ms
#define BUFFER_SIZE                                 255 // Define the payload size here

// const uint8_t PingMsg[] = "PING";
// const uint8_t PongMsg[] = "PONG";

// --- Radio Global Variables ---
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]; // Radio transmit/receive buffer

volatile States_t State = LOWPOWER; // Current state of radio operations

int8_t RssiValue = 0; // RSSI Value of last received packet
int8_t SnrValue = 0; // SNR Value of last received packet

uint32_t ChipId[2] = {0}; // MCU Chip ID (optional usage)

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

void ReadFromUART(void); // forwar declare

/**
 * @brief Main application entry point
 */
int app_start( void )
{
    bool isMaster = true;
    uint8_t i;
    uint32_t random;

    printf("PingPong test Start!\r\n");

    (void)system_get_chip_id(ChipId);

    // Radio initialization and Event registration
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    // Configure LoRa TX Parameters
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    
    // Configure LoRa RX Parameters
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    // Start Listening for packet
    Radio.Rx( RX_TIMEOUT_VALUE );

    // Main loop
    while( 1 )
    {
        // NEW PRINT STATEMENT
        if (uartDataLen == 0 && State == LOWPOWER)
        {
            // printf("LoRa Idle\r\n");
        }

        //printf("LoRa Idle: Listening for UART data...\r\n");

        /* read any UART bytes and (if any) set state to UART_TO_RADIO */
        ReadFromUART();

        // State machine
        switch( State )
        {
        case UART_TO_RADIO:
            if (uartDataLen > 0)
            {
                // Copy UART data to the radio transmit buffer
                BufferSize = uartDataLen;
                if (BufferSize > BUFFER_SIZE) BufferSize = BUFFER_SIZE; // Payload size cap

                memcpy(Buffer, uartBuffer, BufferSize);

                // Clear UART state for the next message
                memset(uartBuffer, 0, UART_RX_BUFFER_SIZE);
                uartDataLen = 0;

                // Stop current radio operations
                Radio.Sleep();

                // Transmit the payload
                Radio.Send(Buffer, BufferSize);

                printf("LoRa TX (%d bytes): ", BufferSize);
                // Print payload in hex
                for (int i = 0; i < BufferSize; i++) printf("%02X ", Buffer[i]);
                printf("\r\n");
            }

            // Return to low power state after sending
            State = LOWPOWER;
            break;

        case TX:
            /* just finished Tx (OnTxDone set this); go back to Rx/listen */
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
             /* RX timed out or failed CRC check; immediately resume listening */
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            /* TX failed to complete; immediately resume listening */
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // idle state
            break;
        }

        // Process radio interrupts.
        // Interrupts are then process in the main loop
        Radio.IrqProcess( );
    }
}

/* --- Radio callback implementations ---*/

/**
 * @brief Function called when a transmission completes successfully
 */
void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

/**
 * @brief Function called when a valid LoRa packet is received
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( ); // Stop radio check

    // Copy payload date
    BufferSize = size;
    if(BufferSize > BUFFER_SIZE) BufferSize = BUFFER_SIZE;
    memset(Buffer, 0, BUFFER_SIZE);
    memcpy( Buffer, payload, BufferSize );
    
    // Record signal quality
    RssiValue = rssi;
    SnrValue = snr;

    State = RX; // Set state for main loop to process received data
}

/**
 * @brief Function called when a transmission times out
 */
void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

/**
 * @brief Function called when reception times out
 */
void OnRxTimeout( void )
{
    printf("OnRxTimeout\r\n");
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

/**
 * @brief Function called if an RX error (e.g. CRC fail) occurs
 */
void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}   


// --- UART handling ---

/*
What the below code does is that it pretty much just stores bytes in a temporary buffer, msgBuffer, to 
prevent incorrect messages from being sent.
We check we get a full message if we receive a '\n' byte indicating "end-of-message".
This prevents the previous error of incomplete bytes being transmitted happening.
*/
#define UART_MSG_BUFFER_SIZE 255
static uint8_t msgBuffer[UART_MSG_BUFFER_SIZE]; // Temporary buffer for accumulating partial UART messages
static uint16_t msgLen = 0;                     // Acts as both the index for next byte and current length of the partial message

/**
 * @brief Non-blocking function to read bytes from UART and check for a complete message
 * * Data is accumulated until a newline ('\n') is received, triggering the radio send.
 */
void ReadFromUART(void) {
    // Check RX FIFO until empty
    while (uart_get_flag_status(UART0, UART_FLAG_RX_FIFO_EMPTY) == RESET) {
        uint8_t byte = uart_receive_data(UART0);

        // Buffer overflow check prevents writing past end of msgBuffer
        if (msgLen < UART_MSG_BUFFER_SIZE) {
            msgBuffer[msgLen++] = byte;
        }

        if (byte == '\n') { // full message
            memcpy(uartBuffer, msgBuffer, msgLen);      // Copy to the global TX buffer
            uartDataLen = msgLen;                       // Store the length
            msgLen = 0;                                 // Reset accumulation buffer
            State = UART_TO_RADIO;                      // Set state to trigger transmission in same loop
        }
    }
}