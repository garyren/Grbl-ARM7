/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides the interface definitions for for uart.c
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_UART_H
#define INC_UART_H

#include "types.h"
#include "LPC210x.h"
#include "config.h"







///////////////////////////////////////////////////////////////////////////////
// code is optimized for power of 2 buffer sizes (16, 32, 64, 128, ...)
// NOTE: the buffers are only used if the respective interrupt mode is
// enabled
#define UART0_RX_BUFFER_SIZE 64         // UART0 receive buffer size
#define UART0_TX_BUFFER_SIZE 64         // UART0 transmit buffer size

///////////////////////////////////////////////////////////////////////////////
// use the following macros to determine the 'baud' parameter values
// for uart0Init() and uart1Init()
// CAUTION - 'baud' SHOULD ALWAYS BE A CONSTANT or
// a lot of code will be generated.
#define UART_BAUD(baud) (uint16_t)((PCLK / ((baud) * 16.0)) + 0.5)

///////////////////////////////////////////////////////////////////////////////
// Definitions for typical UART 'baud' settings
#define B1200         UART_BAUD(1200)
#define B9600         UART_BAUD(9600)
#define B19200        UART_BAUD(19200)
#define B38400        UART_BAUD(38400)
#define B57600        UART_BAUD(57600)
#define B115200       UART_BAUD(115200)

///////////////////////////////////////////////////////////////////////////////
// Definitions for typical UART 'mode' settings
#define UART_8N1      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_NO   + ULCR_STOP_1)
#define UART_7N1      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_NO   + ULCR_STOP_1)
#define UART_8N2      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_NO   + ULCR_STOP_2)
#define UART_7N2      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_NO   + ULCR_STOP_2)
#define UART_8E1      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_EVEN + ULCR_STOP_1)
#define UART_7E1      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_EVEN + ULCR_STOP_1)
#define UART_8E2      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_EVEN + ULCR_STOP_2)
#define UART_7E2      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_EVEN + ULCR_STOP_2)
#define UART_8O1      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_ODD  + ULCR_STOP_1)
#define UART_7O1      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_ODD  + ULCR_STOP_1)
#define UART_8O2      (uint8_t)(ULCR_CHAR_8 + ULCR_PAR_ODD  + ULCR_STOP_2)
#define UART_7O2      (uint8_t)(ULCR_CHAR_7 + ULCR_PAR_ODD  + ULCR_STOP_2)

///////////////////////////////////////////////////////////////////////////////
// Definitions for typical UART 'fmode' settings
#define UART_FIFO_OFF (0x00)
#define UART_FIFO_1   (uint8_t)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG1)
#define UART_FIFO_4   (uint8_t)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG4)
#define UART_FIFO_8   (uint8_t)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG8)
#define UART_FIFO_14  (uint8_t)(UFCR_FIFO_ENABLE + UFCR_FIFO_TRIG14)

uint8_t  uart0_rx_buffer[UART0_RX_BUFFER_SIZE];
volatile uint16_t rxInPtr0, rxOutPtr0;
uint8_t  uart0_tx_buffer[UART0_TX_BUFFER_SIZE];
volatile uint16_t txInPtr0, txOutPtr0;

int      uart0_tx_running;


///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 *
 * Function Name: initUART0()
 *
 * Description:  
 *    This function initializes the UART for async mode
 *
 * Calling Sequence: 
 *    baudrate divisor - use UART0_BAUD macro
 *    mode - see typical modes (above)
 *    fmode - see typical fmodes (above)
 *
 * Returns:
 *    void
 *
 * NOTE: uart0Init(UART_BAUD(9600), UART_8N1, UART_FIFO_8);
 *
 *****************************************************************************/

void uart0ISR(void);
void initUART0(uint16_t baud, uint8_t mode, uint8_t fmode);

void sendChar0(int ch);
void sendCR0(void);
void sendSpace0(void);
void send8Hex0(char nibble2);
void send16Hex0(int value);
void send32Hex0(int value);
//void printf(const char *string, int value);
void sendString0(const char *string);
void sendBytes0(const char *buffer, uint16_t count);

int  getChar0(void);


#endif
