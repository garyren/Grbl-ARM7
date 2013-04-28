/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides interface routines to the LPC ARM UARTs.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#include <limits.h>
#include "types.h"
#include "LPC210x.h"
#include "uart.h"
#include "armVIC.h"





/******************************************************************************
 *
 * Function Name: initUART0()
 *
 * Description:  
 *    This function initializes the UART for async mode
 *
 * Calling Sequence: 
 *    baudrate divisor - use UART_BAUD macro
 *    mode - see typical modes (uart.h)
 *    fmode - see typical fmodes (uart.h)
 *
 * Returns:
 *    void
 *
 * NOTE: uart0Init(UART_BAUD(9600), UART_8N1, UART_FIFO_8);
 *
 *****************************************************************************/
void initUART0(uint16_t baud, uint8_t mode, uint8_t fmode)
{
  // set port pins for UART0
  PINSEL0 = (PINSEL0 & ~U0_PINMASK) | U0_PINSEL;

  U0IER = 0x00;                         // disable all interrupts
  U0IIR;                                // clear interrupt ID
  U0RBR;                                // clear receive register
  U0LSR;                                // clear line status register

  // set the baudrate
  U0LCR = ULCR_DLAB_ENABLE;             // select divisor latches 
  U0DLL = (uint8_t)baud;                // set for baud low byte
  U0DLM = (uint8_t)(baud >> 8);         // set for baud high byte

  // set the number of characters and other
  // user specified operating parameters
  U0LCR = (mode & ~ULCR_DLAB_ENABLE);
  U0FCR = fmode;

  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_UART0);  // UART0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART0);    // UART0 interrupt enabled
  VICVectCntl1 = VIC_ENABLE | VIC_UART0;
  VICVectAddr1 = (uint32_t)uart0ISR;    // address of the ISR

  // initialize the transmit data queue
  txOutPtr0 = txInPtr0 = 0;
  uart0_tx_running = 0;

  // initialize the receive data queue
  rxOutPtr0 = rxInPtr0 = 0;

  // enable receiver interrupts
  U0IER = UIER_ERBFI;
}



/******************************************************************************
 *
 * Function Name: sendChar0()
 *
 * Description:  
 *    This function puts a character into the UART output queue for
 *    transmission.
 *
 * Calling Sequence: 
 *    character to be transmitted
 *
 *****************************************************************************/
void sendChar0(int ch)
{
	uint16_t temp;

	do
	{
		temp = (txInPtr0 + 1) % UART0_TX_BUFFER_SIZE;
	} while (temp == txOutPtr0);

	// check if in process of sending data
	if (uart0_tx_running)
    {
		// add to queue
		uart0_tx_buffer[txInPtr0] = (uint8_t)ch;
		txInPtr0 = temp;
    }
	else
    {
		// set running flag and write to output register
		uart0_tx_running = 1;
		U0THR = (uint8_t)ch;
		U0IER |= UIER_ETBEI;                  // enable TX interrupts
    }	
}


void sendCR0(void)
{
	sendChar0(0x0D);
	sendChar0(0x0A);
}


void sendSpace0(void)
{
	sendChar0(0x20);
}

void send8Hex0(char nibble2)
{
	char temp;
	
	temp=nibble2;
	
	nibble2 = (nibble2>>4) & 0x0F;
	if (nibble2 < 0x0A)
		nibble2 |= 0x30;
	else
		nibble2 += (0x41-0x0A);
	sendChar0(nibble2);
	
	temp &= 0x0F;
	if (temp < 0x0A)
		temp |= 0x30;
	else
		temp += (0x41-0x0A);
	sendChar0(temp);
}

 
void send16Hex0(int value)
{
	int i;
	
	for (i=1; i>=0; i--)
		send8Hex0((value >> (i*8)) & 0xFF);
	sendSpace0();
}	

void send32Hex0(int value)
{
	int i;
	
	for (i=3; i>=0; i--)
		send8Hex0((value >> (i*8)) & 0xFF);
	sendSpace0();
}


	
/******************************************************************************
 *
 * Function Name: sendString0()
 *
 * Description:  
 *    This function writes a NULL terminated 'string' to the UART0 output
 *    queue
 *
 * Calling Sequence: 
 *    address of the string
 *
 *****************************************************************************/
void sendString0(const char *string)
{
	register char ch;

	while ((ch = *string))
	{
		sendChar0(ch);
		string++;
	}
}


/******************************************************************************
 *
 * Function Name: sendBytes0()
 *
 * Description:  
 *    This function writes 'count' characters from 'buffer' to the UART
 *    output queue.
 *
 * Calling Sequence: 
 *    
 *
 *****************************************************************************/
void sendBytes0(const char *buffer, uint16_t count)
{
	while (count)
	{
		sendChar0(*buffer++);
		count--;
	}
}



/******************************************************************************
 *
 * Function Name: getChar0()
 *
 * Description:  
 *    This function gets a character from the UART receive queue
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    character on success, -1 if no character is available
 *
 *****************************************************************************/
int  getChar0(void)
{
  uint8_t ch;

  if (rxInPtr0 == rxOutPtr0) 		// check if character is available
    return -1;

  ch = uart0_rx_buffer[rxOutPtr0++]; // get character, bump pointer
  rxOutPtr0 %= UART0_RX_BUFFER_SIZE; // limit the pointer
  return ch;
}





/******************************************************************************
 *
 * Function Name: uart0ISR()
 *
 * Description:
 *    This function implements the ISR for UART0.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void __attribute__ ((interrupt("IRQ"))) uart0ISR(void) 
{
  uint8_t iid;

  // loop until not more interrupt sources
  while (((iid = U0IIR) & UIIR_NO_INT) == 0)
  {
    // identify & process the highest priority interrupt
    switch (iid & UIIR_ID_MASK)
	{
		case UIIR_RLS_INT:            	// Receive Line Status
			U0LSR;                     	// read LSR to clear
			break;

		case UIIR_CTI_INT:              // Character Timeout Indicator
		case UIIR_RDA_INT:            	// Receive Data Available
			do
			{
				uint16_t temp;

				// calc next insert index & store character
				temp = (rxInPtr0 + 1) % UART0_RX_BUFFER_SIZE;
				uart0_rx_buffer[rxInPtr0] = U0RBR;
				// check for more room in queue
				if (temp != rxOutPtr0)
					rxInPtr0 = temp; // update insert index
			} while (U0LSR & ULSR_RDR);

        break;

		case UIIR_THRE_INT:               // Transmit Holding Register Empty
			while (U0LSR & ULSR_THRE)
			{
				// check if more data to send
				if (txInPtr0 != txOutPtr0)
				{
					U0THR = uart0_tx_buffer[txOutPtr0++];
					txOutPtr0 %= UART0_TX_BUFFER_SIZE;
				}
				else
				{
					// no
					uart0_tx_running = 0;       // clear running flag
					break;
				}
			}
			break;

			default:                          // Unknown
				U0LSR;
				U0RBR;
				break;
		}
	}

	VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
}
