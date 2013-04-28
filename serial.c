/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  
  UART0 ISR port to LPC2106 by Gary Rensberger

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This code was initially inspired by the wiring_serial module by David A. Mellis which
   used to be a part of the Arduino project. */ 

#include <math.h>
#include "armVIC.h"
#include "lpc210x.h"
#include "lpcUART.h"
#include "serial.h"
#include "config.h"
#include "motion_control.h"
#include "protocol.h"

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
uint8_t rx_buffer_tail = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

static bool uart0_tx_running=false;

//-----------------------------------------------------------------------------
void serial_init(void)
{
 	// set port pins for UART0
	PINSEL0 = (PINSEL0 & ~U0_PINMASK) | U0_PINSEL;

	U0IER = 0x00;                         // disable all interrupts
	U0IIR;                                // clear interrupt ID
	U0RBR;                                // clear receive register
	U0LSR;                                // clear line status register

	// set the baudrate - DLAB must be set to access DLL/DLM
	U0LCR = ULCR_DLAB_ENABLE;             	// select divisor latches 
	U0DLL = (uint8_t)B115200;        		// set for baud low byte
	U0DLM = (uint8_t)(B115200 >> 8);			// set for baud high byte

	// set the number of characters and other
	// user specified operating parameters
	U0LCR = UART_8N1 & ~ULCR_DLAB_ENABLE;	// set the mode
	U0FCR = UART_FIFO_1;					// and fifo size
	
	// initialize the interrupt vector
	VICIntSelect &= ~VIC_BIT(VIC_UART0);  // UART0 selected as IRQ
	VICIntEnable = VIC_BIT(VIC_UART0);    // UART0 interrupt enabled
	VICVectCntl1 = VIC_ENABLE | VIC_UART0;
	VICVectAddr1 = (uint32_t)uart0ISR;    // address of the ISR

	// enable receiver interrupts
	U0IER = UIER_ERBFI;	
}

//-----------------------------------------------------------------------------
void serial_write(uint8_t data) {
	uint8_t next_head;
	unsigned cpsr;

 	// Calculate next head
	next_head = (tx_buffer_head + 1) % TX_BUFFER_SIZE;

	// Wait until there is space in the buffer
	while (next_head == tx_buffer_tail) 
	{ 
		if (sys.execute & EXEC_RESET)
		return;  							// Only check for abort to avoid an endless loop.
	}

	cpsr = disableIRQ();                  	// disable global interrupts
	U0IER &= ~UIER_ETBEI;                 	// disable TX interrupts
	restoreIRQ(cpsr);                     	// restore global interrupts

	// check if in process of sending data
	if (uart0_tx_running)
    {
		// add to queue
		tx_buffer[tx_buffer_head] = data;
		tx_buffer_head = next_head;
    }
	else
    {
		// set running flag and write to output register
		uart0_tx_running = 1;
		U0THR = data;
    }

	cpsr = disableIRQ();                  // disable global interrupts
	U0IER |= UIER_ETBEI;                  // enable TX interrupts
	restoreIRQ(cpsr);                     // restore global interrupts
}


//-----------------------------------------------------------------------------
uint8_t serial_read(void)
{
  if (rx_buffer_head == rx_buffer_tail) 
		return SERIAL_NO_DATA;
  else 
  {
    uint8_t data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail++;
    if (rx_buffer_tail == RX_BUFFER_SIZE) 
		rx_buffer_tail = 0; 
    
    return data;
  }
}

//-----------------------------------------------------------------------------
void __attribute__ ((interrupt("IRQ"))) uart0ISR(void) 
{
	uint8_t iid;
	unsigned char c;
	uint8_t head;
	
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
					c = U0RBR;
					switch (c) 
					{	
						// Pick off runtime command characters directly from the serial stream. These characters are
						// not passed into the buffer, but these set system state flag bits for runtime execution.
						case CMD_STATUS_REPORT: 
							sys.execute |= EXEC_STATUS_REPORT; 
							break; 
						case CMD_CYCLE_START:   
							sys.execute |= EXEC_CYCLE_START; 
							break; 
						case CMD_FEED_HOLD:     
							sys.execute |= EXEC_FEED_HOLD; 
							break; 
						case CMD_RESET:         
							mc_reset(); 		// Call motion control reset routine.
							break; 
						default: 				// Write character to buffer     
							head = (rx_buffer_head + 1) % RX_BUFFER_SIZE;
							// if we should be storing the received character into the location
							// just before the tail (meaning that the head would advance to the
							// current location of the tail), we're about to overflow the buffer
							// and so we don't write the character or advance the head.
							if (head != rx_buffer_tail) 
							{
								rx_buffer[rx_buffer_head] = c;
								rx_buffer_head = head;
							}
					} 
				} while (U0LSR & ULSR_RDR);
				break;
				
			case UIIR_THRE_INT:        			// Transmit Holding Register Empty			
				while (U0LSR & ULSR_THRE)
				{
					// check if more data to send
					if (tx_buffer_head != tx_buffer_tail)
					{
						U0THR = tx_buffer[tx_buffer_tail++];
						tx_buffer_tail %= TX_BUFFER_SIZE;
					}
					else
					{
						// no more
						uart0_tx_running = 0;       // clear running flag
						break;
					}
				}		
				break;
				
			default:                          	// Unknown
				U0LSR;
				U0RBR;
				break;
		}
	}		
	VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
}




void serial_reset_read_buffer(void) 
{
  rx_buffer_tail = rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
