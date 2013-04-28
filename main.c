/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud, ported to LPC2106 ARM7 by Gary Rensberger

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

#include "types.h" 
#include "armVIC.h"
#include "LPC210x.h"
#include "config.h"
#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "protocol.h"
#include "limits.h"
#include "report.h"
#include "settings.h"
#include "serial.h"

// Declare system global variable structure
system_t sys; 


static void sysInit(void);
static void lowInit(void);


/******************************************************************************
 *
 * Function Name: main()
 *
 * Description:
 *    This function is the program entry point.  After initializing the
 *    system, it sends a greeting out UART0 then enters an endless loop
 *    handling step commands and filling the step queue.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/

int main(void)
{  	
	sysInit(); 
	
	// Initialize system
	serial_init(); 			// Setup serial baud rate and interrupts
	enableIRQ();
	settings_init(); 		// Load grbl settings from EEPROM
	st_init(); 				// Setup stepper pins and interrupt timers
		

	memset(&sys, 0, sizeof(sys));  	// Clear all system variables
	sys.abort = true;   				// Set abort to complete initialization
	sys.state = STATE_INIT;  			// Set alarm state to indicate unknown initial position

	
	for(;;) 
	{ 
		// Execute system reset upon a system abort, where the main program will return to this loop.
		// Once here, it is safe to re-initialize the system. At startup, the system will automatically
		// reset to finish the initialization process.
		if (sys.abort) 
		{
			// Reset system.
			serial_reset_read_buffer(); // Clear serial read buffer
			plan_init(); 				// Clear block buffer and planner variables
			gc_init(); 					// Set g-code parser to default state
			protocol_init(); 			// Clear incoming line data and execute startup lines
			spindle_init();
			coolant_init();
			limits_init();
			st_reset(); 				// Clear stepper subsystem variables.

			// Sync cleared gcode and planner positions to current system position, which is only
			// cleared upon startup, not a reset/abort. 
			sys_sync_current_position();

			// Reset system variables.
			sys.abort = false;
			sys.execute = 0;
			if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }
		  
			// Check for power-up and set system alarm if homing is enabled to force homing cycle
			// by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
			// startup scripts, but allows access to settings and internal commands. Only a homing
			// cycle '$H' or kill alarm locks '$X' will disable the alarm.
			// NOTE: The startup script will run after successful completion of the homing cycle, but
			// not after disabling the alarm locks. Prevents motion startup blocks from crashing into
			// things uncontrollably. Very bad.
			#ifdef HOMING_INIT_LOCK
			if (sys.state == STATE_INIT && bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
			#endif
		  
			// Check for and report alarm state after a reset, error, or an initial power up.
			if (sys.state == STATE_ALARM) 
			{
				report_feedback_message(MESSAGE_ALARM_LOCK); 
			} else 
			{
				// All systems go. Set system to ready and execute startup script.
				sys.state = STATE_IDLE;
				protocol_execute_startup(); 
			}
		}
		
		protocol_execute_runtime();
		protocol_process(); 				// ... process the serial protocol
		
	}
	return 0;   /* never reached */
}



/******************************************************************************
 *
 * Function Name: lowInit()
 *
 * Description:
 *    This function starts up the PLL then sets up the GPIO pins before
 *    waiting for the PLL to lock.  It finally engages the PLL and
 *    returns
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
static void lowInit(void)
{
  // set PLL multiplier & divisor.
  // values computed from config.h
  PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;

  // enable PLL
  PLLCON = PLLCON_PLLE;
  PLLFEED = 0xAA;                       // Make it happen.  These two updates
  PLLFEED = 0x55;                       // MUST occur in sequence.

  // setup the parallel port pin
  // IOCLR = PIO_ZERO_BITS;                // clear the ZEROs output
  // IOSET = PIO_ONE_BITS;                 // set the ONEs output
  // IODIR = PIO_OUTPUT_BITS;              // set the output bit direction

  // wait for PLL lock
  while (!(PLLSTAT & PLLSTAT_LOCK))
    continue;

  // enable & connect PLL
  PLLCON = PLLCON_PLLE | PLLCON_PLLC;
  PLLFEED = 0xAA;                       // Make it happen.  These two updates
  PLLFEED = 0x55;                       // MUST occur in sequence.

  // setup & enable the MAM
  MAMTIM = MAMTIM_CYCLES;
  MAMCR = MAMCR_FULL;

  // set the peripheral bus speed
  // value computed from config.h
  VPBDIV = VPBDIV_VALUE;                // set the peripheral bus clock speed
}

/******************************************************************************
 *
 * Function Name: sysInit()
 *
 * Description:
 *    This function is responsible for initializing the program
 *    specific hardware
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
static void sysInit(void)
{
	lowInit();                          // setup clocks and processor port pins

	// set the interrupt controller defaults
#if defined(RAM_RUN)
	MEMMAP = MEMMAP_SRAM;               // map interrupt vectors space into SRAM
#elif defined(ROM_RUN)
	MEMMAP = MEMMAP_FLASH;              // map interrupt vectors space into FLASH
#else
	#error RUN_MODE not defined!
#endif
	VICIntEnClear = 0xFFFFFFFF;         // clear all interrupts
	VICIntSelect = 0x00000000;          // clear all FIQ selections
	VICDefVectAddr = (uint32_t)reset;  	// point unvectored IRQs to reset()

	IODIR &= ~(bfI2C_SCL|bfI2C_SDA);
	IOCLR = bfI2C_SCL;					// I2C bits are output low
	IOCLR = bfI2C_SDA;					// 

//  wdtInit();                        	// initialize the watchdog timer
}



