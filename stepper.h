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

#include "types.h"
#include "config.h"

#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#define sleep_mode();


void timer1_ISR(void) __attribute__ ((interrupt("IRQ"))) ;

// Initialize and setup the stepper motor subsystem
void st_init(void);

// Enable steppers, but cycle does not start unless called by motion control or runtime command.
void st_wake_up(void);

// Immediately disables steppers
void st_go_idle(void);

// Reset the stepper subsystem variables       
void st_reset(void);
             
// Notify the stepper subsystem to start executing the g-code program in buffer.
void st_cycle_start(void);

// Reinitializes the buffer after a feed hold for a resume.
void st_cycle_reinitialize(void); 

// Initiates a feed hold of the running program
void st_feed_hold(void);



#endif
