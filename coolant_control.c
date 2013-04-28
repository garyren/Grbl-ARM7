/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012 Sungeun K. Jeon

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
#include "coolant_control.h"
#include "settings.h"
#include "config.h"
#include "planner.h"

#include "config.h"

// static uint8_t current_coolant_mode;

void coolant_init(void)
{
#if 0
  current_coolant_mode = COOLANT_DISABLE;
  #if ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
#endif
  coolant_stop();
}

void coolant_stop(void)
{
#if 0
  #ifdef ENABLE_M7
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
#endif
}


void coolant_run(uint8_t mode)
{
#if 0
  if (mode != current_coolant_mode)
  { 
    plan_synchronize(); // Ensure coolant turns on when specified in program.
    if (mode == COOLANT_FLOOD_ENABLE) { 
      COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
    #ifdef ENABLE_M7  
      } else if (mode == COOLANT_MIST_ENABLE) {
          COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
    #endif
    } else {
      coolant_stop();
    }
    current_coolant_mode = mode;
  }
#endif
}
