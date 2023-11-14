/*
 * watchdog.h
 *
 *  Created on: Apr 10, 2023
 *      Author: Dallas, Riyan
 *      errors fixed - Riyan
 */

//included for uint32_t error
#include <stdint.h>
//included for osMutexId_t error
#include "cmsis_os.h"

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#define BSPC_INVALID 0x1
#define RTD_INVALID 0x2
// RULE (2023 V2): T.4.2.10 Sensor out of defined range
#define APPS_SENSOR_OUT_OF_RANGE_INVALID 0x4
// RULE (2023 V2): T.4.2.4 APPS signals are within 10% of pedal position from each other
#define APPS_SENSOR_CONFLICT_INVALID 0x8

// RULE (2023 V2): T.4.3.4 BSE sensor out of defined range
#define BRAKE_SENSOR_OUT_OF_RANGE_INVALID 0x10

typedef struct {
	uint32_t flags;
	uint32_t tick_stamp[32];
	const uint32_t tick_limit[32];
} Watchdog_Data_Struct;

extern Watchdog_Data_Struct Watchdog_Data;
extern osMutexId_t Watchdog_Data_MtxHandle;

// This task watches the following rules, and shuts down motor controller if invalid
// RTD, BSPC, T.4.2.5, T.4.3.3
void startWatchdogTask();

#endif /* INC_WATCHDOG_H_ */
