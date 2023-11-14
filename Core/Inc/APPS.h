/*
 * APPS.h
 *
 *  Created on: Jan 19, 2023
 *      Author: Matt
 */

#ifndef INC_APPS_H_
#define INC_APPS_H_

#include "main.h"
#include "cmsis_os.h"


#define ADC1_CHANNEL_BUFF_LEN 1024
#define ADC1_CHANNELS 2
#define ADC1_BUFF_LEN (ADC1_CHANNEL_BUFF_LEN * ADC1_CHANNELS)

extern volatile uint16_t ADC1_buff[ADC1_BUFF_LEN];

extern osMutexId_t APPS_Data_MtxHandle;

typedef struct {
	//added to fix .torque error in APPS.c
	uint32_t torque;
	uint32_t flags;
	uint16_t pedalPos;
} APPS_Data_Struct;

#define APPS_BSPC_INVALID 0x1
#define APPS_RTD_INVALID 0x2
// RULE (2023 V2): T.4.2.10 Sensor out of defined range
#define APPS_SENSOR_OUT_OF_RANGE_INVALID 0x4
// RULE (2023 V2): T.4.2.4 APPS signals are within 10% of pedal position from each other
#define APPS_SENSOR_CONFLICT_INVALID 0x8


// RULE (2023 V2): T.4.3.4 BSE sensor out of defined range
#define BRAKE_SENSOR_OUT_OF_RANGE_INVALID 0x10

extern APPS_Data_Struct APPS_Data;

void startAPPSTask();
void requestTorque(int16_t requestedTorque);

extern osMutexId_t Torque_Map_MtxHandle;

#define TORQUE_MAP_ROWS 11
#define TORQUE_MAP_COLUMNS 14

typedef struct {
	int16_t map1[TORQUE_MAP_ROWS][TORQUE_MAP_COLUMNS];
	int16_t map2[TORQUE_MAP_ROWS][TORQUE_MAP_COLUMNS];
	int16_t (*activeMap)[TORQUE_MAP_COLUMNS];
} Torque_Map_Struct;

extern Torque_Map_Struct Torque_Map_Data;

#endif /* INC_APPS_H_ */
