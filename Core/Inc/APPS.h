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

#define ADC1_BUFF_LEN 2048

extern volatile uint16_t ADC1_buff[ADC1_BUFF_LEN];

extern osMutexId_t APPS_Data_MtxHandle;

typedef struct {
	uint16_t torque;
	uint16_t pedalPos;
	uint32_t flags;
} APPS_Data_Struct;

#define APPS_BSPC_INVALID 0x1
#define APPS_RTD_INVALID 0x2

extern APPS_Data_Struct APPS_Data;

void startAPPSTask();

extern osMutexId_t Torque_Map_MtxHandle;

#define TORQUE_MAP_ROWS 11
#define TORQUE_MAP_COLUMNS 14

typedef struct {
	int16_t data[TORQUE_MAP_ROWS][TORQUE_MAP_COLUMNS];
} Individual_Torque_Map_Struct;

typedef struct {
	Individual_Torque_Map_Struct map1;
	Individual_Torque_Map_Struct map2;
	Individual_Torque_Map_Struct* activeMap;
} Torque_Map_Struct;

extern Torque_Map_Struct Torque_Map_Data;

#endif /* INC_APPS_H_ */
