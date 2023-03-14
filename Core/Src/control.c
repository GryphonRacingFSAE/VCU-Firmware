/*
 * control.c
 *
 *  Created on: Jan 20, 2023
 *      Author: Matt
 *
 */

#include "control.h"
#include "APPS.h"
#include "utils.h"
#include "main.h"
#include <string.h>

Ctrl_Data_Struct Ctrl_Data;

void startControlTask() {
	uint32_t tick = osKernelGetTickCount();
	while (1) {
		BSPC();
		RTD();
		pumpCtrl();
		fanCtrl();
		osDelayUntil(tick += CTRL_PERIOD);
	}
}

// Brake system plausibility check
void BSPC() {
	if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
		// If the BSPC is in the invalid state,
		if (APPS_Data.flags & APPS_BSPC_INVALID){
			// Check if the pedal position is <5% to put APPS back into a valid state (EV.5.7.2)
			if (APPS_Data.pedalPos < 5) {
				APPS_Data.flags &= ~APPS_BSPC_INVALID; // Remove the invalid flag
			}
		}
		// Only try to put the APPS into an invalid state if it's valid regarding the BSPC
		// Set to invalid if over >25% travel and brakes engaged (EV.5.7.1)
		else if (APPS_Data.pedalPos > 25 && HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port, GPIO_BRAKE_SW_Pin)) {
			APPS_Data.flags |= APPS_BSPC_INVALID; // Consider APPS as invalid due to BSPC
		}
		osMutexRelease(APPS_Data_MtxHandle);
	} else {
		ERROR_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): control.c:BSPC\n");
	}
}

// Ready to drive
// Rules: EV.10.4.3 & EV.10.5
void RTD() {
	static int callCounts = 0;

	if (callCounts * CTRL_PERIOD > 2000) {
		HAL_GPIO_WritePin(GPIO_RTD_BUZZER_GPIO_Port, GPIO_RTD_BUZZER_Pin, 0);
	}
	if(osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK) {
		if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
			// If the RTD is in the invalid state,
			if (APPS_Data.flags & APPS_RTD_INVALID){
				// Check if the pedal position is <3% to put APPS back into a valid state (EV.10.4.3)				
				if (APPS_Data.pedalPos < 3 && HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port, GPIO_BRAKE_SW_Pin) && Ctrl_Data.tractiveVoltage > RTD_TRACTIVE_VOLTAGE_ON && HAL_GPIO_ReadPin(GPIO_START_BTN_GPIO_Port, GPIO_START_BTN_Pin)) {
					APPS_Data.flags &= ~APPS_RTD_INVALID; // Remove the invalid flag
					HAL_GPIO_WritePin(GPIO_RTD_BUZZER_GPIO_Port, GPIO_RTD_BUZZER_Pin, 1);
					callCounts = 0;
				}				
			} 
			else if (Ctrl_Data.tractiveVoltage < RTD_TRACTIVE_VOLTAGE_OFF) {
				APPS_Data.flags |= APPS_RTD_INVALID; // Consider APPS as invalid due to RTD
			}
			osMutexRelease(APPS_Data_MtxHandle);
		} else {
			ERROR_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): control.c:RTD\n");
		}
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		ERROR_PRINT("Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:RTD\n");
	}
	callCounts++;
}

// Motor & Motor controller cooling pump control
void pumpCtrl() {
	if (osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK){
		// Turn on pump based on motor controller temperature threshold and tractive voltage threshold
		HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin,
				Ctrl_Data.motorControllerTemp > PUMP_MOTOR_CONTROLLER_TEMP_THRESHOLD || Ctrl_Data.tractiveVoltage > PUMP_TRACTIVE_VOLTAGE_THRESHOLD);
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin, GPIO_PIN_SET); // Turn on pump if cannot acquire mutex
		ERROR_PRINT("Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:pumpCtrl\n");
	}
}

// Motor controller cooling fan control
void fanCtrl() {
	if (osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK){
		// Turn on fan based on coolant temperature threshold
		HAL_GPIO_WritePin(GPIO_RAD_FAN_GPIO_Port, GPIO_RAD_FAN_Pin,
				Ctrl_Data.coolantTemp > RAD_FAN_COOLANT_TEMP_THRESHOLD);
		HAL_GPIO_WritePin(GPIO_ACC_FAN_GPIO_Port, GPIO_ACC_FAN_Pin,
				Ctrl_Data.accumulatorMaxTemp > ACC_FAN_ACC_TEMP_THRESHOLD);
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		// Turn on fans if cannot acquire mutex
		HAL_GPIO_WritePin(GPIO_RAD_FAN_GPIO_Port, GPIO_RAD_FAN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_ACC_FAN_GPIO_Port, GPIO_ACC_FAN_Pin, GPIO_PIN_SET);
		ERROR_PRINT("Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:fanCtrl\n");
	}
}
