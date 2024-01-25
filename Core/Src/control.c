/*
 * control.c
 *
 *  Created on: Jan 20, 2023
 *      Author: Matt, Riyan, Ronak
 *
 */

#include "control.h"
#include "APPS.h"
#include "watchdog.h"
#include "utils.h"
#include "main.h"
#include <stdio.h>

//variable for pulse count & last time
volatile uint32_t pulse_count = 0;
uint32_t last_time = 0;

Ctrl_Data_Struct Ctrl_Data = {
	.wheelSpeed = {},
	.motorControllerTemp = 0,
	.accumulatorMaxTemp = 0,
	.coolantTemp = 0,
	.tractiveVoltage = 0,
	.motorSpeed = 0,
};

void startControlTask() {
	uint32_t tick = osKernelGetTickCount();
	while (1) {
		//commented out until watchdog is finished, does not work without watchdog
		//BSPC();
		//RTD();
		wSsensor();
		pumpCtrl();
		fanCtrl();
		osDelayUntil(tick += CTRL_PERIOD);
	}
}
//commented out until watchdog is finished
/*
// Brake system plausibility check
void BSPC() {
	if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
		// If the BSPC is in the invalid state,
		if (Watchdog_Data.flags & APPS_BSPC_INVALID){
			// RULE (2023 V2): EV.5.7.2 BSPC valid again if the pedal position is <5%
			if (APPS_Data.pedalPos < 5) {
				Watchdog_Data.flags &= ~APPS_BSPC_INVALID; // Remove the invalid flag
			}
		}
		// Only try to put the APPS into an invalid state if it's valid regarding the BSPC
		// RULE (2023 V2): EV.5.7.1 BSPC invalid if over >25% travel and brakes engaged
		else if (APPS_Data.pedalPos > 25 && HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port, GPIO_BRAKE_SW_Pin)) {
			Watchdog_Data.flags |= APPS_BSPC_INVALID; // Consider APPS as invalid due to BSPC
		}
		osMutexRelease(APPS_Data_MtxHandle);
	} else {
		ERROR_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): control.c:BSPC\n");
	}
}
*/

//commented out until watchdog is finished
/*
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
			if (Watchdog_Data.flags & APPS_RTD_INVALID){
				// Check if the pedal position is <3% to put APPS back into a valid state (EV.10.4.3)				
				if (APPS_Data.pedalPos < 3 && HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port, GPIO_BRAKE_SW_Pin) && Ctrl_Data.tractiveVoltage > RTD_TRACTIVE_VOLTAGE_ON && HAL_GPIO_ReadPin(GPIO_START_BTN_GPIO_Port, GPIO_START_BTN_Pin)) {
					Watchdog_Data.flags &= ~APPS_RTD_INVALID; // Remove the invalid flag
					HAL_GPIO_WritePin(GPIO_RTD_BUZZER_GPIO_Port, GPIO_RTD_BUZZER_Pin, 1);
					callCounts = 0;
				}				
			} 
			else if (Ctrl_Data.tractiveVoltage < RTD_TRACTIVE_VOLTAGE_OFF) {
				Watchdog_Data.flags |= RTD_INVALID; // Consider APPS as invalid due to RTD
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
*/



/*
 * calculates number of ticks registered within a second
 * divide number of ticks by the number of teeth on wheelspeed reader to get rpm
 * Calculate speed in KPH: 0.1885 * wheel RPM * diameter of tire
*/

void wSsensor() {
	uint32_t rpm = 0;

	//number of teeth on wheel speed reader
	uint16_t teeth = 15;

	//temporary diameter value, need to get tyre diameter
	float diameter = 0.5;

	//increment pulse count when changes seen
	pulse_count = TIM2->CNT;

	//current time variable, uses internal clock to get current time
	uint32_t current_time = HAL_GetTick();

	//calculate time difference, will be used for running calculation every second
	uint32_t time_diff = current_time - last_time;

	//if time difference is one second, run calculation to find rpm
	if(time_diff == 1){
		//calculate wheel rpm
		uint32_t rps = pulse_count / teeth;

		//convert rps to rpm, add to rpm value
		uint32_t conversion = rps * 60;

		//add converted value to rpm values that are already being stored
		rpm += conversion;

		printf("RPM = %ld", rpm);

		//flash lights based on number of ticks counted (for testing purposes)
		for(int i = 0; i < pulse_count; i++){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_Delay(250);
		}

		//reset rps value for next iteration and update last time value with current time
		rps = 0;
		last_time = current_time;
	}
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


