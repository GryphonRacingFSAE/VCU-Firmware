/*
 * control.c
 *
 * Created: Jan 20, 2023
 * Authors: Matt, Dallas, Riyan, Ronak
 *
 */

#include "control.h"
#include "APPS.h"
#include "utils.h"
#include "main.h"
#include <string.h>

//overflow counter array for wheel speed sensors
//each index corresponds to each wheel's overflow counter
static volatile uint16_t ws_Ovc[4] = {0};

//state variables for wheel speed (refers to first or second rising edge for each wheel)
//each index corresponds to each wheel's state (high or low for each index)
static volatile uint8_t ws_State[4] = {0};

//first rising edge for each sensor
static volatile uint32_t ws_Rising[4] = {0};

//second rising edge for each sensor
static volatile uint32_t ws_Rising2[4] = {0};


//wheel frequency array (stores each wheel's frequency)
volatile uint32_t wheelFreq[4];
volatile uint32_t wheelRPM[4];

//number of teeth on wheel hub
volatile uint16_t numTeeth = 32;

Ctrl_Data_Struct Ctrl_Data;

void startControlTask(){
	uint32_t tick = osKernelGetTickCount();
	while(1){

		for(int i = 0; i < 4; i++){
			GRCprintf("Frequency %d= %d\n\r", (i+1), wheelFreq[i]);
		}

		RPMConversion();
		BSPC();
		RTD();
		pumpCtrl();
		fanCtrl();
		LEDCtrl();
		osDelayUntil(tick += CTRL_PERIOD);
	}
}

/*
 * runs checks for period overflow, after 2 overflows the wheel is not moving
 * 	if the instance is equal to TIM1, hal tick is incremented in order to keep all timing accurate and synchronized
 *
 * 	if instance is equal to TIM2
 * 		increment tim2 overflow count
 * 			if overflow is greater than or equal to 2, set tim2 frequency to 0
 *
 * 	if instance is equal to TIM3
 * 		check if CH1 or CH2 is the active channel (same pattern of work for each channel just using channel specific variables)
 * 			increment overflow count
 * 				if overflow count is greater than or equal to 2, set wheel frequency to 0
 *
 * 	if instance is equal to TIM4
 * 		increment tim2 overflow count
 * 			if overflow is greater than or equal to 2, set tim2 frequency to 0

 */

void OverflowCheck(TIM_HandleTypeDef * htim){

	//instance for tim2
	if(htim -> Instance == TIM2){
		ws_Ovc[0]++;
		if(ws_Ovc[0] >= 2){
			wheelFreq[0] = 0;
		}
	}
	//instance for tim3 (channel checks)
	if (htim -> Instance == TIM3){
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			ws_Ovc[1] ++;
			if(ws_Ovc[1] >= 2){
				wheelFreq[1] = 0;
			}
		}
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			ws_Ovc[2] ++;
			if(ws_Ovc[2] >= 2){
				wheelFreq[2] = 0;
			}
		}
	}
	//instance for tim4
	if(htim -> Instance == TIM4){
		ws_Ovc[3] ++;
		if(ws_Ovc[3] >= 2){
			wheelFreq[3] = 0;
		}
	}
}

/*
 * Called when an input capture event occurs
 * using if statements (subject to change if more efficient method is found):
 * 		Check to see which timer the event occurs at (TIM2, TIM3 (need to also check which channel (CH1, CH2), TIM4)
 * for each instance the same algorithm is applied:
 *
 * 		if state 0:
 * 			store timer value within rising edge variable
 * 			calculate time interval between 2 rising edges at the same sensor
 * 			if ticks is not zero and overflow counter is less than 2, calculate frequency
 * 			store frequency within corresponding wheel frequency index in array
 * 			reset overflow counter, set state to 1
 * 		if state 1:
 * 			store timer value within falling edge variable
 * 			calculate time interval between 2 rising edges at the same sensor
 * 			if ticks is not zero and overflow counter is less than 2, calculate frequency
 * 			store frequency within corresponding wheel frequency index in array
 * 			reset overflow counter, set state to 0
 *
 * 	TIMER to array index correspondence:
 * 		TIM2 = 0
 * 		TIM3_CH1 = 1
 * 		TIM3_CH2 = 2
 * 		TIM4 = 3
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim){

	//timer 2 input
	if(htim -> Instance == TIM2){
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			//state 0
			if (ws_State[0] == 0){
				ws_Rising[0] = TIM2 -> CCR1;
				uint32_t ticks_TIM2 = (ws_Rising[0] + (ws_Ovc[0] * htim->Init.Period)) - ws_Rising2[0];
				if(ticks_TIM2 != 0 && ws_Ovc[0] < 2){
					wheelFreq[0] = (uint32_t)(96000000UL/ticks_TIM2);
				}
				ws_Ovc[0] = 0;
				ws_State[0] = 1;
			}
			//state 1
			if (ws_State[0] == 1){
				ws_Rising2[0] = TIM2 -> CCR1;
				uint32_t ticks_TIM2 = (ws_Rising2[0] + (ws_Ovc[0] * htim->Init.Period)) - ws_Rising[0];
				if(ticks_TIM2 != 0 && ws_Ovc[0] < 2){
					wheelFreq[0] = (uint32_t)(96000000UL/ticks_TIM2);
				}
				ws_Ovc[0] = 0;
				ws_State[0] = 0;
			}
		}
	}

	//timer 3 input
	if(htim -> Instance == TIM3){
		//channel 1
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			//state 0
			if(ws_State[1] == 0){
				ws_Rising[1] = TIM3 -> CCR1;
				uint32_t ticks_TIM3_CH1 = (ws_Rising[1] + (ws_Ovc[1] * htim -> Init.Period)) - ws_Rising2[1];
				if(ticks_TIM3_CH1 != 0 && ws_Ovc[1] < 2){
					wheelFreq[1] = (uint32_t)(96000000UL/ticks_TIM3_CH1);
				}
				ws_Ovc[1] = 0;
				ws_State[1] = 1;
			}
			//state 1
			if(ws_State[1] == 1){
				ws_Rising2[1] = TIM3 -> CCR1;
				uint32_t ticks_TIM3_CH1 = (ws_Rising2[1] + (ws_Ovc[1] * htim -> Init.Period)) - ws_Rising[1];
				if(ticks_TIM3_CH1 != 0 && ws_Ovc[1] < 2){
					wheelFreq[1] = (uint32_t)(96000000UL/ticks_TIM3_CH1);
				}
				ws_Ovc[1] = 0;
				ws_State[1] = 0;
			}
		}
	//channel 2
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			//state 0
			if(ws_State[2] == 0){
				ws_Rising[2] = TIM3 -> CCR1;
				uint32_t ticks_TIM3_CH2 = (ws_Rising[2] + (ws_Ovc[2] * htim -> Init.Period)) - ws_Rising2[2];
				if(ticks_TIM3_CH2 != 0 && ws_Ovc[2] < 2){
					wheelFreq[2] = (uint32_t)(96000000UL/ticks_TIM3_CH2);
				}
				ws_Ovc[2] = 0;
				ws_State[2] = 1;
			}
			//state 1
			if(ws_State[2] == 1){
				ws_Rising2[2] = TIM3 -> CCR1;
				uint32_t ticks_TIM3_CH2 = (ws_Rising2[2] + (ws_Ovc[2] * htim -> Init.Period)) - ws_Rising[2];
				if(ticks_TIM3_CH2 != 0 && ws_Ovc[2] < 2){
					wheelFreq[2] = (uint32_t)(96000000UL/ticks_TIM3_CH2);
				}
				ws_Ovc[2] = 0;
				ws_State[2] = 0;
			}
		}
	}
	//timer 4 input
	if(htim -> Instance == TIM4){
		if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			//state 0
			if(ws_State[3] == 0){
				ws_Rising[3] = TIM4 -> CCR1;
				uint32_t ticks_TIM4 = (ws_Rising[3] + (ws_Ovc[3] * htim -> Init.Period) - ws_Rising2[3]);
				if(ticks_TIM4 != 0 && ws_Ovc[3] <2){
					wheelFreq[3] = (uint32_t)(96000000UL/ticks_TIM4);
				}
				ws_Ovc[3] = 0;
				ws_State[3] = 1;
			}
			//state 1
			if(ws_State[3] == 1){
				ws_Rising2[3] = TIM4 -> CCR1;
				uint32_t ticks_TIM4 = (ws_Rising2[3] + (ws_Ovc[3] * htim -> Init.Period) - ws_Rising[3]);
				if(ticks_TIM4 != 0 && ws_Ovc[3] <2){
					wheelFreq[3] = (uint32_t)(96000000UL/ticks_TIM4);
				}
				ws_Ovc[3] = 0;
				ws_State[3] = 0;
			}
		}
	}

}

/*
 * Convert frequencies to rpm
 * 		run for loop for 4 iterations to parse through each wheel
 * 		multiply frequency by 60, divide by number of teeth on hub
 */

void RPMConversion(){
	for(int i = 0; i < 4; i++){
		wheelFreq[i];

		wheelRPM[i] = (wheelFreq[i] * 60)/ numTeeth;
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

void LEDCtrl() {

}

