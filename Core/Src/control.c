/*
 * control.c
 *
 *  Created on: Jan 20, 2023
 *      Author: Matt
 *
 */

#include "control.h"
#include "APPS.h"
#include "watchdog.h"
#include "utils.h"
#include "main.h"

volatile uint16_t gu16_TIM2_OVC = 0;
volatile uint8_t gu8_State = 0;
volatile uint8_t gu8_MSG[35] = {'\0'};
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint32_t gu32_Freq = 0;

Ctrl_Data_Struct Ctrl_Data = {
	.wheelSpeed = {},
	.motorControllerTemp = 0,
	.accumulatorMaxTemp = 0,
	.coolantTemp = 0,
	.tractiveVoltage = 0,
	.motorSpeed = 0,
};

<<<<<<< Updated upstream
void startControlTask() {
	uint32_t tick = osKernelGetTickCount();
	while (1) {
//		ERROR_PRINT("Time thing: %d\n", TIM2->CCR1);
    	GRCprintf("Frequency = %lu Hz\r\n", gu32_Freq);
=======
//rising edge and falling edge variables for each sensor
volatile uint32_t TIM2_rising = 0;
volatile uint32_t TIM2_falling = 0;

volatile uint32_t TIM3_CH1_rising = 0;
volatile uint32_t TIM3_CH1_falling = 0;

volatile uint32_t TIM3_CH2_rising = 0;
volatile uint32_t TIM3_CH2_falling = 0;

volatile uint32_t TIM4_rising = 0;
volatile uint32_t TIM4_falling = 0;

//wheel frequency array (stores each wheel's frequency)
volatile uint32_t wheelFreq[4];
volatile uint32_t wheelRPM[4];

//number of teeth on wheel hub
volatile uint16_t numTeeth = 32;

Ctrl_Data_Struct Ctrl_Data;

void startControlTask() {
	uint32_t tick = osKernelGetTickCount();
	while (1) {
		for (int i = 0; i < 4; i++) {
			GRCprintf("Frequency %d= %d\n\r", (i + 1), wheelFreq[i]);
		}

		RPMConversion();
>>>>>>> Stashed changes
		BSPC();
		RTD();
		pumpCtrl();
		fanCtrl();
		LEDCtrl();
		osDelayUntil(tick += CTRL_PERIOD);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
	  gu16_TIM2_OVC++;
	  if (gu16_TIM2_OVC >= 2) {
		  gu32_Freq = 0;
	  }
  }

<<<<<<< Updated upstream
  /* USER CODE END Callback 1 */
=======
void OverflowCheck(TIM_HandleTypeDef *htim) {

	//instance for tim2
	if (htim->Instance == TIM2) {
		TIM2_OVC++;
		if (TIM2_OVC >= 2) {
			wheelFreq[0] = 0;
		}
	}
	if (htim->Instance == TIM3) {
		TIM3_CH1_OVC++;
		if (TIM3_CH1_OVC >= 2) {
			wheelFreq[1] = 0;
		}
		TIM3_CH2_OVC++;
		if (TIM3_CH2_OVC >= 2) {
			wheelFreq[3] = 0;
		}
	}
	if (htim->Instance == TIM4) {
		TIM4_OVC++;
		if (TIM4_OVC >= 2) {
			wheelFreq[2] = 0;
		}
	}
>>>>>>> Stashed changes
}


<<<<<<< Updated upstream

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if(gu8_State == 0)
    {
        gu32_T1 = TIM2->CCR1;
        gu32_Ticks = (gu32_T1 + (gu16_TIM2_OVC * htim->Init.Period )) - gu32_T2;
        if (gu32_Ticks != 0 && gu16_TIM2_OVC < 2) {
        	gu32_Freq = (uint32_t)(96000000UL/gu32_Ticks);
        }
        gu16_TIM2_OVC = 0;
        gu8_State = 1;
    }
    else if(gu8_State == 1)
    {
        gu32_T2 = TIM2->CCR1;
        gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * htim->Init.Period )) - gu32_T1;
        if (gu32_Ticks != 0 && gu16_TIM2_OVC < 2) {
			gu32_Freq = (uint32_t)(96000000UL/gu32_Ticks);
		}
        gu16_TIM2_OVC = 0;
        gu8_State = 0;
    }
}

=======
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	//timer 2 input
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			//state 0
			if (TIM2_State == 0) {
				TIM2_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM2 = (TIM2_rising
						+ (TIM2_OVC * htim->Init.Period)) - TIM2_falling;
				if (ticks_TIM2 != 0 && TIM2_OVC < 2) {
					wheelFreq[0] = (uint32_t) (96000000UL / ticks_TIM2);
				}
				TIM2_OVC = 0;
				TIM2_State = 1;
			}
			//state 1
			else {
				TIM2_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM2 = (TIM2_falling
						+ (TIM2_OVC * htim->Init.Period)) - TIM2_rising;
				if (ticks_TIM2 != 0 && TIM2_OVC < 2) {
					wheelFreq[0] = (uint32_t) (96000000UL / ticks_TIM2);
				}
				TIM2_OVC = 0;
				TIM2_State = 0;
			}
		}
	}

	//timer 3 input
	else if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			//state 0
			if (TIM3_CH1_State == 0) {
				TIM3_CH1_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM3_CH1 = (TIM3_CH1_rising
						+ (TIM3_CH1_OVC * htim->Init.Period))
						- TIM3_CH1_falling;
				if (ticks_TIM3_CH1 != 0 && TIM3_CH1_OVC < 2) {
					wheelFreq[1] = (uint32_t) (96000000UL / 256 / ticks_TIM3_CH1);
				}
				TIM3_CH1_OVC = 0;
				TIM3_CH1_State = 1;
			}
//			//state 1
			else {
				TIM3_CH1_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM3_CH1 = (TIM3_CH1_falling
						+ (TIM3_CH1_OVC * htim->Init.Period)) - TIM3_CH1_rising;
				if (ticks_TIM3_CH1 != 0 && TIM3_CH1_OVC < 2) {
					wheelFreq[1] = (uint32_t) (96000000UL / 256 / ticks_TIM3_CH1);
				}
				TIM3_CH1_OVC = 0;
				TIM3_CH1_State = 0;
			}
		}
		//channel 2
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			//state 0
			if (TIM3_CH2_State == 0) {
				TIM3_CH2_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				uint32_t ticks_TIM3_CH2 = (TIM3_CH2_rising
						+ (TIM3_CH2_OVC * htim->Init.Period))
						- TIM3_CH2_falling;
				if (ticks_TIM3_CH2 != 0 && TIM3_CH2_OVC < 2) {
					wheelFreq[3] = (uint32_t) (96000000UL / 256 / ticks_TIM3_CH2);
				}
				TIM3_CH2_OVC = 0;
				TIM3_CH2_State = 1;
			}
//			//state 1
			else {
				TIM3_CH2_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				uint32_t ticks_TIM3_CH2 = (TIM3_CH2_falling
						+ (TIM3_CH2_OVC * htim->Init.Period)) - TIM3_CH2_rising;
				if (ticks_TIM3_CH2 != 0 && TIM3_CH2_OVC < 2) {
					wheelFreq[3] = (uint32_t) (96000000UL / 256 / ticks_TIM3_CH2);
				}
				TIM3_CH2_OVC = 0;
				TIM3_CH2_State = 0;
			}
		}
	}
	//timer 4 input
	else if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			//state 0
			if (TIM4_State == 0) {
				TIM4_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM4 = (TIM4_rising
						+ (TIM4_OVC * htim->Init.Period) - TIM4_falling);
				if (ticks_TIM4 != 0 && TIM4_OVC < 2) {
					wheelFreq[2] = (uint32_t) (96000000UL / 256 / ticks_TIM4);
				}
				TIM4_OVC = 0;
				TIM4_State = 1;
			}
			//state 1
			else {
				TIM4_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				uint32_t ticks_TIM4 = (TIM4_falling
						+ (TIM4_OVC * htim->Init.Period) - TIM4_rising);
				if (ticks_TIM4 != 0 && TIM4_OVC < 2) {
					wheelFreq[2] = (uint32_t) (96000000UL / 256 / ticks_TIM4);
				}
				TIM4_OVC = 0;
				TIM4_State = 0;
			}
		}

	}
}

/*
 * Convert frequencies to rpm
 * 		run for loop for 4 iterations to parse through each wheel
 * 		multiply frequency by 60, divide by number of teeth on hub
 */

void RPMConversion() {
	for (int i = 0; i < 4; i++) {
		wheelFreq[i];

		wheelRPM[i] = (wheelFreq[i] * 60) / numTeeth;
	}
}
>>>>>>> Stashed changes

// Brake system plausibility check
void BSPC() {
	if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK) {
		// If the BSPC is in the invalid state,
<<<<<<< Updated upstream
		if (Watchdog_Data.flags & APPS_BSPC_INVALID){
			// RULE (2023 V2): EV.5.7.2 BSPC valid again if the pedal position is <5%
=======
		if (APPS_Data.flags & APPS_BSPC_INVALID) {
			// Check if the pedal position is <5% to put APPS back into a valid state (EV.5.7.2)
>>>>>>> Stashed changes
			if (APPS_Data.pedalPos < 5) {
				Watchdog_Data.flags &= ~APPS_BSPC_INVALID; // Remove the invalid flag
			}
		}
		// Only try to put the APPS into an invalid state if it's valid regarding the BSPC
<<<<<<< Updated upstream
		// RULE (2023 V2): EV.5.7.1 BSPC invalid if over >25% travel and brakes engaged
		else if (APPS_Data.pedalPos > 25 && HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port, GPIO_BRAKE_SW_Pin)) {
			Watchdog_Data.flags |= APPS_BSPC_INVALID; // Consider APPS as invalid due to BSPC
=======
		// Set to invalid if over >25% travel and brakes engaged (EV.5.7.1)
		else if (APPS_Data.pedalPos > 25
				&& HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port,
				GPIO_BRAKE_SW_Pin)) {
			APPS_Data.flags |= APPS_BSPC_INVALID; // Consider APPS as invalid due to BSPC
>>>>>>> Stashed changes
		}
		osMutexRelease(APPS_Data_MtxHandle);
	} else {
		ERROR_PRINT(
				"Missed osMutexAcquire(APPS_Data_MtxHandle): control.c:BSPC\n");
	}
}

// Ready to drive
// Rules: EV.10.4.3 & EV.10.5
void RTD() {
	static int callCounts = 0;

	if (callCounts * CTRL_PERIOD > 2000) {
		HAL_GPIO_WritePin(GPIO_RTD_BUZZER_GPIO_Port,
		GPIO_RTD_BUZZER_Pin, 0);
	}
	if (osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK) {
		if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK) {
			// If the RTD is in the invalid state,
<<<<<<< Updated upstream
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
=======
			if (APPS_Data.flags & APPS_RTD_INVALID) {
				// Check if the pedal position is <3% to put APPS back into a valid state (EV.10.4.3)
				if (APPS_Data.pedalPos < 3
						&& HAL_GPIO_ReadPin(GPIO_BRAKE_SW_GPIO_Port,
						GPIO_BRAKE_SW_Pin)
						&& Ctrl_Data.tractiveVoltage > RTD_TRACTIVE_VOLTAGE_ON
						&& HAL_GPIO_ReadPin(GPIO_START_BTN_GPIO_Port,
						GPIO_START_BTN_Pin)) {
					APPS_Data.flags &= ~APPS_RTD_INVALID; // Remove the invalid flag
					HAL_GPIO_WritePin(GPIO_RTD_BUZZER_GPIO_Port,
					GPIO_RTD_BUZZER_Pin, 1);
					callCounts = 0;
				}
			} else if (Ctrl_Data.tractiveVoltage < RTD_TRACTIVE_VOLTAGE_OFF) {
				APPS_Data.flags |= APPS_RTD_INVALID; // Consider APPS as invalid due to RTD
>>>>>>> Stashed changes
			}
			osMutexRelease(APPS_Data_MtxHandle);
		} else {
			ERROR_PRINT(
					"Missed osMutexAcquire(APPS_Data_MtxHandle): control.c:RTD\n");
		}
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		ERROR_PRINT(
				"Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:RTD\n");
	}
	callCounts++;
}

// Motor & Motor controller cooling pump control
void pumpCtrl() {
	if (osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK) {
		// Turn on pump based on motor controller temperature threshold and tractive voltage threshold
		HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin,
				Ctrl_Data.motorControllerTemp
						> PUMP_MOTOR_CONTROLLER_TEMP_THRESHOLD
						|| Ctrl_Data.tractiveVoltage
								> PUMP_TRACTIVE_VOLTAGE_THRESHOLD);
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin, GPIO_PIN_SET); // Turn on pump if cannot acquire mutex
		ERROR_PRINT(
				"Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:pumpCtrl\n");
	}
}

// Motor controller cooling fan control
void fanCtrl() {
	if (osMutexAcquire(Ctrl_Data_MtxHandle, 5) == osOK) {
		// Turn on fan based on coolant temperature threshold
		HAL_GPIO_WritePin(GPIO_RAD_FAN_GPIO_Port, GPIO_RAD_FAN_Pin,
				Ctrl_Data.coolantTemp > RAD_FAN_COOLANT_TEMP_THRESHOLD);
		HAL_GPIO_WritePin(GPIO_ACC_FAN_GPIO_Port, GPIO_ACC_FAN_Pin,
				Ctrl_Data.accumulatorMaxTemp > ACC_FAN_ACC_TEMP_THRESHOLD);
		osMutexRelease(Ctrl_Data_MtxHandle);
	} else {
		// Turn on fans if cannot acquire mutex
		HAL_GPIO_WritePin(GPIO_RAD_FAN_GPIO_Port, GPIO_RAD_FAN_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_ACC_FAN_GPIO_Port, GPIO_ACC_FAN_Pin,
				GPIO_PIN_SET);
		ERROR_PRINT(
				"Missed osMutexAcquire(Ctrl_Data_MtxHandle): control.c:fanCtrl\n");
	}
}

void LEDCtrl() {
	// LD1 = Green
	// LD2 = Blue
	// LD3 = Red


}
