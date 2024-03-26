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

void startControlTask() {
	uint32_t tick = osKernelGetTickCount();
	while (1) {
//		ERROR_PRINT("Time thing: %d\n", TIM2->CCR1);
    	GRCprintf("Frequency = %lu Hz\r\n", gu32_Freq);
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

  /* USER CODE END Callback 1 */
}



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
	// LD1 = Green
	// LD2 = Blue
	// LD3 = Red
}

void vicorInit(I2C_HandleTypeDef *i2cHandle) {
    HAL_StatusTypeDef status;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High-speed output
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // Alternate function for I2C1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    // Enable clock for GPIOB peripheral
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Enable clock for I2C peripheral
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Initialize the I2C peripheral
    i2cHandle->Instance = I2C1;
    i2cHandle->Init.Timing = 0x00707CBB; // Refer to reference manual for appropriate timing configuration
    i2cHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
    i2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual addressing mode
    i2cHandle->Init.OwnAddress1 = 0x00; // Not used in master mode
    i2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call mode
    i2cHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Enable clock stretching
    status = HAL_I2C_Init(i2cHandle); // Initialize the I2C peripheral and assign status
    if (status != HAL_OK) {
    	GRCprintf("Error initializing I2C peripheral: %d\n", status);
            return;
        }

        // Initialization successful
    GRCprintf("VICOR initialization successful.\n");

}

void vicorDeInit(I2C_HandleTypeDef *i2cHandle) {
    // Disable the I2C peripheral
    HAL_I2C_DeInit(i2cHandle);

    // Deinitialize GPIO pins for I2C SCL and SDA
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7); // Assuming SCL and SDA pins are on GPIOB

    // Disable clock for I2C peripheral
    __HAL_RCC_I2C1_CLK_DISABLE();
}

HAL_StatusTypeDef vicorSendCommand(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t command, uint8_t *data, uint16_t size) {
    // Start the I2C transaction
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2cHandle, address, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Error occurred during transmission
        return status;
    }

    // Send the data
    status = HAL_I2C_Master_Transmit(i2cHandle, address, data, size, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Error occurred during transmission
        return status;
    }

    return HAL_OK; // Command sent successfully
}

HAL_StatusTypeDef vicorReadResponse(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    // Read response from the device
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(i2cHandle, address, data, size, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Error occurred during reception
        return status;
    }

    return HAL_I2C_Master_Receive(i2cHandle, address, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef sendPageCommand(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t pageData) {
    uint8_t command = 0x00; // Page Command
    return vicorSendCommand(i2cHandle, address, command, &pageData, 1);
}

HAL_StatusTypeDef readCommandData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t command, uint8_t *data, uint16_t size) {
    // Set the appropriate page data byte based on the command
    uint8_t pageData = (command == 0x00 || command == 0x01) ? 0x01 : 0x00; // Set to 0x01 for page commands, 0x00 otherwise
    HAL_StatusTypeDef status = sendPageCommand(i2cHandle, address, pageData);
    if (status != HAL_OK) {
        // Error occurred while setting the page
        return status;
    }

    // Read command data
    return vicorReadResponse(i2cHandle, address, data, size);
}

HAL_StatusTypeDef sendOperationCommand(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t operationData) {
    uint8_t command = 0x01; // OPERATION Command
    return vicorSendCommand(i2cHandle, address, command, &operationData, 1);
}

void printStatus(HAL_StatusTypeDef status, const char *message, uint8_t *data) {
    if (status == HAL_OK) {
        GRCprintf("%s: %x\n", message, *data); // Print the read value
    } else {
        GRCprintf("Error %s: %d\n", message, status); // Print error status
    }
}

HAL_StatusTypeDef otFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x4F, data, size);
	printStatus(status, "reading Overtemperature protection", data);
	return status;

}

HAL_StatusTypeDef otWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x51, data, size);
	printStatus(status, "reading Overtemperature warning", data);
	return status;

}

HAL_StatusTypeDef vinFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x55, data, size);
	printStatus(status, "reading High-voltage-side overvoltage protection", data);
	return status;

}

HAL_StatusTypeDef vinWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x57, data, size);
	printStatus(status, "reading High-voltage-side overvoltage warning", data);
	return status;
}

HAL_StatusTypeDef iinFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x5B, data, size);
	printStatus(status, "reading High-voltage-side overcurrent protection", data);
	return status;

}

HAL_StatusTypeDef iinWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x5D, data, size);
	printStatus(status, "reading High-voltage-side overcurrent warning", data);
	return status;

}

HAL_StatusTypeDef statusByte(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x78, data, size);
	printStatus(status, "reading Summary of faults", data);
	return status;

}

HAL_StatusTypeDef statusWord(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x79, data, size);
	printStatus(status, "reading Summary of fault conditions", data);
	return status;

}

HAL_StatusTypeDef statusIout(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x7B, data, size);
	printStatus(status, "reading Overcurrent fault status", data);
	return status;

}

HAL_StatusTypeDef statusInput(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x7C, data, size);
	printStatus(status, "reading Overvoltage and undervoltage fault status", data);
	return status;

}

HAL_StatusTypeDef statusTemp(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x7D, data, size);
	printStatus(status, "reading Overtemperature and undertemperature fault status", data);
	return status;

}

HAL_StatusTypeDef stausCml(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x7E, data, size);
	printStatus(status, "reading PMBus communication fault", data);
	return status;

}

HAL_StatusTypeDef vinData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x88, data, size);
	printStatus(status, "reading Hi-side voltage", data);
	return status;

}

HAL_StatusTypeDef innData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x89, data, size);
	printStatus(status, "reading Hi-side current", data);
	return status;

}

HAL_StatusTypeDef voutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x8B, data, size);
	printStatus(status, "reading LO-side voltage", data);
	return status;

}

HAL_StatusTypeDef ioutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x8C, data, size);
	printStatus(status, "reading LO-side current", data);
	return status;

}

HAL_StatusTypeDef temp1(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x8D, data, size);
	printStatus(status, "reading Reads internal temperature", data);
	return status;

}

HAL_StatusTypeDef poutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0x96, data, size);
	printStatus(status, "reading LO-side power", data);
	return status;

}

HAL_StatusTypeDef mfrVinMin(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA0, data, size);
	printStatus(status, "reading Minimum rated high side voltage", data);
	return status;

}

HAL_StatusTypeDef mfrVinMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA1, data, size);
	printStatus(status, "reading Maximum rated high side voltage", data);
	return status;

}

HAL_StatusTypeDef VoutMin(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA4, data, size);
	printStatus(status, "reading Minimum rated low side voltage", data);
	return status;

}

HAL_StatusTypeDef VoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA5, data, size);
	printStatus(status, "reading Maximum rated low side voltage", data);
	return status;

}

HAL_StatusTypeDef IoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA6, data, size);
	printStatus(status, "reading Maximum rated low side current", data);
	return status;

}

HAL_StatusTypeDef PoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xA7, data, size);
	printStatus(status, "reading Maximum rated low side power", data);
	return status;

}

HAL_StatusTypeDef kFactor(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xD1, data, size);
	printStatus(status, "reading K factor", data);
	return status;

}

HAL_StatusTypeDef BcmRout(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = readCommandData(i2cHandle, address, 0xD4, data, size);
	printStatus(status, "reading low-voltage side output resistance", data);
	return status;

}

void configureVICORDevice(I2C_HandleTypeDef *i2cHandle) {
    // VICOR initialization
    vicorInit(i2cHandle);

    // Set Page and Operation
    uint8_t pageData = 0x00; // Page 00h
    uint8_t operationData = 0x00; // Operation Data
    sendPageCommand(i2cHandle, ADDRESS, pageData);
    sendOperationCommand(i2cHandle, ADDRESS, operationData);
}

void handleStatus(HAL_StatusTypeDef status, const char *message) {
    if (status != HAL_OK) {
        GRCprintf("Error %s: %d\n", message, status);
    }
}

void performVICOROperations(I2C_HandleTypeDef *i2cHandle) {
    uint8_t data[10];
    HAL_StatusTypeDef status;

    // Read faults & warn
    status = otFaultLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Overtemperature protection Limit");

    status = otWarnLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Overtemperature warning Limit");

    status = vinFaultLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading High-voltage-side overvoltage protection Limit");

    status = vinWarnLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading High-voltage-side overvoltage warning Limit");

    status = iinFaultLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading High-voltage-side overcurrent protection Limit");

    status = iinWarnLimit(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading High-voltage-side overcurrent warning Limit");

    // Read status
    status = statusByte(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Summary of faults");

    status = statusWord(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Summary of fault conditions");

    status = statusIout(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Overcurrent fault status");

    status = statusInput(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Overvoltage and undervoltage fault status");

    status = statusTemp(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Overtemperature and undertemperature fault status");

    status = stausCml(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading PMBus communication fault");

    // Read data
    status = innData(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Hi-side current");

    status = vinData(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Hi-side voltage");

    status = voutData(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading LO-side voltage");

    status = ioutData(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reads LO-side current");

    status = temp1(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reads internal temperature");

    status = poutData(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reads LO-side power");

    status = mfrVinMin(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Minimum rated high side voltage");

    status = mfrVinMax(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Maximum rated high side voltage");

    status = VoutMin(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Minimum rated low side voltage");

    status = VoutMax(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Maximum rated low side voltage");

    status = IoutMax(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Maximum rated low side current");

    status = PoutMax(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading Maximum rated low side power");

    status = kFactor(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading K factor");

    status = BcmRout(i2cHandle, ADDRESS, data, sizeof(data));
    handleStatus(status, "Reading low-voltage side output resistance");
}




