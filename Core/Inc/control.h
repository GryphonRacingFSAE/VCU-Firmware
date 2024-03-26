/*
 * control.h
 *
 *  Created on: Jan 20, 2023
 *      Author: Matt
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2c.h"
#include "stm32f7xx_hal_i2c_ex.h"


// Turn on Pump if motor controller > 40c
#define PUMP_MOTOR_CONTROLLER_TEMP_THRESHOLD 400
// Turn on Pump if tractive voltage > 450v
#define PUMP_TRACTIVE_VOLTAGE_THRESHOLD 4500
// Turn on Fan if coolant temp > 40c
#define RAD_FAN_COOLANT_TEMP_THRESHOLD 400
// Turn on Accumulator fan if accumulator temp > 40c
#define ACC_FAN_ACC_TEMP_THRESHOLD 400
// Turn off flag when tractive voltage < 20
#define RTD_TRACTIVE_VOLTAGE_OFF 200
// Turn off flag when tractive voltage > 450
#define RTD_TRACTIVE_VOLTAGE_ON 4500

typedef struct {
	uint32_t wheelSpeed[4];
	int32_t motorControllerTemp; // 10:1 conversion
	int32_t accumulatorMaxTemp; // 10:1 conversion?
	int32_t coolantTemp; // 10:1 conversion
	int32_t tractiveVoltage; // 10:1 conversion
	int32_t motorSpeed; // 1:1
} Ctrl_Data_Struct;

extern Ctrl_Data_Struct Ctrl_Data;
extern osMutexId_t Ctrl_Data_MtxHandle;

void startControlTask();

void BSPC(); // Brake system plausibility check
void RTD(); // Ready to drive
void pumpCtrl(); // Motor & Motor controller cooling pump control
void fanCtrl(); // Accumulator cooling fan control
void LEDCtrl(); // Info



/*
 * DEFINES
 */
#define ADDRESS 0x51


/*
 * FUNCTION PROTOTYPES
 */
void vicorInit(I2C_HandleTypeDef *i2cHandle);
void vicorDeInit(I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef vicorSendCommand(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t command, uint8_t *data, uint16_t size);
HAL_StatusTypeDef vicorReadResponse(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef otFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef otWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef vinFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef vinWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef iinFaultLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef iinWarnLimit(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef statusByte(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef statusWord(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef statusIout(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef statusInput(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef statusTemp(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef stausCml(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef vinData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef innData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef voutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef ioutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef temp1(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef poutData(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef mfrVinMin(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef mfrVinMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef VoutMin(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef VoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef IoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef PoutMax(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef kFactor(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef BcmRout(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint8_t *data, uint16_t size);
void performVICOROperations(I2C_HandleTypeDef *i2cHandle);
void configureVICORDevice(I2C_HandleTypeDef *i2cHandle);
void handleStatus(HAL_StatusTypeDef status, const char *message);

#endif /* INC_CONTROL_H_ */
