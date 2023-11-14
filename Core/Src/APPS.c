/*
 * APPS.c
 *
 *  Created on: Jan 19, 2023
 *      Author: Matt and Ian McKechnie
 */

#include <CAN.h>
#include "APPS.h"
#include "utils.h"
#include "control.h"
#include <string.h>

#define AVG_WINDOW			3
#define APPS1_MIN 			410
#define APPS1_MAX			1230
#define APPS2_MIN 			410
#define APPS2_MAX			1230


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

int16_t interpolate(int16_t xdiff, int16_t ydiff, int16_t yoffset, int16_t xoffset_from_x1) {
	// Interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
	return yoffset + xoffset_from_x1 * ydiff / xdiff;
}

APPS_Data_Struct APPS_Data = {
	.torque = 0,
	.pedalPos = 0,
	.flags = APPS_RTD_INVALID | APPS_BSPC_INVALID // Start in invalid states, let system sort itself out
};
// Columns are RPM in increments of 500 (0-6500), Rows are pedal percent in increments of 10% (0-100%)
Torque_Map_Struct Torque_Map_Data = { {
    { -6, -20, -22, -22, -22, -22, -22, -22, -22, -22, -21, -17, -19, -22 },
    { 12, -5, -15, -14, -11, -8, -7, -8, -6, -8, -7, -4, -12, -20 },
    { 27, 15, 5, 7, 5, 5, 5, 5, 5, 4, 5, 5, -6, -18 },
    { 44, 37, 26, 23, 24, 23, 22, 21, 24, 23, 20, 18, 1, -16 },
    { 67, 57, 49, 49, 48, 51, 48, 50, 42, 47, 41, 46, 16, -14 },
    { 83, 78, 72, 73, 72, 72, 69, 64, 63, 70, 62, 55, 21, -12 },
    { 96, 100, 94, 93, 97, 95, 98, 90, 91, 85, 80, 75, 33, -10 },
    { 109, 109, 108, 111, 107, 110, 109, 107, 105, 100, 88, 80, 36, -8 },
    { 117, 116, 116, 123, 121, 125, 118, 115, 111, 105, 88, 88, 41, -6 },
    { 126, 125, 124, 127, 126, 127, 123, 120, 115, 106, 104, 88, 42, -4 },
    { 129, 130, 130, 129, 129, 130, 130, 130, 120, 108, 97, 86, 42, -2 }
}, {
    {-6, -20, -22, -22, -22, -22, -22, -22, -22, -22, -21, -17, -19, -22},
    {12, -5, -15, -14, -11, -8, -7, -8, -6, -8, -7, -4, -11, -17},
    {27, 15, 5, 7, 5, 5, 5, 5, 5, 4, 5, 5, -3, -12},
    {44, 37, 26, 23, 24, 23, 22, 21, 24, 23, 20, 18, 5, -7},
    {67, 57, 49, 49, 48, 51, 48, 50, 42, 47, 41, 46, 22, -2},
    {83, 78, 72, 73, 72, 72, 69, 64, 63, 70, 62, 55, 27, 0},
    {96, 100, 94, 93, 97, 95, 98, 90, 91, 85, 80, 75, 38, 0},
    {109, 109, 108, 111, 107, 110, 109, 107, 105, 100, 88, 80, 40, 0},
    {117, 116, 116, 123, 121, 125, 118, 115, 111, 105, 88, 88, 44, 0},
    {126, 125, 124, 127, 126, 127, 123, 120, 115, 106, 104, 88, 44, 0},
    {129, 130, 130, 129, 129, 130, 130, 130, 120, 108, 97, 86, 43, 0}
}, Torque_Map_Data.map2 };

// Buffer for DMA
volatile uint16_t ADC1_buff[ADC1_BUFF_LEN] = {};



// TODO: T.4.2.5
// TODO: T.4.3.3
void startAPPSTask() {
	// Circular buffer of previous results of each apps signal
	uint8_t circBuffPos = 0;
	uint32_t apps1PrevMesurments[AVG_WINDOW] = {};
	uint32_t apps2PrevMesurments[AVG_WINDOW] = {};


	uint32_t tick = osKernelGetTickCount();

	while (1) {
		// Averages all samples in entire DMA buffer
		uint32_t DMAAvg[ADC1_CHANNELS] = {};
		for (int i = 0; i < ADC1_BUFF_LEN;) {
			for (int32_t chan = 0; chan < ADC1_CHANNELS; chan++) {
				DMAAvg[chan] += ADC1_buff[i++];
			}
		}
		for (int32_t chan = 0; chan < ADC1_CHANNELS; chan++) {
			DMAAvg[chan] /= ADC1_CHANNEL_BUFF_LEN;
		}

		uint32_t apps1AvgDMA = DMAAvg[0];
		uint32_t apps2AvgDMA = DMAAvg[1];

		//Calculates moving average of previous measurements
		if(++circBuffPos == AVG_WINDOW){
			circBuffPos = 0;
		}

		//Circular for moving average
		apps1PrevMesurments[circBuffPos] = apps1AvgDMA;
		apps2PrevMesurments[circBuffPos] = apps2AvgDMA;

		uint32_t apps1Avg = 0;
		uint32_t apps2Avg = 0;
		for (int i = 0; i < AVG_WINDOW; i++) {
			apps1Avg += apps1PrevMesurments[i];
			apps2Avg += apps2PrevMesurments[i];
		}

		//Moving average of raw analog value
		apps1Avg = apps1Avg/AVG_WINDOW;
		apps2Avg = apps2Avg/AVG_WINDOW;

		// RULE (2023 V2): T.4.2.10 Sensor out of defined range
		if (apps1Avg < APPS1_MIN || apps1Avg > APPS1_MAX || apps2Avg < APPS2_MIN || apps2Avg > APPS2_MAX) {
			if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
				// The rules don't state a way for the sensors to recover from this error
				APPS_Data.flags |= APPS_SENSOR_OUT_OF_RANGE_INVALID;
				osMutexRelease(APPS_Data_MtxHandle);
			} else {
				// FIXME: We better catch these mutex misses
				CRITICAL_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): APPS.c:startAPPSTask\n");
			}
		}

		int32_t appsPos1 = (apps1Avg - APPS1_MIN) * 100 /(APPS1_MAX - APPS1_MIN);
		int32_t appsPos2 = (apps2Avg - APPS2_MIN) * 100 /(APPS2_MAX - APPS2_MIN);

		// RULE (2023 V2): T.4.2.4 (Both APPS sensor positions must be within 10% of pedal travel of each other)
		if (ABS(appsPos1 - appsPos2) <= 10) {
			if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
				// The rules don't state a way for the sensors to recover from this error
				APPS_Data.flags |= APPS_SENSOR_CONFLICT_INVALID;
				osMutexRelease(APPS_Data_MtxHandle);
			} else {
				// FIXME: We better catch these mutex misses
				CRITICAL_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): APPS.c:startAPPSTask\n");
			}
		}
		//Moved above print statement to fix declaration error
		int32_t averageAppsPos = (appsPos1 + appsPos2) / 2;

		//Assuming appsPos is supposed to be averageAppsPos, changed to fix error
		DEBUG_PRINT("APPS1:%d, APPS2:%d, APPS_POS:%d\r\n", apps1Avg, apps2Avg, averageAppsPos);

		int32_t appsPos = 0;

		appsPos = MAX(MIN(averageAppsPos, 100),0); // Clamp to between 0-100%
		if (osMutexAcquire(APPS_Data_MtxHandle, 5) == osOK){
			APPS_Data.pedalPos = appsPos;
			osMutexRelease(APPS_Data_MtxHandle);
		} else {
			// FIXME: We better catch these mutex misses
			CRITICAL_PRINT("Missed osMutexAcquire(APPS_Data_MtxHandle): APPS.c:startAPPSTask\n");
		}

		//Used for BSPC
		// TODO: RULE (2023 V2): EV.4.1.3 No regen < 5km/h

		int32_t pedalPercent = MIN(appsPos, 99); // NOTE: Cap values at slightly less then our max % for easier math
		int32_t rpm = 0;

		if(osMutexAcquire(Ctrl_Data_MtxHandle, osWaitForever) == osOK) {
			rpm = MIN(Ctrl_Data.motorSpeed, 6499); // NOTE: Cap values at slightly less then our max rpm for easier math
			osMutexRelease(Ctrl_Data_MtxHandle);
		} else {
			CRITICAL_PRINT("Missed osMutexAcquire(Ctrl_Data_MtxHandle): APPS.c:startAPPSTask\n");
		}


		// Integer division - rounds down (use this to our advantage)
		int32_t pedalOffset = pedalPercent % 10;
		int32_t pedalLowIndex = pedalPercent / 10;
		int32_t pedalHighIndex = pedalLowIndex + 1;
		int32_t rpmOffset = rpm % 500;
		int32_t rpmLowIndex = rpm / 500;
		int32_t rpmHighIndex = rpmLowIndex + 1;
		if (osMutexAcquire(Torque_Map_MtxHandle, osWaitForever) == osOK) {
			// Grab data points early then release mutex immediately.
			// NOTE: because we capped our values, both lower indexes will never read the maximum index
			// this always leaves one column left for the high index.
			int16_t torque_pedallow_rpmlow = Torque_Map_Data.activeMap[pedalLowIndex][rpmLowIndex];
			int16_t torque_pedallow_rpmhigh = Torque_Map_Data.activeMap[pedalLowIndex][rpmHighIndex];
			int16_t torque_pedalhigh_rpmlow = Torque_Map_Data.activeMap[pedalHighIndex][rpmLowIndex];
			int16_t torque_pedalhigh_rpmhigh = Torque_Map_Data.activeMap[pedalHighIndex][rpmHighIndex];

			osMutexRelease(Torque_Map_MtxHandle);

			// Interpolating across rpm values
			int16_t torque_pedallow = interpolate(500, torque_pedallow_rpmhigh - torque_pedallow_rpmlow, torque_pedallow_rpmlow, rpmOffset);
			int16_t torque_pedalhigh = interpolate(500, torque_pedalhigh_rpmhigh - torque_pedalhigh_rpmlow, torque_pedalhigh_rpmlow, rpmOffset);
			int16_t requestedTorque = interpolate(10, torque_pedalhigh - torque_pedallow, torque_pedallow, pedalOffset);

			requestTorque(requestedTorque); // Transmitting scales by 10 due to Limits of motor controller
		} else {
			CRITICAL_PRINT("Missed osMutexAcquire(Torque_Map_MtxHandle): APPS.c:startAPPSTask\n");
		}

		osDelayUntil(tick += APPS_PERIOD);
	}
}

void requestTorque(int16_t requestedTorque) {
	DEBUG_PRINT("Requesting: %dN.m\r\n", requestedTorque);
	requestedTorque *= 10; // Scaling is 10:1, requested torque is what is requested, it needs to be sent as 10 this value
	uint16_t bitwiseRequestedTorque = *(uint16_t*)&requestedTorque;

	// Format is defined in CM200DZ CAN protocol V6.1 section 2.2
	CANTXMsg txMsg;
	txMsg.header.IDE = CAN_ID_STD;
	txMsg.header.RTR = CAN_RTR_DATA;
	txMsg.header.StdId = 0x0C0;
	txMsg.header.DLC = 8;
	txMsg.to = &hcan2;

	// Bytes 0 & 1 is the requested torque
	txMsg.data[0] = bitwiseRequestedTorque & 0xFF;
	txMsg.data[1] = bitwiseRequestedTorque >> 8;

	// Bytes 2 & 3 is the requested RPM (if not in torque mode)
	txMsg.data[2] = 0;
	txMsg.data[3] = 0;

	// Byte 4 is Forward/Reverse
	txMsg.data[4] = 1; // 1 is Forward

	// Byte 5 is Configuration
	txMsg.data[5] = 0;
		// | 0x1 // Inverter Enable
		// | 0x2 // Inverter Discharge
		// | 0x4 // Speed Mode override

	// Byte 6 & 7 sets torque limits
	txMsg.data[6] = 0;
	txMsg.data[7] = 0;

	// Send over CAN2
	osMessageQueuePut(CANTX_QHandle, &txMsg, 0, 5);
}
