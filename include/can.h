#ifndef CAN_H
#define CAN_H

#include <freertos/task.h>

void initCAN();

void startTransmitCANTask(void* pvParameters);
void startReceiveCANTask(void* pvParameters);

void sendTorque();
void sendState();
void sendPedals();
void sendIMD();
void sendTorqueParameters();

#endif // CAN_H