#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/task.h>

#include "ams.h"
#include "can.h"
#include "globals.h"
#include "peripherals.h"
#include "torque.h"
#include "utils.h"

void setup() {
    Serial.begin(921600);

    initCAN();

    ledcSetup(0, 50, 10); // 50Hz PWM, 10-bit resolution
    pinMode(PUMP_PWM_PIN, OUTPUT);
    ledcAttachPin(PUMP_PWM_PIN, 0); // assign RGB led pins to channels
    pumpCycle(0);

    pinMode(APPS1_PIN, INPUT);
    pinMode(APPS2_PIN, INPUT);
    pinMode(BRAKE_PRESSURE_PIN, INPUT);
    pinMode(PUSH_BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(AMS_SHUTDOWN_PIN, OUTPUT);

    xTaskCreate(startAMSTask, "AMS_TASK", 2048, NULL, 8, NULL);
    Serial.println("Finished creating task 0");
    xTaskCreate(startControlTask, "CONTROL_TASK", 8192, NULL, 2, NULL);
    Serial.println("Finished creating task 1");
    xTaskCreate(startPeripheralTask, "PERIPHERAL_TASK", 8192, NULL, 2, NULL);
    Serial.println("Finished creating task 2");
    xTaskCreate(startTorqueTask, "APPS_TASK", 8192, NULL, 2, NULL);
    Serial.println("Finished creating task 3");
    xTaskCreate(startReceiveCANTask, "CAN_RECEIVE_TASK", 8192, NULL, 3, NULL);
    Serial.println("Finished creating task 4");
    xTaskCreate(startTransmitCANTask, "CAN_TRANSMIT_TASK", 8192, NULL, 5, NULL);
    Serial.println("Finished creating task 5");
}

void loop() {}