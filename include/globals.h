#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Preferences.h>
#include <stdint.h>

#define LED_PIN GPIO_NUM_2
#define PUMP_PWM_PIN GPIO_NUM_4
#define IMD_PWM_FALLING_PIN GPIO_NUM_5
#define FLOW_SENS1_PIN GPIO_NUM_12
#define IMD_PWM_RISING_PIN GPIO_NUM_15
#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_17
#define ACCUM_FAN_PIN GPIO_NUM_21
#define AMS_SHUTDOWN_PIN GPIO_NUM_22
#define AIR_CONTACT_PIN GPIO_NUM_23
#define BRAKE_PRESSURE_PIN GPIO_NUM_25
#define BRAKE_LIGHT_PIN GPIO_NUM_26
#define BUZZER_PIN GPIO_NUM_27
#define APPS2_PIN GPIO_NUM_32
#define APPS1_PIN GPIO_NUM_33
#define PUSH_BUTTON_PIN GPIO_NUM_35

// Turn on Pump if motor controller > 40c
#define PUMP_MOTOR_CONTROLLER_TEMP_THRESHOLD 400
#define PUMP_MOTOR_CONTROLLER_MAX_TEMP 800
// Turn on Pump if tractive voltage > 450v
#define PUMP_TRACTIVE_VOLTAGE_THRESHOLD 300
// Turn on Fan if coolant temp > 45c
#define RAD_FAN_COOLANT_TEMP_THRESHOLD 450
// Turn on Accumulator fan if accumulator temp > 40c
#define ACC_FAN_ACC_TEMP_THRESHOLD 400
// Turn off flag when tractive voltage < 20
#define RTD_TRACTIVE_VOLTAGE_OFF 200
// Turn off flag when tractive voltage > 450
#define RTD_TRACTIVE_VOLTAGE_ON 350

#define APPS_BSPC_INVALID 0x1
// RULE (2023 V2): T.4.2.10 Sensor out of defined range
#define APPS_SENSOR_OUT_OF_RANGE_INVALID 0x2
// RULE (2023 V2): T.4.2.4 APPS signals are within 10% of pedal position from each other
#define APPS_SENSOR_CONFLICT_INVALID 0x4
// RULE (2023 V2): T.4.3.4 BSE sensor out of defined range
#define BRAKE_SENSOR_OUT_OF_RANGE_INVALID 0x8

#define CTRL_RTD_INVALID 0x10
#define RTD_BUTTON 0x20
#define BRAKE_SWITCH 0x40
#define PUMP_ACTIVE 0x80
#define ACCUMULATOR_FAN_ACTIVE 0x100
#define RADIATOR_FAN_ACTIVE 0x200
#define AMS_CHATTER_ACTIVE 0x400

// IMD states
#define IMD_SHORT_CIRCUIT 0
#define IMD_NORMAL_CONDITION 1
#define IMD_UNDERVOLTAGE 2
#define IMD_STARTUP 3
#define IMD_DEVICE_ERROR 4
#define IMD_EARTH_FAULT 5

#define READ_ONLY_MODE true
#define READ_WRITE_MODE false

#define DEFAULT_TORQUE 1200           // Torque in Nm * 10
#define DEFAULT_POWER 500             // Power in kW * 10
#define DEFAULT_TARGET_SPEED_LIM 6000 // in RPM
#define SPEED_LIM_RANGE 500           // in RPM
#define REGEN_ENABLED 0
#define DEFAULT_IDLE_PUMP_SPEED 10 // Speed in %

extern Preferences param_storage;

typedef struct {
    int16_t motor_speed;
    int16_t tractive_voltage;
    int16_t motor_controller_temp;
    int16_t coolant_temp;
} MotorController;

extern MotorController global_motor_controller;

typedef struct {
    uint16_t pedal_position;
    uint16_t brake_pressure;
} Peripherals;

extern Peripherals global_peripherals;

typedef struct {
    int16_t max_temp;
    uint16_t DTC1;
    uint16_t DTC2;
    uint32_t last_heartbeat;
} BMS;

extern BMS global_bms;

typedef struct {
    int16_t requested_torque;
    uint32_t flags;
} OutputPeripherals;

typedef struct {
    uint32_t duty_cycle; // in % * 10
    uint32_t frequency;  // in hz * 10
    uint16_t resistance; // in k ohms
    uint8_t state;
} IMD;
extern IMD global_imd;

typedef struct {
    uint32_t flow_rate; // ~L/min * 10
} FlowSensors;
extern FlowSensors global_flow_sensors;

#define FAULTS_ACTIVE(flags) ((flags)&0x1F)

extern OutputPeripherals global_output_peripherals;

#endif // GLOBALS_H