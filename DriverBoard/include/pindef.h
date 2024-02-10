#ifndef PINDEF_H
#define PINDEF_H

#ifdef TARGET_UVA_SOLAR_CAR

#define LEFT_TURN_IN PA_10
#define RIGHT_TURN_IN PA_9
#define HAZARDS_IN PB_15
#define MECHANICAL_BRAKE_IN PA_8
#define REGEN_IN PB_11
#define THROTTLE_VALUE_IN PB_2
#define CRUISE_CONTROL_IN_POSITIVE PB_13
#define CRUISE_CONTROL_IN_NEGATIVE PB_12
#define CRUISE_CONTROL_IN_NEUTRAL PB_14

#define CAN_RX             PA_11 // Rx for MCU
#define CAN_TX             PA_12
#define CAN_STBY           PB_10 // Digital Output

#define LEFT_TURN_OUT PA_0 // 8
#define RIGHT_TURN_OUT PA_1 // 9
#define BRAKE_LIGHTS_OUT PB_12 // 26
#define DRO_OUT PB_11 // 25
#define BMS_STROBE_OUT PB_0 // 16
#define THROTTLE PB_13 // 27
#define REVERSE_IN PB_2 // 18

// #define BRAKE_LIGHT_IN PB_13
// #define LEFT_TURN_LIGHT_IN PA_0
// #define RIGHT_TURN_LIGHT_IN PA_1
// #define DRO PB_12
// #define BMS_STROBE PC_4
// #define LEFT_TURN PB_0
// #define RIGHT_TURN PB_15
// #define HAZARDS PC_13
// #define BRAKE PA_9
// #define REGEN PC_6
// #define FORWARD PB_1

#else

#define LEFT_TURN_IN NC
#define RIGHT_TURN_IN NC
#define HAZARDS_IN NC
#define MECHANICAL_BRAKE_IN NC
#define REGEN_IN NC
#define THROTTLE_VALUE_IN NC
#define FORWARD_REVERSE_IN NC

#define LEFT_TURN_OUT NC
#define RIGHT_TURN_OUT NC
#define BRAKE_LIGHTS_OUT NC
#define DRO_OUT NC
#define BMS_STROBE_OUT NC

#endif

#endif