#ifndef PINDEF_H
#define PINDEF_H

#ifdef TARGET_UVA_SOLAR_CAR

#define LEFT_TURN_IN PB_1 // 17
#define RIGHT_TURN_IN PB_14 // 28
#define HAZARDS_IN PA_2 // 2
#define MECHANICAL_BRAKE_IN PB_15 // 31
#define REGEN_IN PB_15 // 29
#define THROTTLE_VALUE_IN PB_13 // 27
#define FORWARD_REVERSE_IN PB_2 // 18

#define LEFT_TURN_OUT PA_8 // 8
#define RIGHT_TURN_OUT PA_9 // 9
#define BRAKE_LIGHTS_OUT PB_12 // 26
#define DRO_OUT PB_11 // 25
#define BMS_STROBE_OUT PB_0 // 16

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
// #define THROTTLE PB_14

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