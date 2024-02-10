#ifndef PINDEF_H
#define PINDEF_H

#ifdef TARGET_UVA_SOLAR_CAR

#define FORWARD_REVERSE_IN PB_2 // 18

#define CRUISE_ENABLED PB_14
#define CRUISE_INC PB_13
#define CRUISE_DEC PB_12

#define LEFT_TURN_IN PA_10
#define RIGHT_TURN_IN PA_9
#define HAZARDS_IN PB_15
#define MECHANICAL_BRAKE_IN PA_8
#define REGEN_IN PB_11
#define THROTTLE_VALUE_IN PB_2
#define FORWARD_REVERSE_IN NC  // only one not done

#define LEFT_TURN_OUT PA_2
#define RIGHT_TURN_OUT PA_1
#define BRAKE_LIGHTS_OUT PA_4
#define DRO_OUT PA_0
#define BMS_STROBE_OUT PA_3



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

#define CRUISE_ENABLED PB_14
#define CRUISE_INC PB_13
#define CRUISE_DEC PB_12

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