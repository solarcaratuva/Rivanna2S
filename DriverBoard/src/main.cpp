#include "DriverCANInterface.h"
#include "Printing.h"
#include "ThisThread.h"
#include "ECUCANStructs.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>


#define LOG_LEVEL          LOG_INFO
#define MAIN_LOOP_PERIOD   1s
#define ERROR_CHECK_PERIOD 100ms
#define FLASH_PERIOD       500ms
#define IDLE_PERIOD        100ms
#define THROTTLE_LOW_VOLTAGE         0.66
#define THROTTLE_LOW_VOLTAGE_BUFFER  0.20
#define THROTTLE_HIGH_VOLTAGE        3.08
#define THROTTLE_HIGH_VOLTAGE_BUFFER 0.10


// PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
//                                            MAIN_CAN_STBY);
// BPSCANInterface bps_can_interface(BMS_CAN1_RX, BMS_CAN1_TX, BMS_CAN1_STBY);


//Current Pin Values
#define LEFT_TURN_IN PA_10
#define RIGHT_TURN_IN PA_9
#define HAZARDS_IN PB_15
#define MECHANICAL_BRAKE_IN PA_8
#define REGEN_IN PB_11
#define THROTTLE_VALUE_IN PB_2
#define CRUISE_CONTROL_IN_POSITIVE PB_13
#define CRUISE_CONTROL_IN_NEGATIVE PB_12
#define CRUISE_CONTROL_IN_NEUTRAL PB_14

#define RIGHT_TURN_OUT PA_1
#define LEFT_TURN_OUT PA_2
#define BMS_STROBE_OUT PA_3
#define BRAKE_LIGHTS_OUT PA_4


//Old read in 
/*
DigitalIn brakeLightsSwitch(MECHANICAL_BRAKE_IN);
DigitalIn leftTurnSwitch(LEFT_TURN_IN);
DigitalIn rightTurnSwitch(RIGHT_TURN_IN);
DigitalIn hazardsSwitch(HAZARDS_IN);
DigitalIn regenSwitch(REGEN_IN);
DigitalIn throttleSwitch(THROTTLE_VALUE_IN);
DigitalIn cruiseControlPSwitch(CRUISE_CONTROL_IN_POSITIVE);
DigitalIn cruiseControlNSwitch(CRUISE_CONTROL_IN_NEGATIVE);
DigitalIn cruiseControlEnableSwitch(CRUISE_CONTROL_IN_NEUTRAL);

DigitalIn rightTurnOut(RIGHT_TURN_OUT);
DigitalIn leftTurnOut(LEFT_TURN_OUT);
DigitalIn bmsStrobeOut(BMS_STROBE_OUT);
DigitalIn brakeLightOut(BRAKE_LIGHT_OUT);
*/

//Can't switch positive and negative cruise control is cruise control isn't on



bool flashHazards, flashLSignal, flashRSignal = false;
bool brakeLightsEnabled = false;
bool regenEnabled = false;
bool rpmPositive = false;
bool reverseEnabled = false;
bool strobeEnabled = false;
Thread signalFlashThread;

bool flashHazardsState = false;

const bool ACTIVELOW_ON = true;
const bool ACTIVELOW_OFF = false;

DigitalOut brake_lights(BRAKE_LIGHTS_OUT);
DigitalOut leftTurnSignal(LEFT_TURN_OUT);
DigitalOut rightTurnSignal(RIGHT_TURN_OUT);
DigitalOut dro(DRO_OUT);
DigitalOut bms_strobe(BMS_STROBE_OUT);

DigitalIn brakeLightsSwitch(MECHANICAL_BRAKE_IN);
DigitalIn leftTurnSwitch(LEFT_TURN_IN);
DigitalIn rightTurnSwitch(RIGHT_TURN_IN);
DigitalIn hazardsSwitch(HAZARDS_IN);
DigitalIn regenSwitch(REGEN_IN);
DigitalIn reverseSwitch(REVERSE_IN);
AnalogIn throttle(THROTTLE, 5.0f);


DriverCANInterface vehicle_can_interface(CAN_RX, CAN_TX, CAN_STBY);

const bool LOG_ECU_POWERAUX_COMMANDS = false;
const bool LOG_BPS_PACK_INFORMATION = true;
const bool LOG_BPS_ERROR = false;
const bool LOG_BPS_CELL_VOLTAGE = false;
const bool LOG_BPS_CELL_TEMPERATURE = false;


//A lot of the outputs are active low. However, this might be confusing to read.


uint16_t readThrottle() {
    float adjusted_throttle_input =
        ((throttle.read_voltage() - THROTTLE_LOW_VOLTAGE -
          THROTTLE_LOW_VOLTAGE_BUFFER) /
         (THROTTLE_HIGH_VOLTAGE - THROTTLE_HIGH_VOLTAGE_BUFFER -
          THROTTLE_LOW_VOLTAGE - THROTTLE_LOW_VOLTAGE_BUFFER));
    if (adjusted_throttle_input <= 0.0f) {
        return 0;
    } else if (adjusted_throttle_input >= 1.0f) {
        return 256;
    } else {
        return (uint16_t)(adjusted_throttle_input * 256.0);
    }
}

void read_inputs() {
    flashHazards = hazardsSwitch.read();
    if(flashHazards) {
        flashHazardsState = !flashHazardsState;
    }
    flashLSignal = leftTurnSwitch.read();
    flashRSignal = rightTurnSwitch.read();
    regenEnabled = regenSwitch.read();
    reverseEnabled = reverseSwitch.read();
    brakeLightsEnabled = brake_lights.read() || (regenEnabled && rpmPositive);
}

void signalFlashHandler() {
    while (true) {
        // Note: Casting from a `DigitalOut` to a `bool` gives the most recently written value
        if (brakeLightsEnabled) {
            rightTurnSignal = ACTIVELOW_ON;
            leftTurnSignal = ACTIVELOW_ON;
        } else if (flashHazardsState) {
            bool leftTurnSignalState = leftTurnSignal;
            leftTurnSignal = !leftTurnSignalState;
            rightTurnSignal = !leftTurnSignalState;
        } else if (flashLSignal) {
            leftTurnSignal = !leftTurnSignal;
            rightTurnSignal = ACTIVELOW_OFF;
        } else {
            leftTurnSignal = ACTIVELOW_OFF;
            rightTurnSignal = ACTIVELOW_OFF;
        }
        ThisThread::sleep_for(FLASH_PERIOD);
    }
}

/*
int main() {
    signalFlashThread.start(signalFlashHandler);
    log_debug("Start main()");
while (true){
    read_inputs();

    //Testing Lights on Top Hood
    if (rightTurnOut){
        
        rightTurnSignal = !leftTurnSignalState;
        log_debug("-------Right Lights Out On-------");
    }
    if (leftTurnOut){
        leftTurnSignal = !leftTurnSignalState;
        log_debug("-------Left Lights Out On-------");
    }
    if (bmsStrobeOut){
        log_debug("-------BMS Strobe Out On-------");
    }
    if (brakeLightOut){
        log_debug("-------Brake Lights Out On-------");
    }

    //Testing Buttons

    if (brakeLightsSwitch){
        log_debug("-------Brake Lights On-------");
    }

    if (leftTurnSwitch){
        log_debug("-------Left Turn Lights On-------");
    }

    if (rightTurnSwitch){
        log_debug("-------Right Turn Lights On-------");
    }

    if (hazardsSwitch){
        log_debug("-------Hazards Lights On-------");
    }

    if (regenSwitch){
        log_debug("-------Regen Switch On-------");
    }

    if (throttleSwitch){
        log_debug("-------Throttle Switch On-------");
    }      

    if (cruiseControlEnableSwitch){
        log_debug("-------Cruise Control Switch On-------");
    } 

    if (cruiseControlPSwitch){
        log_debug("-------Cruise Control is Positive-------");
    } 

    if (cruiseControlNSwitch){
        log_debug("-------Cruise Control is Negative-------");
    }      

    //else {
        //log_debug("NOTHING IS ON");
    //}

}
}
*/

int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    signalFlashThread.start(signalFlashHandler);

    dro = ACTIVELOW_ON;


    while (true) {
        log_debug("Main thread loop");
        read_inputs();

        ECUMotorCommands to_motor;

        uint16_t pedalValue = readThrottle();
        uint16_t regenValue;    
        uint16_t throttleValue;

        if (regenEnabled) {
            // One pedal drive (tesla style)
            if (pedalValue <= 50) {
                throttleValue = 0;
                regenValue = 79.159 * pow(50-pedalValue, 0.3);
            } else if (pedalValue < 100) {
                throttleValue = 0;
                regenValue = 0;
            } else {
                throttleValue = -56.27610464*pow(156-(pedalValue-100),0.3) + 256;
                regenValue = 0;
            }
        } else {
            throttleValue = pedalValue;
            regenValue = 0;
        }
      
        to_motor.throttle = throttleValue;
        to_motor.regen = regenValue;

        to_motor.forward_en = !reverseEnabled;
        to_motor.reverse_en = reverseEnabled; 

        to_motor.motor_on = true;
        vehicle_can_interface.send(&to_motor);

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

void DriverCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    rpmPositive = can_struct->motor_rpm > 0;
}
void DriverCANInterface::handle(BPSError *can_struct) {
    bms_strobe = can_struct->internal_communications_fault || can_struct-> low_cell_voltage_fault || can_struct->open_wiring_fault || can_struct->current_sensor_fault || can_struct->pack_voltage_sensor_fault || can_struct->thermistor_fault || can_struct->canbus_communications_fault || can_struct->high_voltage_isolation_fault || can_struct->charge_limit_enforcement_fault || can_struct->discharge_limit_enforcement_fault || can_struct->charger_safety_relay_fault || can_struct->internal_thermistor_fault || can_struct->internal_memory_fault;
}
