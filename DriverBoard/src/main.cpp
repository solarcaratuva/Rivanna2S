#include "DriverCANInterface.h"
#include "DriverRateLimiter.h"
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

#define LEFT_TURN_IN PA_10
#define RIGHT_TURN_IN PA_9
#define HAZARDS_IN PB_15
#define MECHANICAL_BRAKE_IN PA_8
#define REGEN_IN PB_11
#define THROTTLE_VALUE_IN PB_2
#define CRUISE_CONTROL_IN_POSITIVE PB_13
#define CRUISE_CONTROL_IN_NEGATIVE PB_12
#define CRUISE_CONTROL_IN_NEUTRAL PB_14

// PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
//                                            MAIN_CAN_STBY);
// BPSCANInterface bps_can_interface(BMS_CAN1_RX, BMS_CAN1_TX, BMS_CAN1_STBY);

/*
bool flashHazards, flashLSignal, flashRSignal = false;
bool brakeLightsEnabled = false;
bool regenEnabled = false;
bool rpmPositive = false;
bool reverseEnabled = false;
bool strobeEnabled = false;
Thread signalFlashThread;

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
*/

//Code for testing buttons

DigitalIn brakeLightsSwitch(MECHANICAL_BRAKE_IN);
DigitalIn leftTurnSwitch(LEFT_TURN_IN);
DigitalIn rightTurnSwitch(RIGHT_TURN_IN);
DigitalIn hazardsSwitch(HAZARDS_IN);
DigitalIn regenSwitch(REGEN_IN);
DigitalIn throttleSwitch(THROTTLE_VALUE_IN);
DigitalIn cruiseControlPSwitch(CRUISE_CONTROL_IN_POSITIVE);
DigitalIn cruiseControlNSwitch(CRUISE_CONTROL_IN_NEGATIVE);
DigitalIn cruiseControlEnableSwitch(CRUISE_CONTROL_ENABLE_IN);

while (true){
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

    else {
        log_debug("NOTHING IS ON");
    }

}
/*
const bool LOG_ECU_POWERAUX_COMMANDS = false;
const bool LOG_BPS_PACK_INFORMATION = true;
const bool LOG_BPS_ERROR = false;
const bool LOG_BPS_CELL_VOLTAGE = false;
const bool LOG_BPS_CELL_TEMPERATURE = false;

TokenBucket ecu_motor_token_bucket(1, 1000); //(number of tokens, milliseconds)
TokenBucket ecu_power_aux_token_bucket(1, 1000); //(number of tokens, milliseconds)
TokenBucket solar_current_token_bucket(1, 1000); //(number of tokens, milliseconds)
TokenBucket solar_voltage_token_bucket(1, 1000); //(number of tokens, milliseconds)
TokenBucket solar_temp_token_bucket(1, 2000); //(number of tokens, milliseconds)
TokenBucket solar_photo_token_bucket(1, 2000); //(number of tokens, milliseconds)
TokenBucket motor_controller_power_token_bucket(1, 1000);
TokenBucket motor_controller_drive_token_bucket(1,1000);
TokenBucket bps_pack_token_bucket(1,2000);
TokenBucket bps_cell_voltage_token_bucket(1,2000);
TokenBucket bps_cell_temp_token_bucket(1,2000);
TokenBucket bps_error_token_bucket(1,500);
TokenBucket power_aux_error_token_bucket(1,500);
TokenBucket motor_controller_error_token_bucket(1,500);


//A lot of the outputs are active low. However, this might be confusing to read.

const bool ACTIVELOW_ON = false;
const bool ACTIVELOW_OFF = true;

bool flashHazardsState = false;

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

// get message, send to handle function, handle function sends to pi or drops message


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

        //Send all messages to CAN Interface
        vehicle_can_interface.send(&to_motor);

        //Send to handler to determine whether the message should be sent to pi
        log_debug("Sending to handler ecumotorcommands");
        ecu_motor_token_bucket.handle(&to_motor, ECUMotorCommands_MESSAGE_ID);


        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}


void DriverCANInterface::handle(MotorControllerError *can_struct) {
    log_debug("Sending to handler mcerror");
    motor_controller_error_token_bucket.handle(can_struct, MotorControllerError_MESSAGE_ID);
}

void DriverCANInterface::handle(ECUPowerAuxCommands *can_struct) {
    log_debug("Sending to handler ecupoweraux");
    ecu_power_aux_token_bucket.handle(can_struct, ECUPowerAuxCommands_MESSAGE_ID);
}

void DriverCANInterface::handle(PowerAuxError *can_struct) {
    log_debug("Sending to handler powerauxerror");
    power_aux_error_token_bucket.handle(can_struct, PowerAuxError_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarCurrent *can_struct) {
    log_debug("Sending to handler solarcurrent");
    solar_current_token_bucket.handle(can_struct, SolarCurrent_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarVoltage *can_struct) {
    log_debug("Sending to handler solarvotlage");
    solar_voltage_token_bucket.handle(can_struct, SolarVoltage_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarTemp *can_struct) {
    log_debug("Sending to handler solartemp");
    solar_temp_token_bucket.handle(can_struct, SolarTemp_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarPhoto *can_struct) {
    log_debug("Sending to handler solarphoto");
    solar_photo_token_bucket.handle(can_struct, SolarPhoto_MESSAGE_ID);
}

void DriverCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    rpmPositive = can_struct->motor_rpm > 0;
    log_debug("Sending to handler mcpowerstatus");
    motor_controller_power_token_bucket.handle(can_struct, MotorControllerPowerStatus_MESSAGE_ID);
}

void DriverCANInterface::handle(MotorControllerDriveStatus *can_struct) {
    log_debug("Sending to handler mcdrivestatus");
    motor_controller_drive_token_bucket.handle(can_struct, MotorControllerDriveStatus);
}

void DriverCANInterface::handle(BPSPackInformation *can_struct) {
    log_debug("Sending to handler bpspackinformation");
    motor_controller_drive_token_bucket.handle(can_struct, BPSPackInformation);
}

void DriverCANInterface::handle(BPSCellVoltage *can_struct) {
    log_debug("Sending to handler bpscellvoltage");
    motor_controller_drive_token_bucket.handle(can_struct, BPSCellVoltage);
}

void DriverCANInterface::handle(BPSCellTemperature *can_struct) {
    log_debug("Sending to handler bpscelltemp");
    motor_controller_drive_token_bucket.handle(can_struct, BPSCellTemperature);
}

//Should be sent straight to raspberry pi
void DriverCANInterface::handle(BPSError *can_struct) {
    bms_strobe = can_struct->internal_communications_fault || can_struct-> low_cell_voltage_fault || can_struct->open_wiring_fault || can_struct->current_sensor_fault || can_struct->pack_voltage_sensor_fault || can_struct->thermistor_fault || can_struct->canbus_communications_fault || can_struct->high_voltage_isolation_fault || can_struct->charge_limit_enforcement_fault || can_struct->discharge_limit_enforcement_fault || can_struct->charger_safety_relay_fault || can_struct->internal_thermistor_fault || can_struct->internal_memory_fault;
    log_debug("Sending to handler bpserror");
    bps_error_token_bucket.handle(can_struct, BPSError_MESSAGE_ID);
}
*/