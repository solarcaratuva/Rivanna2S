#include "DriverCANInterface.h"
#include "DriverRateLimiter.h"
#include "Printing.h"
#include "ThisThread.h"
#include "ECUCANStructs.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>

#define LOG_LEVEL          LOG_DEBUG
#define MAIN_LOOP_PERIOD   1s
#define MOTOR_LOOP_PERIOD  10ms
#define MOTRO_LOOP_PERIOD  10ms
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

const bool PIN_ON = false;
const bool PIN_OFF = true;

bool flashHazards, flashLSignal, flashRSignal = false;
bool brakeLightsEnabled = false;
bool regenEnabled = false;
bool rpmPositive = false;
bool strobeEnabled = false;
Thread signalFlashThread;
Thread motor_thread;
int RPM = 0;



DigitalOut brake_lights(BRAKE_LIGHTS_OUT);
DigitalOut leftTurnSignal(LEFT_TURN_OUT);
DigitalOut rightTurnSignal(RIGHT_TURN_OUT);
DigitalOut drl(DRL_OUT);
DigitalOut bms_strobe(BMS_STROBE_OUT);

DigitalIn brakeLightsSwitch(MECHANICAL_BRAKE_IN);
DigitalIn leftTurnSwitch(LEFT_TURN_IN);
DigitalIn rightTurnSwitch(RIGHT_TURN_IN);
DigitalIn hazardsSwitch(HAZARDS_IN);
DigitalIn regenSwitch(REGEN_IN);
DigitalIn cruiseControlSwitch(CRUISE_ENABLED);
AnalogIn throttle(THROTTLE_VALUE_IN, 5.0f);

DriverCANInterface vehicle_can_interface(CAN_RX, CAN_TX, CAN_STBY);

ECUMotorCommands to_motor;

const bool LOG_ECU_POWERAUX_COMMANDS = false;
const bool LOG_BPS_PACK_INFORMATION = true;
const bool LOG_BPS_ERROR = false;
const bool LOG_BPS_CELL_VOLTAGE = false;
const bool LOG_BPS_CELL_TEMPERATURE = false;


//Sends one message every second
TokenBucket ecu_motor_token_bucket(1, 1000); //(number of tokens, seconds)
TokenBucket ecu_power_aux_token_bucket(1, 1000);
TokenBucket solar_current_token_bucket(1, 1000);
TokenBucket solar_voltage_token_bucket(1, 1000);
TokenBucket solar_temp_token_bucket(1, 1000);
TokenBucket solar_photo_token_bucket(1, 1000);
TokenBucket motor_controller_power_token_bucket(1, 1000);
TokenBucket motor_controller_drive_token_bucket(1,1000);
TokenBucket bps_pack_token_bucket(1,1000);
TokenBucket bps_cell_voltage_token_bucket(1,1000);
TokenBucket bps_cell_temp_token_bucket(1,1000);
TokenBucket bps_error_token_bucket(1,1000);
TokenBucket power_aux_error_token_bucket(1,1000);
TokenBucket motor_controller_error_token_bucket(1,1000);

CANMessage ecu_power_aux_message;
CANMessage solar_current_message;
CANMessage solar_voltage_message;
CANMessage solar_temp_message;
CANMessage solar_photo_message;
CANMessage motor_controller_power_message;
CANMessage motor_controller_drive_message;
CANMessage bps_pack_message;
CANMessage bps_voltage_message;
CANMessage bps_temp_message;
CANMessage bps_error;
CANMessage power_aux_error;
CANMessage motor_controller_error;

/*
A lot of the outputs are active low. However, this might be confusing to read.
*/
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
    brakeLightsEnabled = brake_lights.read() || (regenEnabled && rpmPositive);
}

void signalFlashHandler() {
    while (true) {
        // Note: Casting from a `DigitalOut` to a `bool` gives the most recently written value
        if (brakeLightsEnabled) {
            rightTurnSignal = PIN_ON;
            leftTurnSignal = PIN_ON;
        } else if (flashHazardsState) {
            bool leftTurnSignalState = leftTurnSignal;
            leftTurnSignal = !leftTurnSignalState;
            rightTurnSignal = !leftTurnSignalState;
        } else if (flashLSignal) {
            leftTurnSignal = !leftTurnSignal;
            rightTurnSignal = PIN_OFF;
        } else if (flashRSignal) {
            leftTurnSignal = PIN_OFF;
            rightTurnSignal = !rightTurnSignal;
        } else {
            leftTurnSignal = PIN_OFF;
            rightTurnSignal = PIN_OFF;
        }
        ThisThread::sleep_for(FLASH_PERIOD);
    }
}

//Moved motor control from main loop (1s) to it's own loop (10ms)
void motor_message_handler(){
    while(true){
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

        //This is if we handle on db side, rn handled on motor side
        // if(cruiseControlSwitch) {
        //     to_motor.throttle = throttleValue; //use CC value from Karthik's
        // } 
        
        to_motor.regen = regenValue;

        to_motor.forward_en = true;
        to_motor.reverse_en = false; 

        to_motor.cruise_control_en = cruiseControlSwitch;
        to_motor.cruise_control_speed = 0; //replace with speed form Karthik's algorithm

        to_motor.motor_on = true;

        //Send all messages to CAN Interface
        vehicle_can_interface.send(&to_motor);

        //Serialize the CAN Message to prepare to send to handler
        CANMessage motor_message;
        to_motor.serialize(&motor_message);

        //Send to handler
        log_debug("Sending to handler ecumotorcommands\n");
        ecu_motor_token_bucket.handle(&motor_message, ECUMotorCommands_MESSAGE_ID);


        ThisThread::sleep_for(MOTOR_LOOP_PERIOD);
    }

}

int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");
    
    motor_thread.start(motor_message_handler);
    signalFlashThread.start(signalFlashHandler);

    drl = PIN_ON;

    while (true) {
        log_debug("Main thread loop");

        read_inputs();

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}


void DriverCANInterface::handle(MotorControllerError *can_struct) {
    //Fixing message that is sent to handler; convert can struct as message using .serialize()

    log_debug("Sending to handler mcerror\n");
    can_struct->serialize(&motor_controller_error);
    motor_controller_error_token_bucket.handle(&motor_controller_error, MotorControllerError_MESSAGE_ID);
}

void DriverCANInterface::handle(ECUPowerAuxCommands *can_struct) {
    log_debug("Sending to handler ecupoweraux\n");
    can_struct->serialize(&ecu_power_aux_message);
    ecu_power_aux_token_bucket.handle(&ecu_power_aux_message, ECUPowerAuxCommands_MESSAGE_ID);
}

void DriverCANInterface::handle(PowerAuxError *can_struct) {
    log_debug("Sending to handler powerauxerror\n");
    can_struct->serialize(&power_aux_error);
    power_aux_error_token_bucket.handle(&power_aux_error, PowerAuxError_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarCurrent *can_struct) {
    log_debug("Sending to handler solarcurrent\n");
    can_struct->serialize(&solar_current_message);
    solar_current_token_bucket.handle(&solar_current_message, SolarCurrent_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarVoltage *can_struct) {
    log_debug("Sending to handler solar voltage\n");
    can_struct->serialize(&solar_voltage_message);
    solar_voltage_token_bucket.handle(&solar_voltage_message, SolarVoltage_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarTemp *can_struct) {
    log_debug("Sending to handler solartemp\n");
    can_struct->serialize(&solar_temp_message);
    solar_temp_token_bucket.handle(&solar_temp_message, SolarTemp_MESSAGE_ID);
}

void DriverCANInterface::handle(SolarPhoto *can_struct) {
    log_debug("Sending to handler solarphoto\n");
    can_struct->serialize(&solar_photo_message);
    solar_photo_token_bucket.handle(&solar_photo_message, SolarPhoto_MESSAGE_ID);
}

void DriverCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    // rpmPositive = can_struct->motor_rpm > 0; 
    RPM = can_struct->motor_rpm; 
    log_debug("Sending to handler mcpowerstatus\n");
    can_struct->serialize(&motor_controller_power_message);
    motor_controller_power_token_bucket.handle(&motor_controller_power_message, MotorControllerPowerStatus_MESSAGE_ID);
}

void DriverCANInterface::handle(MotorControllerDriveStatus *can_struct) {
    log_debug("Sending to handler mcdrivestatus\n");
    can_struct->serialize(&motor_controller_drive_message);
    motor_controller_drive_token_bucket.handle(&motor_controller_drive_message, MotorControllerDriveStatus_MESSAGE_ID);
}

void DriverCANInterface::handle(BPSPackInformation *can_struct) {
    log_debug("Sending to handler bpspackinformation\n");
    can_struct->serialize(&bps_pack_message);
    motor_controller_drive_token_bucket.handle(&bps_pack_message, BPSPackInformation_MESSAGE_ID);
}

void DriverCANInterface::handle(BPSCellVoltage *can_struct) {
    log_debug("Sending to handler bpscellvoltage\n");
    can_struct->serialize(&bps_voltage_message);
    motor_controller_drive_token_bucket.handle(&bps_voltage_message, BPSCellVoltage_MESSAGE_ID);
}

void DriverCANInterface::handle(BPSCellTemperature *can_struct) {
    log_debug("Sending to handler bpscelltemp\n");
    can_struct->serialize(&bps_temp_message);
    motor_controller_drive_token_bucket.handle(&bps_temp_message, BPSCellTemperature_MESSAGE_ID);
}

//Should be sent straight to raspberry pi
void DriverCANInterface::handle(BPSError *can_struct) {
    bms_strobe = can_struct->internal_communications_fault || can_struct-> low_cell_voltage_fault || can_struct->open_wiring_fault || can_struct->current_sensor_fault || can_struct->pack_voltage_sensor_fault || can_struct->thermistor_fault || can_struct->canbus_communications_fault || can_struct->high_voltage_isolation_fault || can_struct->charge_limit_enforcement_fault || can_struct->discharge_limit_enforcement_fault || can_struct->charger_safety_relay_fault || can_struct->internal_thermistor_fault || can_struct->internal_memory_fault;
    log_debug("Sending to handler bpserror\n");
    can_struct->serialize(&bps_error);
    bps_error_token_bucket.handle(&bps_error, BPSError_MESSAGE_ID);
}
