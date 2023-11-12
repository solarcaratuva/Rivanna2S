#include "main.h"
#include "ECUCANInterface.h"
#include "ECUCANStructs.h"
#include "ECUInputReader.h"
#include "Printing.h"
#include "log.h"
#include "pindef.h"
#include <chrono>
#include <math.h>
#include <mbed.h>
#include <rtos.h>


// for token bucket
#include "ECUCANRateLimiter.h/TokenBucket"
#include "algorithms/token_bucket.h"
#include "threads/thread.h"
#include "time/timestamp.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include "message_id_frequency"
using namespace std
// end for token bucket

#define LOG_LEVEL              LOG_DEBUG
#define MAIN_LOOP_PERIOD       1s
#define MOTOR_THREAD_PERIOD    10ms
#define POWERAUX_THREAD_PERIOD 10ms
#define SEND_CAN_MESSAGE_PERIOD 10ms

// Can Interface
ECUCANInterface vehicle_can_interface(CAN_RX, CAN_TX, CAN_STBY);

// Input Reader
ECUInputReader input_reader;

// CAN Messages
ECUMotorCommands to_motor;
ECUPowerAuxCommands to_poweraux;

// Message Sending Threads
Thread motor_thread;
Thread poweraux_thread;
Thread send_can_message_thread;

// Drop or Send Thread
Thread drop_or_send;

int charge_relay_status;

int RPM = 0;

void motor_message_handler() {
    while (true) {
        uint16_t pedalValue = input_reader.readThrottle();
        uint16_t regenValue;    
        uint16_t throttleValue;
        if (input_reader.readRegen() && charge_relay_status) {
            // Tesla mode

            if (pedalValue <= 50) {
                throttleValue = 0;
                regenValue = 79.159 * pow(50 - pedalValue, 0.3);
            } else if (pedalValue < 100) {
                throttleValue = 0;
                regenValue = 0;
            } else {
                throttleValue = -56.27610464 * pow(156 - (pedalValue - 100), 0.3) + 256;
                regenValue = 0;
            }
        } else {
            throttleValue = pedalValue;
            regenValue = 0;
        }
        if (throttleValue > 224) {
            throttleValue = 224;
        }

        to_motor.throttle = throttleValue;
        to_motor.regen = regenValue;
        log_error("R: %d T: %d", regenValue, throttleValue);

        to_motor.forward_en = input_reader.readForwardEn();
        to_motor.reverse_en = input_reader.readReverseEn();
        to_motor.cruise_control_en = 0;
        to_motor.cruise_control_speed = 0;
        to_motor.motor_on = 0;  // TODO: switch this to input reader 

        // Send message
        vehicle_can_interface.send(&to_motor);
        to_motor.log(LOG_DEBUG);

        // Sleep
        ThisThread::sleep_for(MOTOR_THREAD_PERIOD);
    }
}

void poweraux_message_handler() {
    while (true) {
        // Read poweraux commands
        to_poweraux.hazards = input_reader.readHazards();
        to_poweraux.brake_lights = input_reader.readBrakePedal() || (input_reader.readRegen() && RPM > 0); 


        //debug("Brake value is: %d", to_poweraux.brake_lights);
        
        // ECUPowerAuxCommands headlights field  will be set low to differentiate messages from ECU vs BatteryBoard
        to_poweraux.headlights = 0;

        to_poweraux.left_turn_signal = input_reader.readLeftTurnSignal();
        to_poweraux.right_turn_signal = input_reader.readRightTurnSignal();

        // Send message
        //to_poweraux.left_turn_signal = 1;
        //to_poweraux.brake_lights = 1;
        //to_poweraux.hazards = 1;


        vehicle_can_interface.send(&to_poweraux);
        to_poweraux.log(LOG_DEBUG);

        // Sleep
        ThisThread::sleep_for(POWERAUX_THREAD_PERIOD);
    }
}


//Initialize token buckets for each message id from message_id_frequency.txt file - will refer to this for intervals
//Doesn't include error message token buckets - those will be sent straight through
void init_token_buckets() {
    TokenBucket ecu_motor_commands_token_bucket(string "ECUMotorCommands_MESSAGE_ID", int 1, int 0.1);
    TokenBucket motor_controller_drive_status_token_bucket(string "MotorControllerDriveStatus_MESSAGE_ID", int 1, int 0.1);
    TokenBucket motor_controller_drive_status_aux_bus_token_bucket(string "MotorControllerDriveStatus_AUX_BUS_MESSAGE_ID", int 1, int 0.1);
    TokenBucket solar_current_token_bucket(string "SolarCurrent_MESSAGE_ID", int 1, int 1);
    TokenBucket solar_temp_token_bucket(string "SolarTemp_MESSAGE_ID", int 1, int 1);
    TokenBucket solar_voltage_token_bucket(string "SolarVoltage_MESSAGE_ID", int 1, int 1);
    TokenBucket solar_photo_token_bucket(string "SolarPhoto_MESSAGE_ID", int 1, int 1);
    TokenBucket bps_pack_info_token_bucket(string "BPSPackInformation_MESSAGE_ID", int 1, int 1);
    TokenBucket bps_cell_voltage_token_bucket(string "BPSCellVoltage_MESSAGE_ID", int 1, int 1);
    TokenBucket bps_cell_temp_token_bucket(string "BPSCellTemperature_MESSAGE_ID", int 1, int 1);
    TokenBucket motor_controller_power_token_bucket(string "MotorControllerPowerStatus_MESSAGE_ID", int 1, int 2);
    TokenBucket motor_controller_power_aux_bus_token_bucket(string "MotorControllerPowerStatus_AUX_BUS_MESSAGE_ID", int 1, int 2);
    TokenBucket ecu_power_aux_commands_token_bucket(string "ECUPowerAuxCommands_MESSAGE_ID", int 1, int 2);  
}

void drop_or_send_message(){
    init_token_buckets();

    // start bucket with total number of tokens
    // bucket_total = uintTokenTotal
    // if first time, then last_time = 0
    // else last_time = &last_time

    // past_interval_time(last_time, uintTokenRate, &last_time,)
    // if past_interval_time == true and bucket_total < token_total
        // bucket_total = bucket_total + 1
    
    // if tokens present in bucket bucket_total != 0
        // send message through
        // bucket_total = bucket_total - 1
    
    // elif tokens is not present in bucket
        // drop message

    while (true) {
        switch(message.id){
            case ECUMotorCommands_MESSAGE_ID
                break;
            case ECUPowerAuxCommands_MESSAGE_ID
                break;
            case PowerAuxError_MESSAGE_ID
                // send through
                break;
            case SolarCurrent_MESSAGE_ID
                break;
            case SolarTemp_MESSAGE_ID
                break;
            case SolarVoltage_MESSAGE_ID
                break;
            case SolarPhoto_MESSAGE_ID
                break;
            case MotorControllerPowerStatus_MESSAGE_ID or case MotorControllerPowerStatus_AUX_BUS_MESSAGE_ID
                break;
            case MotorControllerDriveStatus_MESSAGE_ID or case MotorControllerDriveStatus_AUX_BUS_MESSAGE_ID
                break;
            case MotorControllerError_MESSAGE_ID or case MotorControllerError_AUX_BUS_MESSAGE_ID
                // send through
                break;
            case BPSPackInformation_MESSAGE_ID
                break; 
            case BPSError_MESSAGE_ID
                // send through
                break;
            case BPSCellVoltage_MESSAGE_ID
                break;
            case BPSCellTemperature_MESSAGE_ID
                break;
        }
    }

    ThisThread::sleep_for(SEND_CAN_MESSAGE_PERIOD);
}


int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    motor_thread.start(motor_message_handler);
    poweraux_thread.start(poweraux_message_handler);
    send_can_message_thread.start(Token_Handler);

    while (true) {
        log_debug("Main thread loop");
 
        // 
        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
        //Thread that constantly pulls CAN Messages
        //Lock can message
        //Thread that determines whether message should be dropped or sent
    }
}

void ECUCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    RPM = can_struct->motor_rpm;
}

void ECUCANInterface::handle(BPSPackInformation *can_struct) {
    charge_relay_status = can_struct->charge_relay_status;
}

void ECUCANInterface::send_to_pi(CANMessage *message, uint16_t message_id) {
    if (uartTX != NC) {
        char message_data[17];
        CANInterface::write_CAN_message_data_to_buffer(message_data,
                                                           message);
        log_debug("Sending test message to PI");
        log_debug("Message id is %d", message->id);
        char data_to_pi[25];
        data_to_pi[0] = 249;
        data_to_pi[1] = message->id / 0x0100;; //% 0x1000;
        data_to_pi[2] = message->id % (0x0100);
        for (int i = 0; i < 17; i++) {
            data_to_pi[i+3] = message_data[i];
        }
        data_to_pi[24] = 250;

        log_debug("Raw UART message %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", data_to_pi[0], data_to_pi[1], data_to_pi[2], data_to_pi[3], data_to_pi[4], data_to_pi[5], data_to_pi[6], data_to_pi[7], data_to_pi[8], data_to_pi[9], data_to_pi[10], data_to_pi[11], data_to_pi[12], data_to_pi[13], data_to_pi[14], data_to_pi[15], data_to_pi[16], data_to_pi[17], data_to_pi[18], data_to_pi[19], data_to_pi[20], data_to_pi[21], data_to_pi[22], data_to_pi[23], data_to_pi[24]);
        static BufferedSerial raspberry_pi(uartTX, uartRX, 9600);
        raspberry_pi.write(data_to_pi, sizeof(data_to_pi));
    }
}

/*
while (true) {
    if message.id == "MOTORECU" {
        // do stuff
        if time_since_1970 % motor_ecu_interval == 0 {
            // send message to pi
        } else {
            last_motor_ecu_message = "message"
        }
    }
}*/

// difference between current code
// this is rate limiting while current code is not, which overwhelms the pi

// difference between hashmap
// hashmap takes up too much memory, for this 

// set hierachy based on message.id -> convert to interval time
// new CAN message ->
// hashmap {message.id, message + /0 + time}
// switch statement for message.id
// lock
// ECUMOTORCMDS: 01010101    17171s
// unlock
// ECUPOWERAUZ: 010101       8383s

// 2nd thread
// look through hashmap and determine if interval has been reached
// set a delay on this thread            

// take out the send to pi stuff and override 


// put can messages here