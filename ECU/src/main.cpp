#include "ECUCANInterface.h"
#include "ECUCANStructs.h"
#include "ECUInputReader.h"
#include "Printing.h"
#include "log.h"
#include "pindef.h"
#include "MainCANInterface.h"
#include "CanInterface.h"
#include <mbed.h>
#include <rtos.h>
#include <math.h>
#include <chrono>

#define LOG_LEVEL              LOG_DEBUG
#define MAIN_LOOP_PERIOD       1s
#define MOTOR_THREAD_PERIOD    10ms
#define POWERAUX_THREAD_PERIOD 10ms

// Can Interface
ECUCANInterface vehicle_can_interface(CAN_RX, CAN_TX, CAN_STBY);

//Serial Thing
UnbufferedSerial raspberry_pi(PI_UART_RX, PI_UART_TX, 9600);


// Input Reader
ECUInputReader input_reader;

// CAN Messages
ECUMotorCommands to_motor;
ECUPowerAuxCommands to_poweraux;

// Message Sending Threads
Thread motor_thread;
Thread poweraux_thread;

int RPM = 0;
int charge_relay_status = 0;

void send_to_pi(CANStruct *can_struct) {
    log_debug("Sending message to PI");
    ThisThread::flags_wait_all(0x1);
    CANMessage message;
    //while (can.read(message)) {
    char message_data[17];        
    message_data[0] = '\0';
    for (int i = 0; i < message.len; i += 1) {
        sprintf(message_data + (i * 2), "%02X", message.data[i]);
    }
    raspberry_pi.write(&message_data, sizeof(message_data));
    //}
}


void motor_message_handler() {
    while (true) {
        uint16_t pedalValue = input_reader.readThrottle();
        uint16_t regenValue;    
        uint16_t throttleValue;
        if (input_reader.readRegen()) {
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

        to_motor.throttle = throttleValue;

        //This checks if the battery pack can charge and if it can't set regen to 0.
        if (charge_relay_status) {
            to_motor.regen = regenValue;
        }
        else {
            to_motor.regen = 0;
        }
        log_error("R: %d T: %d", regenValue, throttleValue);

        to_motor.forward_en = input_reader.readForwardEn();
        to_motor.reverse_en = input_reader.readReverseEn();
        to_motor.cruise_control_en = 0;
        to_motor.cruise_control_speed = 0;
        to_motor.motor_on = 0;  // TODO: switch this to input reader 

        // Send message
        vehicle_can_interface.send(&to_motor);
        send_to_pi(&to_motor);
        // Sleep
        ThisThread::sleep_for(MOTOR_THREAD_PERIOD);
    }
}

void poweraux_message_handler() {
    while (true) {
        // Read poweraux commands
        to_poweraux.hazards = input_reader.readHazards();
        to_poweraux.brake_lights = input_reader.readBrakePedal() || (input_reader.readRegen() && RPM > 0); 

        // ECUPowerAuxCommands headlights field  will be set low to differentiate messages from ECU vs BatteryBoard
        to_poweraux.headlights = 0;

        to_poweraux.left_turn_signal = input_reader.readLeftTurnSignal();
        to_poweraux.right_turn_signal = input_reader.readRightTurnSignal();

        // Send message
        vehicle_can_interface.send(&to_poweraux);
        send_to_pi(&to_poweraux);
        // Sleep
        ThisThread::sleep_for(POWERAUX_THREAD_PERIOD);
    }
}

int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    motor_thread.start(motor_message_handler);
    poweraux_thread.start(poweraux_message_handler);

    while (true) {
        log_debug("Main thread loop");

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

void ECUCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    RPM = can_struct->motor_rpm;
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(BPSPackInformation *can_struct) {
    charge_relay_status = can_struct->charge_relay_status;
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(PowerAuxError *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(MotorControllerDriveStatus *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(MotorControllerError *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(BPSError *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(BPSCellVoltage *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);}

void ECUCANInterface::handle(BPSCellTemperature *can_struct) {
    CANStruct* can = can_struct;
    send_to_pi(can);
}
