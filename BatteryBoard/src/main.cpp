// #include "BPSCANInterface.h"
// #include "BPSRelayController.h"
#include "DigitalOut.h"
// #include "PowerAuxCANInterface.h"
#include "Printing.h"
#include "ThisThread.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>

#define LOG_LEVEL          LOG_DEBUG
#define MAIN_LOOP_PERIOD   1s
#define ERROR_CHECK_PERIOD 100ms
#define FLASH_PERIOD       500ms
#define PRECHARGE_PAUSE    100ms

DigitalIn aux_input(AUX_PLUS);
DigitalIn dcdc_input(DCDC_PLUS);
DigitalIn fantech_input(FanTech);
DigitalIn contact12_input(CONTACT_12);

DigitalOut mppt_precharge(MPPT_PRECHARGE);
DigitalOut motor_precharge(MOTOR_PRECHARGE);
DigitalOut discharge_enable(DISCHARGE_ENABLE);
DigitalOut charge_enable(CHARGE_ENABLE);

Thread precharge_check;
bool allow_precharge = true;


void start_precharge() { //Enables switch to start precharging
    charge_enable = true;
}

void battery_precharge() { 
    while (true) {
        printf(
            "aux_input: %d; dc_input: %d; fantech_input: %d; contact12_input: %d\n",
            aux_input.read(),
            dcdc_input.read(),
            fantech_input.read(),
            contact12_input.read()
        );

        // int relay_status = charge_enable.read();
        // int vbus_status = vbus.read();

        // if(relay_status && vbus_status && allow_precharge) {
        //     allow_precharge = false;
        //     start_precharge();
        //     continue;
        // }
        // if(!relay_status || !vbus) {
        //     bool dont_allow_charge = false;
        //     chrono::steady_clock::time_point start = chrono::steady_clock::now();
        //     while(chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start).count() < 30) {
        //         if(relay_status || vbus) {
        //             dont_allow_charge = true;
        //             break;
        //         }
        //         ThisThread::sleep_for(PRECHARGE_PAUSE);
        //     }
        //     if(!dont_allow_charge) {
        //         allow_precharge = true;
        //     }
        //     continue;
        // }
    }
    
}


int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    // signalFlashThread.start(signalFlashHandler);
    // peripheral_error_thread.start(peripheral_error_handler);
    precharge_check.start(battery_precharge);

    while (true) {
        log_debug("Main thread loop");

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

// void PowerAuxCANInterface::handle(ECUPowerAuxCommands *can_struct) {
//     can_struct->log(LOG_INFO);

//     brake_lights = can_struct->brake_lights;

//     flashLSignal = can_struct->left_turn_signal;
//     flashRSignal = can_struct->right_turn_signal;
//     flashHazards = can_struct->hazards;

//     signalFlashThread.flags_set(0x1);
// }

// void BPSCANInterface::handle(BPSPackInformation *can_struct) {
//     can_struct->log(LOG_INFO);

//     bps_relay_controller.update_state(can_struct);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSError *can_struct) {
//     can_struct->log(LOG_INFO);

//     bps_relay_controller.update_state(can_struct);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSCellVoltage *can_struct) {
//     can_struct->log(LOG_INFO);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSCellTemperature *can_struct) {
//     can_struct->log(LOG_INFO);

//     vehicle_can_interface.send(can_struct);
// }
