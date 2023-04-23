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

/*
Everything needs to be precharged: Both discharge and charge (they are current in opposite directions)
MPPT precharge must be enabled before the motor can be discharged
Motor precharge must be enabled before the motor can be charged
Precharge must happen when the car is turned on and if the discharge/charge have been disabled for some time (30s)
*/

// Charge
DigitalOut mppt_precharge(MPPT_PRECHARGE);
DigitalOut discharge_enable(DISCHARGE_ENABLE);
// Discharge
DigitalOut motor_precharge(MOTOR_PRECHARGE);
DigitalOut charge_enable(CHARGE_ENABLE);

Thread precharge_check;
bool allow_precharge = true;
std::chrono::steady_clock::time_point last_time_since_r1r2_input;

// Enables switch to start precharging
void start_precharge() {
    charge_enable = true;
    mppt_precharge = true;
    motor_precharge = true;
}

void stop_precharge() {
    charge_enable = false;
    mppt_precharge = false;
    motor_precharge = false;
}

void battery_precharge() {
    while (true) {
        printf(
            "aux_input: %d; dc_input: %d; fantech_input: %d; contact12_input: %d; ",
            aux_input.read(),
            dcdc_input.read(),
            fantech_input.read(),
            contact12_input.read()
        );

        printf(
            "mppt_precharge: %d; motor_precharge: %d; discharge_enable: %d; charge_enable: %d\n",
            mppt_precharge.read(),
            motor_precharge.read(),
            discharge_enable.read(),
            charge_enable.read()
        );

        bool contact12_has_input = contact12_input.read();
        bool is_precharging = charge_enable.read();

        if (contact12_has_input || is_precharging) {
            last_time_since_r1r2_input = chrono::steady_clock::now();
        }

        // If we want to allow_precharge, then precharge for 30 seconds before disabling precharge.
        if (allow_precharge) {
            long seconds_since_r1r2_input = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - last_time_since_r1r2_input).count();
            if (seconds_since_r1r2_input > 30) {
                allow_precharge = true;
            }

            if (!is_precharging) {
                // Start precharge
                start_precharge();
            }

            if (!is_precharging && !contact12_has_input) {
                last_time_since_r1r2_input = chrono::steady_clock::now();
            } else {
            }
        }

        ThisThread::sleep_for(100ms);

        // int relay_status = charge_enable.read();
        // int vbus_status = vbus.read();

        // if(relay_status && vbus_status && allow_precharge) {
        //     allow_precharge = false;
        //     start_precharge();
        //     continue;
        // }a
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
