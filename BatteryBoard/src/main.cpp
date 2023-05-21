#include "BPSCANInterface.h"
#include "BatteryBoardCANInterface.h"
#include "ECUCANStructs.h"

#include "DigitalOut.h"
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
#define CHARGE_PAUSE    100ms

DigitalIn aux_input(AUX_PLUS);
DigitalIn dcdc_input(DCDC_PLUS);
DigitalIn fantech_input(FanTech);
DigitalIn contact12_input(CONTACT_12);

/* TODO: verify (/update) the information in these comments and then update the two methods starting at line 63
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


// vehicle_can_interface is the CAN line between all the boards
// bps_can_interface is the CAN line to the BPS
// initializing a can_interface starts a thread that listens for CAN messages

BatteryBoardCANInterface vehicle_can_interface(CAN_RX, CAN_TX, CAN_STBY);
//  ------------------------------------------------------
// TODO: add pin defs to the pindef.h file to specify BPS CAN Pins
BPSCANInterface bps_can_interface(CAN_RX, CAN_TX, CAN_STBY);
//  ------------------------------------------------------

Thread precharge_check;
Thread discharge_check;
bool allow_precharge = true;
bool allow_discharge = true;
std::chrono::steady_clock::time_point last_time_since_r1r2_input;
int charge_relay_status;
int discharge_relay_status;

//  ------------------------------------------------------
// TODO: make sure these methods are correct
// Outputs
// Four pins that are used in this
// mppt_precharge = PB_1
// motor_precharge = PA_7
// charge_enable = PA_5
// discharge_enable = PA_6

// Enables switch to start precharging
void start_precharge() {
    charge_enable.write(1);
    mppt_precharge.write(1);
    motor_precharge.write(1);
}

void start_discharge() {
    discharge_enable.write(1);
    mppt_precharge.write(0);
    motor_precharge.write(0);
}
// ------------------------------------------------------


void battery_precharge() {
    while (true) {
        int contact_status = contact12_input.read();

        // Start precharge if state allows precharge + high voltage at contact12 + charge_relay_status high from BMS
        if(charge_relay_status && contact_status && allow_precharge) {
            allow_precharge = false; // after precharge starts don't restart it
            start_precharge();
            continue;
        }
        if(!charge_relay_status || !contact_status) {
            // wait 30 seconds for charge_relay_status from the BMS to go low OR
            // wait 30 seconds for contact12 to go low

            bool dont_allow_charge = false;
            chrono::steady_clock::time_point start = chrono::steady_clock::now();
            while(chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start).count() < 30) {
                 // TODO check if this should be a && or an ||
                 // if its && that means for precharge to start again only one of the two conditions has to be low for 30s
                if(charge_relay_status && contact_status) {
                    dont_allow_charge = true;
                    break;
                }
                // check if the state has returned to a state where precharge isn't necessary every CHARGE_PAUSE (100ms)
                ThisThread::sleep_for(CHARGE_PAUSE);
            }
            if (!dont_allow_charge) {
                allow_precharge = true;
            }
            continue;
        }
    }
    
}

void battery_discharge() {
    // same logic as the method above
    while (true) {
        int contact_status = contact12_input.read();

        if(discharge_relay_status && contact_status && allow_discharge) {
            allow_discharge = false;
            start_discharge();
            continue;
        }
        if(!discharge_relay_status || !contact_status) {
            bool dont_allow_charge = false;
            chrono::steady_clock::time_point start = chrono::steady_clock::now();
            while(chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start).count() < 30) {
                // TODO: same as above && vs ||
                if(discharge_relay_status && contact_status) {
                    dont_allow_charge = true;
                    break;
                }
                ThisThread::sleep_for(CHARGE_PAUSE);
            }
            if(!dont_allow_charge) {
                allow_discharge = true;
            }
            continue;
        }
    }
    
}

int main() {
    // Starts precharge and discharge threads and stays active
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    precharge_check.start(battery_precharge);
    discharge_check.start(battery_discharge);

    while (true) {
        log_debug("Main thread loop");
        log_debug("aux_input: %d; dc_input: %d; fantech_input: %d; contact12_input: %d; ", aux_input.read(), dcdc_input.read(), fantech_input.read(), contact12_input.read());
        log_debug("mppt_precharge: %d; motor_precharge: %d; discharge_enable: %d; charge_enable: %d\n",  mppt_precharge.read(), motor_precharge.read(), discharge_enable.read(), charge_enable.read());
        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

void BPSCANInterface::handle(BPSPackInformation *can_struct) {
    charge_relay_status = can_struct->charge_relay_status;
    discharge_relay_status = can_struct->discharge_relay_status;
}

void BPSCANInterface::handle(BPSError *can_struct) {
    // Checks critical faults are present and sends ECUPowerAux command to turn on bms strobe
    // ECUPowerAuxCommands headlights field  will only be set high by the BatteryBoard if a fault is present

    int bms_strobe = can_struct->internal_communications_fault || can_struct-> low_cell_voltage_fault || can_struct->open_wiring_fault || can_struct->current_sensor_fault || can_struct->pack_voltage_sensor_fault || can_struct->thermistor_fault || can_struct->canbus_communications_fault || can_struct->high_voltage_isolation_fault || can_struct->charge_limit_enforcement_fault || can_struct->discharge_limit_enforcement_fault || can_struct->charger_safety_relay_fault || can_struct->internal_thermistor_fault || can_struct->internal_memory_fault;
    
    if (bms_strobe) {
        ECUPowerAuxCommands x;
        x.headlights = bms_strobe; 
        vehicle_can_interface.send(&x);
    }
}

void BPSCANInterface::message_forwarder(CANMessage *message) {
    // message_forwarder is called whenever the BPSCANInterface gets a CAN message
    // this forwards the message to the vehicle can bus
    vehicle_can_interface.send_message(message);
}

