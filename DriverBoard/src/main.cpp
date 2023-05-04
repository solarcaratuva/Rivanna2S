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

#define LOG_LEVEL          LOG_INFO
#define MAIN_LOOP_PERIOD   1s
#define ERROR_CHECK_PERIOD 100ms
#define FLASH_PERIOD       500ms
#define IDLE_PERIOD        100ms

// PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
//                                            MAIN_CAN_STBY);
// BPSCANInterface bps_can_interface(BMS_CAN1_RX, BMS_CAN1_TX, BMS_CAN1_STBY);

bool flashLSignal, flashRSignal = false;
bool prevHazardsButtonState = false;
bool isFlashingHazards = false;
bool brakeLightsEnabled = false;
bool regenEnabled = false;
bool rpmPositive = false;
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

const bool LOG_ECU_POWERAUX_COMMANDS = false;
const bool LOG_BPS_PACK_INFORMATION = true;
const bool LOG_BPS_ERROR = false;
const bool LOG_BPS_CELL_VOLTAGE = false;
const bool LOG_BPS_CELL_TEMPERATURE = false;

/*
A lot of the outputs are active low. However, this might be confusing to read.
*/
const bool ACTIVELOW_HIGH = false;
const bool ACTIVELOW_LOW = true;


// Input reading is done separately from flash loop
void read_inputs() {
    bool newHazardState = hazardsSwitch.read();
    bool flashRising = newHazardState && !prevHazardsButtonState;
    if(flashRising) {
        isFlashingHazards = !isFlashingHazards;
    }
    prevHazardsButtonState = newHazardState;
//    if (brake_lights.read()) {
//        brakeLightsEnabled = true;
//    } else {
//        brakeLightsEnabled = false;
//    }
    flashLSignal = leftTurnSwitch.read();
    flashRSignal = rightTurnSwitch.read();
    regenEnabled = regenSwitch.read();
    brakeLightsEnabled = brake_lights.read() || (regenEnabled && rpmPositive);
}

void signalFlashHandler() {
    while (true) {
        read_inputs();
        // Note: Casting from a `DigitalOut` to a `bool` gives the most recently written value
        if (brakeLightsEnabled) {
            rightTurnSignal = ACTIVELOW_HIGH;
            leftTurnSignal = ACTIVELOW_HIGH;
        } else if (isFlashingHazards) {
            bool leftTurnSignalState = leftTurnSignal;
            leftTurnSignal = !leftTurnSignalState;
            rightTurnSignal = !leftTurnSignalState;
        } else if (flashLSignal) {
            leftTurnSignal = !leftTurnSignal;
            rightTurnSignal = ACTIVELOW_LOW;
        } else {
            leftTurnSignal = ACTIVELOW_LOW;
            rightTurnSignal = ACTIVELOW_LOW;
        }
        ThisThread::sleep_for(FLASH_PERIOD);
    }
}


AnalogIn fan_tach(FanTach);
AnalogIn brake_light_current(BRAKE_LIGHT_CURRENT);
AnalogIn headlight_current(DRL_CURRENT);
AnalogIn bms_strobe_current(BMS_STROBE_CURRENT);
AnalogIn left_turn_current(LEFT_TURN_CURRENT);
AnalogIn right_turn_current(RIGHT_TURN_CURRENT);
Thread peripheral_error_thread;

BPSRelayController bps_relay_controller(HORN_EN, DRL_EN, AUX_PLUS,
                                        BMS_STROBE_EN);


// Comment this out for now.

void peripheral_error_handler() {
    PowerAuxError msg;
    while (true) {
        msg.bps_strobe_error = (bms_strobe_current.read_u16() < 1000 &&
                                bps_relay_controller.bps_fault_indicator_on());
        bms_strobe = msg.bps_strobe_error;
        msg.brake_light_error =
            (brake_light_current.read_u16() < 1000 && brake_lights.read());
        msg.fan_error = (fan_tach.read_u16() < 1000);
        msg.left_turn_error =
            (left_turn_current.read_u16() < 1000 && leftTurnSignal.read());
        msg.right_turn_error =
            (right_turn_current.read_u16() < 1000 && rightTurnSignal.read());
        msg.bps_error = bps_relay_controller.bps_has_fault();

        vehicle_can_interface.send(&msg);
        ThisThread::sleep_for(ERROR_CHECK_PERIOD);
    }
}


int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    signalFlashThread.start(signalFlashHandler);
    peripheral_error_thread.start(peripheral_error_handler);

    dro = true;

    while (true) {
        log_debug("Main thread loop");

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

// Comment out CAN-related code for now.

void PowerAuxCANInterface::handle(ECUPowerAuxCommands *can_struct) {
    if (LOG_ECU_POWERAUX_COMMANDS) can_struct->log(LOG_INFO);

    brake_lights = can_struct->brake_lights;

    flashLSignal = can_struct->left_turn_signal;
    flashRSignal = can_struct->right_turn_signal;
    flashHazards = can_struct->hazards;

    signalFlashThread.flags_set(0x1);
}

void PowerAuxCANInterface::handle(MotorControllerPowerStatus *can_struct) {
    can_struct->log(LOG_INFO);
    rpmPositive = can_struc->motor_rpm > 0;
}

void PowerAuxCANInterface::handle(ECUMotorCommands *can_struct) {
    can_struct->log(LOG_INFO);
}

void BPSCANInterface::handle(BPSPackInformation *can_struct) {
    if (LOG_BPS_PACK_INFORMATION) can_struct->log(LOG_INFO);

    bps_relay_controller.update_state(can_struct);

    vehicle_can_interface.send(can_struct);
}

void BPSCANInterface::handle(BPSError *can_struct) {
    if (LOG_BPS_ERROR) can_struct->log(LOG_INFO);

    bps_relay_controller.update_state(can_struct);

    vehicle_can_interface.send(can_struct);
}

void BPSCANInterface::handle(BPSCellVoltage *can_struct) {
    if (LOG_BPS_CELL_VOLTAGE) can_struct->log(LOG_INFO);

    vehicle_can_interface.send(can_struct);
}

void BPSCANInterface::handle(BPSCellTemperature *can_struct) {
    if (LOG_BPS_CELL_TEMPERATURE) can_struct->log(LOG_INFO);

    vehicle_can_interface.send(can_struct);
}