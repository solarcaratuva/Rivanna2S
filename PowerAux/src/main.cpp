#include "DigitalOut.h"
#include "PowerAuxCANInterface.h"
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

// TODO: add DRO pin to pindef.h and in main() set dro=1

PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
                                           MAIN_CAN_STBY);

bool hazardsState, brakeLightsState, headLightsState, leftTurnSignalState, rightTurnSignalState, flashBMS = false;
Thread signalFlashThread;

DigitalOut brake_lights(BRAKE_LIGHT_EN);
DigitalOut leftTurnSignal(LEFT_TURN_EN);
DigitalOut rightTurnSignal(RIGHT_TURN_EN);
DigitalOut bms_strobe(BMS_STROBE_EN);
DigitalOut drl(DRL_EN);

void signalFlashHandler() {
    // Change lights based on can information
    while(true){
        log_debug("Flash Thread");
        if(!flashBMS){
            bms_strobe = 0;
        }
        if(hazardsState || leftTurnSignalState || rightTurnSignalState || flashBMS){
            if(flashBMS){
                bms_strobe = !bms_strobe;
            }
            if(hazardsState){
                bool currLeftSignalState = leftTurnSignal;
                // practically synchronziing the two signal lights
                // its flashing
                leftTurnSignalState = !currLeftSignalState;
                rightTurnSignalState = !currLeftSignalState;  
            } else if(leftTurnSignalState){
                leftTurnSignal = !leftTurnSignal;
                rightTurnSignal = false;
            } else if(rightTurnSignalState){
                rightTurnSignal = !rightTurnSignal;
                leftTurnSignal = false;
            } else{
                leftTurnSignal = false;
                rightTurnSignal = false;
            }
            ThisThread::sleep_for(FLASH_PERIOD);
        } else{
            leftTurnSignal = false;
            rightTurnSignal = false;
        }
        ThisThread::flags_wait_all(0x1);
    }
}

AnalogIn fan_tach(FanTach);
AnalogIn brake_light_current(BRAKE_LIGHT_CURRENT);
AnalogIn headlight_current(DRL_CURRENT);
AnalogIn bms_strobe_current(BMS_STROBE_CURRENT);
AnalogIn left_turn_current(LEFT_TURN_CURRENT);
AnalogIn right_turn_current(RIGHT_TURN_CURRENT);

int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    signalFlashThread.start(signalFlashHandler);
    drl = 1;
    while (true) {
        log_debug("Main thread loop");

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

void PowerAuxCANInterface::handle(ECUPowerAuxCommands *can_struct) {
    // Process Can data
    can_struct->log(LOG_LEVEL);
    if(can_struct->headlights){
        flashBMS = can_struct->hazards;
        signalFlashThread.flags_set(0x1);
        return;
    }
    hazardsState = can_struct->hazards;
    brakeLightsState = can_struct->brake_lights;
    headLightsState = can_struct->headlights;
    leftTurnSignalState = can_struct->left_turn_signal;
    rightTurnSignalState = can_struct->right_turn_signal;
    signalFlashThread.flags_set(0x1);
}