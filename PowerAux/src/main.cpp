#include "DigitalOut.h"
#include "PowerAuxCANInterface.h"
#include "Printing.h"
#include "ThisThread.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>
#include "SignalFlashHandler.h"

#define LOG_LEVEL          LOG_DEBUG
#define MAIN_LOOP_PERIOD   1s
#define ERROR_CHECK_PERIOD 100ms
#define FLASH_PERIOD       500ms

// TODO: add DRO pin to pindef.h and in main() set dro=1

PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
                                           MAIN_CAN_STBY);

bool flashHazards, flashLSignal, flashRSignal, flashBMS = false;
Thread signalFlashThread;

DigitalOut brake_lights(BRAKE_LIGHT_EN);
DigitalOut leftTurnSignal(LEFT_TURN_EN);
DigitalOut rightTurnSignal(RIGHT_TURN_EN);
DigitalOut bms_strobe(BMS_STROBE_EN);
DigitalOut drl(DRL_EN);

SignalFlashHandler signalFlashHandler1(brake_lights, leftTurnSignal, rightTurnSignal, bms_strobe);



void signalFlashHandler() {
    signalFlashHandler1.set_callbacks([&]()
    {
        ThisThread::sleep_for(FLASH_PERIOD);
    }, [&]()
    {
        ThisThread::flags_wait_all(0x1);
    });
    signalFlashHandler1.loop();

}

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
    can_struct->log(LOG_DEBUG);
    if (can_struct->headlights) {
        flashBMS = can_struct->hazards;
        signalFlashThread.flags_set(0x1);

    }

    brake_lights = can_struct->brake_lights;

    flashLSignal = can_struct->left_turn_signal;
    flashRSignal = can_struct->right_turn_signal;
    flashHazards = can_struct->hazards;

    signalFlashHandler1.updateState(flashHazards, flashLSignal, flashRSignal, flashBMS);

    signalFlashThread.flags_set(0x1);
    
}
