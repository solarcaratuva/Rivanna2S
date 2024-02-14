#include "SignalFlashHandler.h"



SignalFlashHandler::SignalFlashHandler(TestSignalControl& control) : signalControl(control) {}


void SignalFlashHandler::set_callbacks(std::function<void()> sleep, std::function<void()> wait){
    this->sleep = sleep;
    this->wait = wait;

}

void SignalFlashHandler::updateState(bool flashHazards, bool flashLSignal, bool flashRSignal, bool flashBMS){
    this->flashHazards = false;
    this->flashLSignal = false;
    this->flashRSignal = false;
    this->flashBMS = false;
}

void SignalFlashHandler::handle(){
        log_debug("Flash thread");
        if (!flashBMS) {
            bms_strobe = 0;
        }
        if (flashHazards || flashLSignal || flashRSignal || flashBMS) {
            if (flashBMS) {
                bms_strobe = !bms_strobe;
            }
            if (flashHazards) {
                bool leftTurnSignalState = leftTurnSignal;
                leftTurnSignal = !leftTurnSignalState;
                rightTurnSignal = !leftTurnSignalState;
            } else if (flashLSignal) {
                leftTurnSignal = !leftTurnSignal;
                rightTurnSignal = false;
            } else if (flashRSignal) {
                rightTurnSignal = !rightTurnSignal;
                leftTurnSignal = false;
            } else {
                leftTurnSignal = false;
                rightTurnSignal = false;
            }

            this->sleep();
        } else {
            leftTurnSignal = false;
            rightTurnSignal = false;
        }
        this->wait();
}

void SignalFlashHandler::loop(){
    while (1 == 1){
        handle();
    }
}
