#pragma once
#include "SignalControl.h"

class SignalFlashHandler{
    private:
    SignalControl& signalControl;
    std::function<void()> sleep;
    std::function<void()> wait;
    bool flashHazards, flashLSignal, flashRSignal, flashBMS;

public:
    SignalFlashHandler(SignalControl& control);

    void set_callbacks(std::function<void()> sleepFunc, std::function<void()> waitFunc);
    void updateState(bool flashHazards, bool flashLSignal, bool flashRSignal, bool flashBMS);
    void handle();
    void loop();
};