#include "DigitalOut.h"
#include "Printing.h"
#include "ThisThread.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>

class SignalFlashHandler {
public:
    DigitalOut leftTurnSignal;
    DigitalOut rightTurnSignal;
    DigitalOut bms_strobe;
    DigitalOut brake_lights;
    std::function<void()> sleep, wait;
    bool flashHazards, flashLSignal, flashRSignal, flashBMS;
    // DigitalOut can be mocked, to test
    // Dependency injection
    SignalFlashHandler(DigitalOut& brake_lights, DigitalOut& leftTurnSignal, DigitalOut& rightTurnSignal, DigitalOut& bms_strobe);

    void set_callbacks(std::function<void()> sleep, std::function<void()> wait);
    // Called by other thread to update variables
    void updateState(bool flashHazards, bool flashLSignal, bool flashRSignal, bool flashBMS);
    // Handles change in state, will be tested
    void handle();
    // Calls handle method
    void loop();
};