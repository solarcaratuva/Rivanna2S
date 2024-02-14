    #include "TestSignalControl.h"
    #include "SignalFlashHandler.h"

    TestSignalControl::TestSignalControl(){}
    SignalFlashHandler::SignalFlashHandler(TestSignalControl& control) : signalControl(control) {}

    bool TestSignalControl::getFlashLeftSignalValue() const {
        return flashLSignal;
    }

    bool TestSignalControl::getFlashRightSignalValue() const {
        return flashRSignal;
    }

    void TestSignalControl::setFlashLeftSignal(bool enable){
        flashLSignal = enable;
    } 

    void TestSignalControl::setFlashRightSignal(bool enable){
        flashRSignal = enable;
    }

    void SignalFlashHandler::set_callbacks(std::function<void()> sleepFunc, std::function<void()> waitFunc){
        sleep = sleepFunc;
        wait = waitFunc;
    }
    void SignalFlashHandler::updateState(bool fHazards, bool fLSignal, bool fRSignal, bool fBMS){
        flashHazards = fHazards;
        flashLSignal = fLSignal;
        flashRSignal = fRSignal;
        flashBMS = fBMS;

        signalControl.setFlashLeftSignal(flashLSignal);
        signalControl.setFlashRightSignal(flashRSignal);
    }
    void SignalFlashHandler::handle(){
        


        // if (!flashBMS) {
        //     bms_strobe = 0;
        // }
        // if (flashHazards || flashLSignal || flashRSignal || flashBMS) {
        //     if (flashBMS) {
        //         bms_strobe = !bms_strobe;
        //     }
        //     if (flashHazards) {
        //         bool leftTurnSignalState = leftTurnSignal;
        //         leftTurnSignal = !leftTurnSignalState;
        //         rightTurnSignal = !leftTurnSignalState;
        //     } else if (flashLSignal) {
        //         leftTurnSignal = !leftTurnSignal;
        //         rightTurnSignal = false;
        //     } else if (flashRSignal) {
        //         rightTurnSignal = !rightTurnSignal;
        //         leftTurnSignal = false;
        //     } else {
        //         leftTurnSignal = false;
        //         rightTurnSignal = false;
        //     }

        //     this->sleep();
        // } else {
        //     leftTurnSignal = false;
        //     rightTurnSignal = false;
        // }
        // this->wait();
    };