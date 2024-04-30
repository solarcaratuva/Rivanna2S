#include "DriverCANInterface.h"
#include "MotorControllerCANStructs.h"
#include "log.h"

BatteryBoardCANInterface::BatteryBoardCANInterface(PinName rd, PinName td, PinName standby_pin)
    : MainCANInterface(rd, td, standby_pin) {
    can.frequency(250000);
}



