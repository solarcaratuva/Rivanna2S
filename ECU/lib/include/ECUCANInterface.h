#ifndef ECU_CAN_INTERFACE_H
#define ECU_CAN_INTERFACE_H

#include "MainCANInterface.h"
// TODO: change the NC, NC in the instantiation of MainCANInteface to rx and tx pins for serial 
class ECUCANInterface : public MainCANInterface {
  public:
    ECUCANInterface(PinName rd, PinName td, PinName standby_pin)
        : MainCANInterface(rd, td, standby_pin, pi_rx, pi_tx) {}
    
    void handle(MotorControllerPowerStatus *can_struct) override;
};

#endif
