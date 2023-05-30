#ifndef ECU_CAN_INTERFACE_H
#define ECU_CAN_INTERFACE_H

#include "MainCANInterface.h"
#include "pindef.h"
// TODO: change the NC, NC in the instantiation of MainCANInteface to rx and tx pins for serial 
class ECUCANInterface : public MainCANInterface {
  public:
    ECUCANInterface(PinName rd, PinName td, PinName standby_pin)
        : MainCANInterface(rd, td, standby_pin, PI_UART_RX, PI_UART_TX) {}
    
    void handle(MotorControllerPowerStatus *can_struct) override;
};

#endif
