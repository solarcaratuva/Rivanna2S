#ifndef DRIVER_CAN_INTERFACE_H
#define DRIVER_CAN_INTERFACE_H

#include "BPSCANStructs.h"
#include "MotorControllerCANStructs.h"
#include "CANInterface.h"
#include <mbed.h>
//#include "pindef.h"

class DriverCANInterface : public CANInterface {
  public:
    DriverCANInterface(PinName rd, PinName td, PinName standby_pin);
    void handle(BPSError *can_struct);
    void handle(MotorControllerPowerStatus *can_struct);
    int send(CANStruct *can_struct);
    void sendPI(PinName recieve, PinName send, int baud, char* ptr, char message_data[17]);


  private:
    //UnbufferedSerial raspberry_pi(PI_RX, PI_TX, 9600);
    void message_handler() override;
};

#endif
