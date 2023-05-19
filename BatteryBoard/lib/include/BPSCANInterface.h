#ifndef BPS_CAN_INTERFACE_H
#define BPS_CAN_INTERFACE_H

#include "BPSCANStructs.h"
#include "ECUCANStructs.h"
#include "CANInterface.h"

class BPSCANInterface : public CANInterface {
  public:
    BPSCANInterface(PinName rd, PinName td, PinName standby_pin);
    void handle(ECUMotorCommands *can_struct);
    int send(CANStruct *can_struct);
  private:
    void message_handler() override;
};

#endif
