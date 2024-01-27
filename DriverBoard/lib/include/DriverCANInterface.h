#ifndef DRIVER_CAN_INTERFACE_H
#define DRIVER_CAN_INTERFACE_H

#include "BPSCANStructs.h"
#include "MotorControllerCANStructs.h"
#include "CANInterface.h"

class DriverCANInterface : public MainCANInterface {
  public:
    DriverCANInterface(PinName rd, PinName td, PinName standby_pin);
    void handle(ECUMotorCommands *can_struct);
    void handle(ECUPowerAuxCommands *can_struct);
    void handle(PowerAuxError *can_struct);
    void handle(SolarCurrent *can_struct);
    void handle(SolarVoltage *can_struct);
    void handle(SolarTemp *can_struct);
    void handle(SolarPhoto *can_struct);
    void handle(MotorControllerPowerStatus *can_struct);
    void handle(MotorControllerDriveStatus *can_struct);
    void handle(MotorControllerError *can_struct);
    void handle(BPSPackInformation *can_struct);
    void handle(BPSError *can_struct);
    void handle(BPSCellVoltage *can_struct);
    void handle(BPSCellTemperature *can_struct);

    int send(CANStruct *can_struct);

  private:
    void message_handler() override;
};

#endif
