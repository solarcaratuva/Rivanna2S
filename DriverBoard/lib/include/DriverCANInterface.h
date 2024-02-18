#ifndef DRIVER_CAN_INTERFACE_H
#define DRIVER_CAN_INTERFACE_H

#include "CANInterface.h"
#include "CANStructs.h"
#include "MainCANInterface.h"

class DriverCANInterface : public MainCANInterface {
  public:
    DriverCANInterface(PinName rd, PinName td, PinName standby_pin);
    void handle(ECUMotorCommands *can_struct) override;
    void handle(ECUPowerAuxCommands *can_struct) override;
    void handle(PowerAuxError *can_struct) override;
    void handle(SolarCurrent *can_struct) override;
    void handle(SolarVoltage *can_struct) override;
    void handle(SolarTemp *can_struct) override;
    void handle(SolarPhoto *can_struct) override;
    void handle(MotorControllerPowerStatus *can_struct) override;
    void handle(MotorControllerDriveStatus *can_struct) override;
    void handle(MotorControllerError *can_struct) override;
    void handle(BPSPackInformation *can_struct) override;
    void handle(BPSError *can_struct) override;
    void handle(BPSCellVoltage *can_struct) override;
    void handle(BPSCellTemperature *can_struct) override;

    int send(CANStruct *can_struct);

    void send_to_pi(CANMessage *message, uint16_t message_id);

  private:
    void message_handler() override;
};

#endif
