#include "DriverCANInterface.h"
#include "MotorControllerCANStructs.h"
#include "log.h"
#include "pindef.h"

DriverCANInterface::DriverCANInterface(PinName rd, PinName td, PinName standby_pin)
    : CANInterface(rd, td, standby_pin) {
    can.frequency(250000);
    // raspberry_pi.baud(9600);
}

int DriverCANInterface::send(CANStruct *can_struct) {
    CANMessage message;
    can_struct->serialize(&message);
    message.id = can_struct->get_message_ID();
    int result = can.write(message);

    char message_data[17];
    CANInterface::write_CAN_message_data_to_buffer(message_data, &message);
    if (result == 1) {
        log_debug("Sent CAN message with ID 0x%03X Length %d Data 0x%s",
                  message.id, message.len, message_data);
    } else {
        log_error(
            "Failed to send CAN message with ID 0x%03X Length %d Data 0x%s",
            message.id, message.len, message_data);
    }

    return result;
}

void DriverCANInterface::message_handler() {
    while (true) {
        ThisThread::flags_wait_all(0x1);
        CANMessage message;
        while (can.read(message)) {
            char message_data[17];
           
            char* ptr = reinterpret_cast<char*>(&message.id);
            for (size_t i = 0; i < sizeof(uint32_t); ++i) {
                raspberry_pi.putc(ptr[i]);
            }
            for (int i = 0; i < 17; i++) {
                raspberry_pi.putc(message_data[i]);
            }
            raspberry_pi.putc('\n');

            CANInterface::write_CAN_message_data_to_buffer(message_data,
                                                           &message);
            log_debug("Received CAN message with ID 0x%03X Length %d Data 0x%s ", message.id, message.len, message_data);
            if (message.id == BPSError_MESSAGE_ID) {
                BPSError can_struct;
                can_struct.deserialize(&message);
                handle(&can_struct);
            } else if (message.id == MotorControllerPowerStatus_MESSAGE_ID) {
                MotorControllerPowerStatus can_struct;
                can_struct.deserialize(&message);
                handle(&can_struct);
            }
        }
    }
}
