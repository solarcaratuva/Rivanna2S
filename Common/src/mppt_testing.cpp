
#include "MPPTCANStruct.h"


// Function to test the initialization and message ID retrieval
void testInitializationAndMessageID() {
    MPPT_CAN_180 mppt180;
    assert(mppt180.get_message_ID() == RIVANNA2_MPPT_COMMANDS_180_ID);
    std::cout << "MPPT_CAN_180 Initialization and Message ID Test Passed" << std::endl;

    MPPT_CAN_280 mppt280;
    assert(mppt280.get_message_ID() == RIVANNA2_MPPT_COMMANDS_280_ID);
    std::cout << "MPPT_CAN_280 Initialization and Message ID Test Passed" << std::endl;

    MPPT_CAN_480 mppt480;
    assert(mppt480.get_message_ID() == RIVANNA2_MPPT_COMMANDS_480_ID);
    std::cout << "MPPT_CAN_480 Initialization and Message ID Test Passed" << std::endl;
}

// Example test function for serialization and deserialization
void testSerializationAndDeserialization() {
    // Test data setup
    MPPT_CAN_180 mppt180;
    // Set some test values
    mppt180.current_in = 1.5f;
    mppt180.voltage_in = 12.0f;

    // Serialize
    CANMessage message;
    mppt180.serialize(&message);

    // Now deserialize into a new struct
    MPPT_CAN_180 mppt180Deserialized;
    mppt180Deserialized.deserialize(&message);

    // Check the values
    assert(mppt180Deserialized.current_in == 1.5f);
    assert(mppt180Deserialized.voltage_in == 12.0f);
    std::cout << "MPPT_CAN_180 Serialization and Deserialization Test Passed" << std::endl;

}

int main() {
    
}






// #include "Common/include/MPPTCANStruct.h"
// #include "Common/include/dbc/structs/mppt.h"

// int main() {
    
//     CANMessage message;
//     while (can.read(message)) {
//         message_forwarder(&message);
        
//         char message_data[17];
//         CANInterface::write_CAN_message_data_to_buffer(message_data,
//                                                         &message);
//         log_debug("Received CAN message with ID 0x%08X Length %d Data 0x%s "
//                     "from MPPT",
//                     message.id, message.len, message_data);

//         if (message.id == RIVANNA2_MPPT_COMMANDS_180_ID) {
//             rivanna2_mppt_commands_280 can_struct;
//             can_struct.deserialize(&message);
//             handle(&can_struct);
//         } else if (message.id ==
//                     RIVANNA2_MPPT_COMMANDS_280_ID) {
//             rivanna2_mppt_commands_280_t can_struct;
//             can_struct.deserialize(&message);
//             handle(&can_struct);
//         } else if (message.id == RIVANNA2_MPPT_COMMANDS_480_ID) {
//             MotorControllerError can_struct;
//             can_struct.deserialize(&message);
//             handle(&can_struct);
//         }
//     }
    
// }

