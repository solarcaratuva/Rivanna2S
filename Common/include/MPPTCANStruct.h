#include "CANStruct.h"
#include "dbc/structs/rivanna2.h"
#include "dbc/structs/mppt.h"
#include "log.h"

#ifndef MPPT_CAN_INTERFACE_H
#define MPPT_CAN_INTERFACE_H

typedef struct MPPT180VoltageAndCurrent
    : CANStruct,
    rivanna2_mppt_commands_180_t { 

        void deserialize(CANMessage *message) {
        rivanna2_mppt_commands_180_unpack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_180_LENGTH);
        }

        uint32_t get_message_ID() {
            return RIVANNA2_MPPT_COMMANDS_180_ID;
        }

    } MPPT180VoltageAndCurrent;



typedef struct MPPT280VoltageAndPower
    : CANStruct,
    rivanna2_mppt_commands_280_t {

        void deserialize(CANMessage *message) {
        rivanna2_mppt_commands_280_unpack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_280_LENGTH);
        }
       
        uint32_t get_message_ID() {
            return RIVANNA2_MPPT_COMMANDS_280_ID;
        }

    } MPPT280VoltageAndPower;




typedef struct MPPT480Temperatures
    : CANStruct,
    rivanna2_mppt_commands_480_t {

        void deserialize(CANMessage *message) {
        rivanna2_mppt_commands_480_unpack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_480_LENGTH);
        }  

        uint32_t get_message_ID() {
            return RIVANNA2_MPPT_COMMANDS_480_ID;
        }

    } MPPT480Temperatures;



#endif