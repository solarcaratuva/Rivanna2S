#include "CANStruct.h"
#include "dbc/structs/rivanna2.h"
#include "dbc/structs/mppt.h"
#include "log.h"

#ifndef MPPT_CAN_INTERFACE_H
#define MPPT_CAN_INTERFACE_H

typedef struct MPPT_CAN_180
    : CANStruct,
    rivanna2_mppt_commands_180_t {

        void serialize(CANMessage *message) {
            rivanna2_mppt_commands_180_pack(
                message->data, this,
                RIVANNA2_MPPT_COMMANDS_180_LENGTH);
            message->len = RIVANNA2_MPPT_COMMANDS_180_LENGTH;
        }

        void deserialize(CANMessage *message) {
            rivanna2_mppt_commands_180_pack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_180_LENGTH);
        }   

        uint32_t get_message_ID() {
            //should be 180 for this MPPT?? or should i make one for all of them
            return RIVANNA2_MPPT_COMMANDS_180_ID;
        }

    } MPPT_CAN_180;

typedef struct MPPT_CAN_280
    : CANStruct,
    rivanna2_mppt_commands_280_t {

        void serialize(CANMessage *message) {
            rivanna2_mppt_commands_280_pack(
                message->data, this,
                RIVANNA2_MPPT_COMMANDS_280_LENGTH);
            message->len = RIVANNA2_MPPT_COMMANDS_280_LENGTH;
        }

        void deserialize(CANMessage *message) {
            rivanna2_mppt_commands_280_pack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_280_LENGTH);
        }   

        uint32_t get_message_ID() {
            //should be 280 for this MPPT
            return RIVANNA2_MPPT_COMMANDS_280_ID;
        }

    } MPPT_CAN_280;

typedef struct MPPT_CAN_480
    : CANStruct,
    rivanna2_mppt_commands_480_t {

        void serialize(CANMessage *message) {
            rivanna2_mppt_commands_280_pack(
                message->data, this,
                RIVANNA2_MPPT_COMMANDS_480_LENGTH);
            message->len = RIVANNA2_MPPT_COMMANDS_480_LENGTH;
        }

        void deserialize(CANMessage *message) {
            rivanna2_mppt_commands_480_pack(
            this, message->data,
            RIVANNA2_MPPT_COMMANDS_480_LENGTH);
        }   

        uint32_t get_message_ID() {
            //should be 280 for this MPPT
            return RIVANNA2_MPPT_COMMANDS_480_ID;
        }

    } MPPT_CAN_480;





#endif