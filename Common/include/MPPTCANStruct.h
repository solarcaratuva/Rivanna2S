#include "CANStruct.h"
#include "dbc/structs/rivanna2.h"
#include "dbc/structs/mppt.h"
#include "log.h"

#ifndef MPPT_CAN_INTERFACE_H
#define MPPT_CAN_INTERFACE_H

typedef struct mppt_can_180_currIn_voltIn
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

    } mppt_can_180_curr_and_volt_in;

typedef struct mppt_can_280_voltOut_powerIn
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

    } mppt_can_280_voltOut_powerIn;

typedef struct mppt_can_480_temperatures
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

    } mppt_can_480_temperatures;





#endif