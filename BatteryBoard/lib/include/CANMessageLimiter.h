#pragma once

#include <mbed.h>
#include <chrono>
#include <thread>

// Reimplementation of message handler
class CANMessageLimiter {
public:
    int tokens;
    int time_unit;
    float bucket;
    int64_t last_check;

    CANMessageLimiter(int tokens, int time_unit);

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id);
};