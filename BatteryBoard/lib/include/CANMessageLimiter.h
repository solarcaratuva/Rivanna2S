#pragma once

#include <mbed.h>
#include <chrono>
#include <thread>

// Reimplementation of message handler
class CANMessageLimiter {
public:
    int capacity;
    int time_unit_ms;
    float tokens;
    int64_t last_check;

    CANMessageLimiter(int capacity, int time_unit_ms);

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id);
};