#include "CANMessageLimiter.h"
#include "BPSCANInterface.h"
#include "BatteryBoardCANInterface.h"
 
CANMessageLimiter::CANMessageLimiter(int capacity, int time_unit_ms) {
    this->capacity = capacity;
    this->time_unit_ms = time_unit_ms;
    this->tokens = static_cast<float>(capacity);
    this->last_check = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void CANMessageLimiter::handle(CANMessage *message, uint16_t message_id) {
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto time_passed = current - this->last_check;
    this->last_check = current;        
    this->tokens = this->tokens + time_passed * (static_cast<float>(this->capacity) / this->time_unit_ms);
    if (this->tokens > this->capacity) {
        this->tokens = this->capacity;
    }

    if (this->tokens >= 1) {
        this->tokens -= 1;
        log_debug("Message Sent | Message Type: %s\n", message_id);
        BatteryBoardCANInterface::send_to_pi(message, message_id);
    }

    else {
        log_debug("Message Dropped | Message Type: %s\n", message_id);
    }
}

