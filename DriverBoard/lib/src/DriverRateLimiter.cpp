#include "DriverRateLimiter.h"
#include "DriverCANInterface.h"
 
//Re-implementation of TokenBucket, not yet formatted
TokenBucket::TokenBucket(int tokens, int time_unit) {
    this->tokens = tokens;
    this->time_unit = time_unit;
    this->bucket = static_cast<float>(tokens);
    this->last_check = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void TokenBucket::handle(CANMessage *message, uint16_t message_id) {
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto time_passed = current - this->last_check;
    this->last_check = current;        
    this->bucket = this->bucket + time_passed * (static_cast<float>(this->tokens) / this->time_unit);
    if (this->bucket > this->tokens) {
        this->bucket = this->tokens;
    }

    if (this->bucket > 1) {
        this->bucket -= 1;
        log_debug("Message Sent | Message Type: %s\n", message_id);
        DriverCANInterface::send_to_pi(message, message_id);
    }

    else {
        log_debug("Message Dropped | Message Type: %s\n", message_id);
    }
}

