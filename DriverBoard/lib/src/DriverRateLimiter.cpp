#include "DriverRateLimiter.h"
#include <chrono>
#include <iostream>
#include <thread>
 
//Re-implementation of TokenBucket, not yet formatted

class TokenBucket {
public:
    TokenBucket(int tokens, int time_unit, void (*forward_callback)(int)) {
        this->tokens = tokens;
        this->time_unit = time_unit;
        this->forward_callback = forward_callback;
        this->bucket = static_cast<float>(tokens);
        this->last_check = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
private:
    int tokens;
    int time_unit;
    void (*forward_callback)(int);
    void (*drop_callback)(int);
    float bucket;
    long long last_check;
};
 
void forward(int packet) {
    std::cout << "Packet Forwarded: " << packet << std::endl;
}

void drop(int packet) {
    std::cout << "Packet Dropped: " << packet << std::endl;
}

TokenBucket::TokenBucket(int tokens, int time_unit, void (*forward_callback)) {
    this->tokens = tokens;
    this->time_unit = time_unit;
    this->forward_callback = forward_callback;
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
        this->forward_callback(CANMessage *message, uint16_t message_id);
    }
}

/* Main function for testing
int main() {
    TokenBucket throttle(1, 1000, forward, drop);
    int packet = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        throttle.handle(packet);
        packet += 1;
    }
    return 0;
}
*/