#include "ECUCANRateLimiter.h"
#include <chrono>
#include <tuple>
 
 
// Change this to be more accurate than just seconds
std::tuple<int, bool> TokenBucket::past_interval_time(int last_time, int refill_rate) {
    if (last_time == 0) {
        log_debug("First Time for this CAN Message");
    }
    
    const auto current_time = std::chrono::system_clock::now();
    double current_time_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count();
    if ((current_time_seconds - last_time) >= refill_rate) {
        last_time = current_time_seconds;
        past_interval = true;
    }
    else {
        last_time = last_time;
        past_interval = false;
    }
    return std::make_tuple(last_time, past_interval);
}

// Get value of last time
double TokenBucket::get_last_time(){
    return last_time;
}

// Set value of last time
void TokenBucket::set_last_time(double new_last_time){
    last_time = new_last_time;
}

// Get value of refill rate
int TokenBucket::get_refill_rate(){
    return refill_rate;
}


// Re-implementation of TokenBucket, not yet formatted

#include <iostream>
#include <chrono>
#include <thread>

class TokenBucket {
public:
    TokenBucket(int tokens, int time_unit, void (*forward_callback)(int), void (*drop_callback)(int)) {
        this->tokens = tokens;
        this->time_unit = time_unit;
        this->forward_callback = forward_callback;
        this->drop_callback = drop_callback;
        this->bucket = static_cast<float>(tokens);
        this->last_check = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void handle(int packet) {
        auto current = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        auto time_passed = current - this->last_check;
        this->last_check = current;        
        this->bucket = this->bucket + time_passed * (static_cast<float>(this->tokens) / this->time_unit);
        if (this->bucket > this->tokens) {
            this->bucket = this->tokens;
        }
        if (this->bucket < 1) {
            this->drop_callback(packet);
        } else {
            this->bucket -= 1;
            this->forward_callback(packet);
        }
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