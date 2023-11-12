#include "ECUCANRateLimiter.h"
#include <chrono>
#include <tuple>
 
 
// Change this to be more accurate than just seconds
std::tuple<int, bool> TokenBucket::past_interval_time(int last_time, int refill_rate) {
    if (last_time == 0) {
        log_debug("First Time for this CAN Message");
    }

    const auto current_time = std::chrono::system_clock::now();
    int current_time_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch()).count();
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
