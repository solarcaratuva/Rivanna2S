#include "ECUCANRateLimiter.h"
#include <chrono>
#include <tuple>
 
 
//change this to be more accurate than just seconds
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