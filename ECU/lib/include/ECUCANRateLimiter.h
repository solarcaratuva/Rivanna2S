#include <chrono>
#include <mutex>
#include <sstream>

class TokenBucket {
public:
    // bucket_capacity represents the current number of tokens in the bucket while total_capacity is the maximum number of tokens
    TokenBucket(string message_id, int token_capacity, int refill_rate) : message_id("default"), bucket_capacity(token_capacity), refillRate_(refillRate), total_capacity(token_capacity) {}


private:

    //change this to be more accurate than just seconds
    bool past_interval_time(int last_time, int refill_rate, int* last_time, int* past_interval) {
        const auto current_time = std::chrono::system_clock::now();
        int current_time_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch()).count();
        if ((current_time_seconds - last_time) >= refill_rate) {
            *last_time = current_time_seconds;
            *past_interval = true;
        }
        else {
            *last_time = last_time;
            *past_interval = false;
        }
    }
};
