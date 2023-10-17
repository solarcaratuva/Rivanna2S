#include <chrono>
#include <mutex>

class TokenBucket {
public:
    // bucket_capacity represents the current number of tokens in the bucket while total_capacity is the maximum number of tokens
    TokenBucket(int token_capacity, int refill_rate) : bucket_capacity(capacity), refillRate_(refillRate), total_capacity(capacity) {}


private:
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
