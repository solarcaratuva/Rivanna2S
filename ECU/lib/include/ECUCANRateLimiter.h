#include <chrono>
#include <mutex>

class TokenBucket {
public:
    // bucket_capacity represents the current number of tokens in the bucket while total_capacity is the maximum number of tokens
    TokenBucket(int token_capacity, int refill_rate) : bucket_capacity(capacity), refillRate_(refillRate), total_capacity(capacity) {}


private:
    int get_current_time() {
        const auto current_time = std::chrono::system_clock::now();
        int current_time_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch()).count();

        return current_time_seconds;
    }
    bool past_interval_time(int refill_rate) {
        if ()
    }
};
