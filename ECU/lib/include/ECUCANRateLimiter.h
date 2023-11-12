#include <mutex>
#include <sstream>

class TokenBucket {
public:
    // Class Structure
    TokenBucket(string message_id, int token_capacity, int refill_rate, int last_time) : message_id("default"), bucket_capacity(token_capacity), refillRate_(refillRate), lastTime(0);

    // Determines whether the last message sent is beyond set interval
    void past_interval_time(int last_time, int refill_rate);
};
