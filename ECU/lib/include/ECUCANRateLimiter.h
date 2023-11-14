#include <mutex>
#include <sstream>

class TokenBucket {
public:
    // Class Structure
    TokenBucket(string message_id, int refill_rate, int last_time) : message_id("default"), refillRate_(refillRate), lastTime(0);

    // Determines whether the last message sent is beyond set interval
    void past_interval_time(int last_time, int refill_rate);

    // Get last time value
    void get_last_time();

    // Get refill rate value
    void get_refill_rate();
};
