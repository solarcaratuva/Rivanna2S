#include <mutex>
#include <sstream>

// Old implementation

// class TokenBucket {
// public:
//     // Class Structure
//     TokenBucket(string message_id, int refill_rate, int last_time) : message_id("default"), refillRate_(refillRate), lastTime(0);

//     // Determines whether the last message sent is beyond set interval
//     void past_interval_time(int last_time, int refill_rate);

//     // Get last time value
//     void get_last_time();

//     // Get refill rate value
//     void get_refill_rate();
// };

// Reimplementation
class TokenBucket {
public:
    TokenBucket(int tokens, int time_unit, void (*forward_callback)(int));

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id)
};