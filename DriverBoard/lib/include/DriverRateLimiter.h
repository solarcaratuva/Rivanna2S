#include <mutex>
#include <sstream>
#include <mbed.h>
#include <chrono>


// Reimplementation of message handler
class TokenBucket {
public:
    int tokens;
    int time_unit;
    float bucket;
    int64_t last_check;

    TokenBucket(int tokens, int time_unit);

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id);
};