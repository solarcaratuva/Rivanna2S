#include <mutex>
#include <sstream>


// Reimplementation of message handler
class TokenBucket {
public:
    TokenBucket(int tokens, int time_unit);

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id);
};