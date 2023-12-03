#include <mutex>
#include <sstream>


// Reimplementation
class TokenBucket {
public:
    TokenBucket(int tokens, int time_unit, void (*forward_callback)(int));

    // to-do: change int packet
    void handle(CANMessage *message, uint16_t message_id)
};