#include <chrono>
#include <mutex>

class TokenBucket {
public:
    TokenBucket(int capacity, int refillRate) : capacity_(capacity), refillRate_(refillRate), tokens_(capacity) {}

    bool tryConsume(int tokens) {
        std::lock_guard<std::mutex> lock(mutex_);
        refillTokens();
        if (tokens_ >= tokens) {
            tokens_ -= tokens;
            return true;
        }
        return false;
    }

private:
    void refillTokens() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRefillTime_);
        int tokensToAdd = elapsed.count() * refillRate_ / 1000;
        tokens_ = std::min(capacity_, tokens_ + tokensToAdd);
        lastRefillTime_ = now;
    }

    int capacity_;
    int refillRate_;
    int tokens_;
    std::chrono::steady_clock::time_point lastRefillTime_ = std::chrono::steady_clock::now();
    std::mutex mutex_;
};
