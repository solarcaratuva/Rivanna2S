class SignalFlashHandler {
public:
    DigitalOut leftTurnSignal;
    DigitalOut rightTurnSignal;
    DigitalOut bms_strobe;
    // DigitalOut can be mocked, to test
    // Dependency injection
    SignalFlashHandler(DigitalOut leftTurnSignal, DigitalOut rightTurnSignal, DigitalOut bms_strobe, std::function<void()> sleep, std::function<void()> wait);
    // Called by other thread to update variables
    void updateState(bool flashHazards, bool flashLSignal, bool flashRSignal, bool flashBMS);
    // Handles change in state, will be tested
    void handle();
    // Calls handle method
    void loop();
}