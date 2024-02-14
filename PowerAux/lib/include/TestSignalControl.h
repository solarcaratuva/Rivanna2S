#pragma once

class TestSignalControl {
private:
    bool flashLSignal;
    bool flashRSignal;

public:
    TestSignalControl();

    void setFlashHazards(bool enable);
    void setFlashLeftSignal(bool enable);
    void setFlashRightSignal(bool enable);
    void setFlashBMS(bool enable);
    bool getFlashLeftSignalValue() const;
    bool getFlashRightSignalValue() const;

    bool isFlashHazardsSignaled();
    bool isFlashLeftSignalSignaled();
    bool isFlashRightSignalSignaled();
    bool isFlashBmsSignaled();

};