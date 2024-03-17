#include <gtest/gtest.h>
#include "../PowerAux/lib/include/SignalFlashHandler.h"
#include "../PowerAux/lib/include/TestSignalControl.h"
#include "ethan.h"
 // Include your test implementation
// TestSignalControl testSignalControl; // Use your test implementation
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// Test fixture for SignalFlashHandler testing
class SignalFlashHandlerTest : public ::testing::Test {
protected:
    // TestSignalControl testSignalControl;

    // SetUp function to configure test conditions
    void SetUp() override {
        // Additional setup if needed
    }

    // TearDown function to clean up after the test
    void TearDown() override {
        // Additional teardown if needed
    }
};

// Test case to check the functionality of SignalFlashHandler
TEST_F(SignalFlashHandlerTest, TestSignalFlashHandlerFunctionality) {
    // Update state to simulate conditions
    SignalFlashHandler signalFlashHandler;
    signalFlashHandler.updateState(false, true, true, false);

    // Set callbacks for sleep and wait

    signalFlashHandler.set_callbacks([](){}, [](){});

    ASSERT_TRUE(signalFlashHandler.flashLSignal);
    ASSERT_TRUE(signalFlashHandler.flashRSignal);

}

TEST_F(SignalFlashHandlerTest, TestSignalHandleFunctionalityLeftAndRightTurnSignal) {
    // Update state to simulate conditions
    SignalFlashHandler signalFlashHandler;
    signalFlashHandler.updateState(false, false, true, false);

    // Set callbacks for sleep and wait


    signalFlashHandler.set_callbacks([](){}, [](){});

    ASSERT_FALSE(signalFlashHandler.leftTurnSignal);
    ASSERT_FALSE(signalFlashHandler.rightTurnSignal);

    signalFlashHandler.handle();

    ASSERT_FALSE(signalFlashHandler.leftTurnSignal);
    ASSERT_TRUE(signalFlashHandler.rightTurnSignal);
    // Call the handle function
    //signalFlashHandler.handle();

    // Validate the expected behavior based on your implementation
    // You can use ASSERT_* or EXPECT_* macros to check expected results

}

TEST_F(SignalFlashHandlerTest, TestSignalHandleFunctionalityFlashHazards) {
    //test flash hazards

    // Update state to simulate conditions
    SignalFlashHandler signalFlashHandler;
    signalFlashHandler.updateState(true, false, false, false);

    // Set callbacks for sleep and wait

    signalFlashHandler.set_callbacks([](){}, [](){});

    ASSERT_FALSE(signalFlashHandler.leftTurnSignal);
    ASSERT_FALSE(signalFlashHandler.rightTurnSignal);

    signalFlashHandler.handle();

    ASSERT_TRUE(signalFlashHandler.leftTurnSignal);
    ASSERT_TRUE(signalFlashHandler.rightTurnSignal);

}

TEST_F(SignalFlashHandlerTest, TestSignalHandleFunctionalityFlashBMS) {
    //test flash hazards

    // Update state to simulate conditions
    SignalFlashHandler signalFlashHandler;
    signalFlashHandler.updateState(false, false, false, true);

    // Set callbacks for sleep and wait

    signalFlashHandler.set_callbacks([](){}, [](){});

    ASSERT_FALSE(signalFlashHandler.bms_strobe);

    signalFlashHandler.handle();

    ASSERT_TRUE(signalFlashHandler.bms_strobe);

}