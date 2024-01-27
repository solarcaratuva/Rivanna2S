#include "MotorCANInterface.h"
#include "MotorControllerCANInterface.h"
#include "MotorInterface.h"
#include "MotorStateTracker.h"
#include "Printing.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>

#define LOG_LEVEL        LOG_DEBUG
#define MAIN_LOOP_PERIOD 100ms

EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
Thread event_thread;

MotorCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX);
MotorControllerCANInterface motor_controller_can_interface(MTR_CTRL_CAN_RX,
                                                           MTR_CTRL_CAN_TX,
                                                           MTR_CTRL_CAN_STBY);

// Motor Interface
I2C throttle(SDA_ACCEL, SCL_ACCEL);
I2C regen(SDA_REGEN, SCL_REGEN);
DigitalOut gear(FWD_REV_EN);
DigitalOut ignition(MAIN_SWITCH);

MotorInterface motor_interface(throttle, regen, gear, ignition);

// Motor State Tracker
MotorStateTracker motor_state_tracker;

Timeout ECUMotorCommands_timeout;

// If we have not received an ECUMotorCommands struct in 100ms, we assume that
// the CAN bus is down and set the throttle to 0.
void handle_ECUMotorCommands_timeout() { motor_interface.sendThrottle(0x000); }

// RPM + Current
int rpm, current, suggestedSpeed;
bool enabled;

// set values for initalization
double dt, max, min, Kp, Kd, Ki;

void init(double _max, double _min, double _Kp, double _Kd, double _Ki){
    max = _max;
    min = _min;
    Kp = _Kp;
    Kd = _Kd;
    Ki = _Ki;
}

double calculate(double setpoint, double pv){
    // Calculate error
    double error = setpoint - pv;
    
    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;
    double output = Pout + Iout + Dout;

    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;
    
    _pre_error = error;

    return output;
}



int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");
    event_thread.start(callback(&event_queue, &EventQueue::dispatch_forever));

    ECUMotorCommands_timeout.attach(
    event_queue.event(handle_ECUMotorCommands_timeout), 100ms);

    // init(___, __, ___, ___, ___);
   // dt = 
    
    while (true) {
        if(!enabled){
            log_debug("Main thread loop");
            // request frames from the motor controller
            motor_controller_can_interface.request_frames(true, true, true);
            ThisThread::sleep_for(MAIN_LOOP_PERIOD);
        } else{
            log_debug("Cruise Control Loop");
            motor_controller_can_interface.request_frames(true, true, true);
            ThisThread::sleep_for(MAIN_LOOP_PERIOD);
        }
    }
}



void MotorCANInterface::handle(ECUMotorCommands *can_struct) {
    if(enabled){
        // Reset current timeout
        ECUMotorCommands_timeout.detach();
        // Set new timeout for 100ms from now
        ECUMotorCommands_timeout.attach(
            event_queue.event(handle_ECUMotorCommands_timeout), 100ms);

        can_struct->log(LOG_INFO);

        motor_interface.sendIgnition(can_struct->motor_on);
        motor_interface.sendDirection(
            can_struct->forward_en); // TODO: verify motor controller will not allow
                                    // gear change when velocity is non-zero
        motor_interface.sendThrottle(can_struct->throttle);
        motor_interface.sendRegen(can_struct->regen);
        // double send = calculate(suggestedSpeed, ___);
        // motor_interface.sendThrottle(send); 
        suggestedSpeed = can_struct->cruise_control_speed;
        enabled = can_struct->cruise_control_en;
    } else{
        // Reset current timeout
        ECUMotorCommands_timeout.detach();
        // Set new timeout for 100ms from now
        ECUMotorCommands_timeout.attach(
            event_queue.event(handle_ECUMotorCommands_timeout), 100ms);

        can_struct->log(LOG_INFO);

        motor_interface.sendIgnition(can_struct->motor_on);
        motor_interface.sendDirection(
            can_struct->forward_en); // TODO: verify motor controller will not allow
                                    // gear change when velocity is non-zero
        // do a calculation to send throttle
        suggestedSpeed = can_struct->cruise_control_speed;
        enabled = can_struct->cruise_control_en;
        log_error("R: %d T: %d", can_struct->regen, can_struct->throttle);

    }
}

void MotorControllerCANInterface::handle(
    MotorControllerPowerStatus *can_struct) {
    can_struct->log(LOG_INFO);
    rpm = can_struct->motor_rpm;
    current = can_struct->motor_current;
    suggestedSpeed = (rpm * 3.1415926535 * 16 * 60)/(63360); 
    // calculate(current, ___); 
    // timer
    // dt = (new timer) - (old timer)
    motor_state_tracker.setMotorControllerPowerStatus(*can_struct);
}

void MotorControllerCANInterface::handle(
    MotorControllerDriveStatus *can_struct) {
    can_struct->log(LOG_INFO);
    motor_state_tracker.setMotorControllerDriveStatus(*can_struct);
}

void MotorControllerCANInterface::handle(MotorControllerError *can_struct) {
    can_struct->log(LOG_INFO);
    motor_state_tracker.setMotorControllerError(*can_struct);
}


void MotorControllerCANInterface::message_forwarder(CANMessage *message) {
    // message_forwarder is called whenever the MotorControllerCANInterface gets a CAN message
    // this forwards the message to the vehicle can bus
    vehicle_can_interface.send_message(message);
}


