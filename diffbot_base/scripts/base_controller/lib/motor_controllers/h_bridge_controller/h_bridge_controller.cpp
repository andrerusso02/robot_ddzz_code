#include "h_bridge_controller/h_bridge_controller.h"

diffbot::HBridgeController::HBridgeController(int enable_pin, int in1_pin, int in2_pin)
{
    enable_pin_ = enable_pin;
    in1_pin_ = in1_pin;
    in2_pin_ = in2_pin;
}

void diffbot::HBridgeController::begin() {
    pinMode(enable_pin_, OUTPUT);
    pinMode(in1_pin_, OUTPUT);
    pinMode(in2_pin_, OUTPUT);
}

void diffbot::HBridgeController::setSpeed(int value)
{
    if (value > 0)
    {
        digitalWrite(in1_pin_, HIGH);
        digitalWrite(in2_pin_, LOW);
    }
    else if (value < 0)
    {
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, HIGH);
        value = value * -1;
    }
    else // zero speed
    {
        // Cut power to the motor
        digitalWrite(in1_pin_, LOW);
        digitalWrite(in2_pin_, LOW);
    }

    analogWrite(enable_pin_, value);
}