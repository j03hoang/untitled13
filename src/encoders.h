#ifndef UNTITLED13_ENCODERS_H
#define UNTITLED13_ENCODERS_H

#include <RotaryEncoder.h>
#include "config.h"

RotaryEncoder *encoderLeft = nullptr;
RotaryEncoder *encoderRight = nullptr;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

class Encoders;

extern Encoders encoders;

class Encoders {
    public:
     void init() {
         encoderLeft = new RotaryEncoder(ENCODER_A_IN1, ENCODER_A_IN2, RotaryEncoder::LatchMode::TWO03);
         encoderRight = new RotaryEncoder(ENCODER_B_IN1, ENCODER_B_IN2, RotaryEncoder::LatchMode::TWO03);

         attachInterrupt(digitalPinToInterrupt(ENCODER_A_IN1), leftInputChange, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_A_IN2), leftInputChange, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_B_IN1), rightInputChange, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_B_IN2), rightInputChange, CHANGE);
     }

     void update() {
         int leftDelta = m_left_counter;
         int rightDelta = m_right_counter;
         m_left_counter = 0;
         m_right_counter = 0;
         float leftChange = leftDelta * MM_PER_COUNT_LEFT;
         float rightChange = rightDelta * MM_PER_COUNT_RIGHT;
         m_fwd_change = 0.5f * (leftChange + rightChange);
         m_robot_distance += m_fwd_change;
     }

     void leftInputChange() {
         encoderLeft->tick();
         leftEncoderTicks = encoderLeft->getPosition();
         Serial.print("LEFT:");
         Serial.println(leftEncoderTicks);
     }

     void rightInputChange() {
         encoderRight->tick();
         rightEncoderTicks = encoderRight->getPosition();
         Serial.print("RIGHT:");
         Serial.println(rightEncoderTicks);
     }

     float getFwdChange() const {
         return m_fwd_change;
     }

    private:
     float m_robot_distance;
     float m_robot_angle;
     float m_fwd_change;
     float m_rot_change;
     int m_left_counter = 0;
     int m_right_counter = 0;
};

inline void left_encoder_isr() {
    return encoders.leftInputChange();
}

inline void right_encoder_isr() {
    return encoders.rightInputChange();
}


#endif
