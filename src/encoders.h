#ifndef UNTITLED13_ENCODERS_H
#define UNTITLED13_ENCODERS_H

#include <RotaryEncoder.h>
#include "config.h"

RotaryEncoder *encoderLeft = nullptr;
RotaryEncoder *encoderRight = nullptr;

class Encoders;
extern Encoders encoders;

void left_encoder_isr();
void right_encoder_isr();

class Encoders {
    public:
     void init() {
         encoderLeft = new RotaryEncoder(ENCODER_A_IN1, ENCODER_A_IN2, RotaryEncoder::LatchMode::TWO03);
         encoderRight = new RotaryEncoder(ENCODER_B_IN1, ENCODER_B_IN2, RotaryEncoder::LatchMode::TWO03);

         attachInterrupt(digitalPinToInterrupt(ENCODER_A_IN1), left_encoder_isr, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_A_IN2), left_encoder_isr, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_B_IN1), right_encoder_isr, CHANGE);
         attachInterrupt(digitalPinToInterrupt(ENCODER_B_IN2), right_encoder_isr, CHANGE);
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
         m_right_counter = encoderLeft->getPosition();
         Serial.print("LEFT:");
         Serial.println(m_left_counter);
     }

     void rightInputChange() {
         encoderRight->tick();
         m_left_counter = encoderRight->getPosition();
         Serial.print("RIGHT:");
         Serial.println(m_right_counter);
     }

     float getFwdChange() const {
         return m_fwd_change;
     }

    private:
     float m_robot_distance;
     float m_fwd_change;
     long m_left_counter = 0;
     long m_right_counter = 0;
};

void left_encoder_isr() {
    return encoders.leftInputChange();
}

void right_encoder_isr() {
    return encoders.rightInputChange();
}

#endif
