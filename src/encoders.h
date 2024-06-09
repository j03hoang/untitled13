#ifndef UNTITLED13_ENCODERS_H
#define UNTITLED13_ENCODERS_H

#include <Encoder.h>
#include "config.h"

class Encoders;
extern Encoders encoders;

Encoder encoderLeft(ENCODER_A_1, ENCODER_A_2);
Encoder encoderRight(ENCODER_B_1, ENCODER_B_2);

class Encoders {
    public:
     void init() {
         reset();
     }

     void reset() {
         m_left_counter = 0;
         m_right_counter = 0;
         m_robot_distance = 0;
     }

     void update() {
         float leftDelta = m_left_counter;
         float rightDelta = m_right_counter;
         m_left_counter = 0;
         m_right_counter = 0;
         float leftChange = leftDelta * MM_PER_COUNT_LEFT;
         float rightChange = rightDelta * MM_PER_COUNT_RIGHT;
         m_fwd_change = 0.5f * (leftChange + rightChange);
         m_robot_distance += m_fwd_change;

         Serial.println(m_robot_distance);
     }

     void leftInputChange() {
         m_left_counter = encoderLeft.read();
         Serial.println(m_left_counter);
     }

     void rightInputChange() {
         m_right_counter = encoderRight.read();
         Serial.println(m_right_counter);
     }

     float getFwdChange() const {
         return m_fwd_change;
     }

    private:
     float m_robot_distance;
     float m_fwd_change;
     uint8_t m_left_counter;
     uint8_t m_right_counter;
};


#endif
