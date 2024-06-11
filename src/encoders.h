#ifndef UNTITLED13_ENCODERS_H
#define UNTITLED13_ENCODERS_H

#include <Encoder.h>
#include "config.h"

/**
 * Library: https://github.com/PaulStoffregen/Encoder
 * uses an internal interrupt which tracks ticks whenever there is a clock CHANGE
 *
 * Datasheet: https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors.pdf
 */

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

     /**
      *
      */
     void update() {
         leftInputChange();
         rightInputChange();
         float leftTicks = m_left_counter - m_prev_left_counter;
         float rightTicks = m_right_counter - m_prev_right_counter;
         m_prev_left_counter = m_left_counter;
         m_prev_right_counter = m_right_counter;

         float leftChange = convertTicksToMM(leftTicks);
         float rightChange = convertTicksToMM(rightTicks);
         m_fwd_change = 0.5f * (leftChange + rightChange);
         m_robot_distance += m_fwd_change;

         Serial.print("DISTANCE: ");
         Serial.println(m_robot_distance);
     }

    /** determine distance traveled as a function of rots/tick and wheel circumference */
     float convertTicksToMM(int numTicks) {
         float numRotations = (float) numTicks / ROT_PER_TICK;
         float distance = numRotations * WHEEL_CIRCUMFERENCE;
         return distance;
     }

    /** reads total number of ticks of either wheel*/
    // TODO: counters are somehow decreasing when library should track absolute ticks
     void leftInputChange() {
         m_left_counter = encoderLeft.read();
         Serial.print("LEFT: ");
         Serial.println(m_left_counter);
     }

     void rightInputChange() {
         m_right_counter = encoderRight.read();
         Serial.print("RIGHT: ");
         Serial.println(m_right_counter);
     }

     float getDistance() const {
         Serial.println(m_robot_distance);
         return m_robot_distance;
     }

     float getFwdChange() const {
         return m_fwd_change;
     }

    private:
     float m_robot_distance;
     float m_fwd_change;
     uint8_t m_left_counter;
     uint8_t m_right_counter;
     uint8_t m_prev_left_counter;
     uint8_t m_prev_right_counter;
};

extern Encoders encoders;

#endif
