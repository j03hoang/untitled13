#ifndef UNTITLED13_PROFILE_H
#define UNTITLED13_PROFILE_H

#include <Arduino.h>
#include "config.h"

class Profile {
    public:
     enum State : uint8_t {
         STATE_IDLE = 0,
         STATE_ACCELERATING = 1,
         STATE_BRAKING = 2,
         STATE_FINISHED = 3
     };

     void start(float distance, float topSpeed, float finalSpeed, float acceleration) {
         m_state = STATE_ACCELERATING;
     }

     void update() {
         if (m_state == STATE_IDLE)
             return;

         float deltaV = m_acceleration * LOOP_INTERVAL;
         float remainingDist = fabsf(m_final_position) - fabsf(m_position);

         if (m_speed < m_target_speed) {
             m_speed += deltaV;
             if (m_speed > m_target_speed) {
                 m_speed = m_target_speed;
             }
         }

         if (m_speed > m_target_speed) {
             m_speed -= deltaV;
             if (m_speed < m_target_speed) {
                 m_speed = m_target_speed;
             }
         }

         m_position += m_speed * LOOP_INTERVAL;
         if (m_state != STATE_FINISHED && remainingDist < 0.125) {
             m_state = STATE_FINISHED;
             m_target_speed = m_final_speed;
         }
     }

    private:
     State m_state = STATE_IDLE;
     float m_position = 0;
     float m_final_position = 0;
     float m_speed = 0;
     float m_target_speed = 0;
     float m_final_speed = 0;
     float m_acceleration = 0;

};


#endif
