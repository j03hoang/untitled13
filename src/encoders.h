#ifndef UNTITLED13_ENCODERS_H
#define UNTITLED13_ENCODERS_H

#include <RotaryEncoder.h>
#include "config.h"

RotaryEncoder encoder(ENCODER_PIN_1, ENCODER_PIN_2, RotaryEncoder::LatchMode::TWO03);

class Encoders {
    public:
     void init() {

     }

    private:
     float m_robot_distance;
     float m_robot_angle;
     float m_fwd_change;
     float m_rot_change;
};


#endif
