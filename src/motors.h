#ifndef UNTITLED13_MOTORS_H
#define UNTITLED13_MOTORS_H

#include <Arduino.h>
#include "config.h"

class Motors {
    void init() {
        pinMode(MOTOR_A_FWD, OUTPUT);
        pinMode(MOTOR_A_REVERSE, OUTPUT);
        pinMode(MOTOR_B_FWD, OUTPUT);
        pinMode(MOTOR_B_REVERSE, OUTPUT);
    }

    void updateControllers(float velocity, float omega, float steeringAdjustment) {

    }

    void set_left_motor_pwm(int pwm) {
        pwm = constrain(pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        if (pwm < 0) {
            analogWrite(MOTOR_A_REVERSE, pwm);
        } else {
            analogWrite(MOTOR_A_FWD, pwm);
        }
    }


};


#endif
