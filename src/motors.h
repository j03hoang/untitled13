#ifndef UNTITLED13_MOTORS_H
#define UNTITLED13_MOTORS_H

#include <Arduino.h>
#include "encoders.h"
#include "config.h"
#include "imu.h"

class Motors;
extern Motors motors;

class Motors {
    public:
     void init() {
         pinMode(MOTOR_A_FWD, OUTPUT);
         pinMode(MOTOR_A_REVERSE, OUTPUT);
         pinMode(MOTOR_B_FWD, OUTPUT);
         pinMode(MOTOR_B_REVERSE, OUTPUT);
         resetControllers();
         stop();
     }

     void resetControllers() {
         m_fwd_error = 0;
         m_rot_error = 0;
         m_prev_fwd_error = 0;
         m_prev_rot_error = 0;
     }

     void stop() {
         set_left_motor_pwm(0);
         set_right_motor_pwm(0);
     }

     float positionController() {
         float increment = m_velocity * LOOP_INTERVAL;
         m_fwd_error += increment - encoders.getFwdChange();
         float errorDiff = m_fwd_error - m_prev_fwd_error;
         float output = FWD_KP * m_fwd_error + FWD_KD * errorDiff;
         return output;
     }

     float angleController() {
         float increment = m_omega * LOOP_INTERVAL;
         m_rot_error += increment - gyro.getRotChange();

         return 0;
     }

//     void updateControllers(float velocity, float omega, float steeringAdjustment) {
//         m_velocity = velocity;
//         m_omega = omega;
//
//         float posOutput = positionController();
//
//     }

     void set_left_motor_pwm(int pwm) {
         pwm = constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
         if (pwm < 0)
             CCW(MOTOR_A_FWD, MOTOR_B_REVERSE, -pwm);
         else
             CW(MOTOR_A_FWD, MOTOR_B_REVERSE, pwm);
     }

     void set_right_motor_pwm(int pwm) {
         pwm = constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
         if (pwm < 0)
             CCW(MOTOR_B_FWD, MOTOR_B_REVERSE, -pwm);
         else
             CW(MOTOR_B_FWD, MOTOR_B_REVERSE, pwm);
     }

     void CCW(uint8_t pin1, uint8_t pin2, int pwm) {
         pwm = constrain(pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
         analogWrite(pin1, 0);
         analogWrite(pin2, pwm);
     }

     void CW(uint8_t pin1, uint8_t pin2, int pwm) {
         pwm = constrain(pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
         analogWrite(pin1, pwm);
         analogWrite(pin2, 0);
     }

    private:
     float m_velocity;
     float m_fwd_error;
     float m_prev_fwd_error;
     float m_omega;
     float m_rot_error;
     float m_prev_rot_error;


};


#endif
