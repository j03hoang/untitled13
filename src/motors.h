#ifndef UNTITLED13_MOTORS_H
#define UNTITLED13_MOTORS_H

/**
 *
 * Datasheet: https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors.pdf
 *
 * Motors are attached throw a motor driver that regulates voltage. Refer to the following datasheet:
 * https://components101.com/sites/default/files/component_datasheet/L298N-Motor-Driver-Datasheet.pdf
 */

#include <Arduino.h>
#include "encoders.h"
#include "config.h"
#include "imu.h"

class Motors;
extern Motors motors;

class Motors {
    public:
     void init() {
         pinMode(MOTOR_A_1, OUTPUT);
         pinMode(MOTOR_A_2, OUTPUT);
         pinMode(MOTOR_B_1, OUTPUT);
         pinMode(MOTOR_B_2, OUTPUT);
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
         setLeftMotorPWM(MOTOR_MAX_PWM);
         setRightMotorPWM(MOTOR_MAX_PWM);
     }

     /** TODO: PID Controllers, UNIMPLEMENTED */
//     float positionController() {
//         float increment = m_velocity * LOOP_INTERVAL;
//         m_fwd_error += increment - encoders.getFwdChange();
//         float errorDiff = m_fwd_error - m_prev_fwd_error;
//         float output = FWD_KP * m_fwd_error + FWD_KD * errorDiff;
//         return output;
//     }
//
//     float angleController() {
//         float increment = m_omega * LOOP_INTERVAL;
//         m_rot_error += increment - gyro.getRotChange();
//
//         return 0;
//     }

//     void updateControllers(float velocity, float omega, float steeringAdjustment) {
//         m_velocity = velocity;
//         m_omega = omega;
//
//         float posOutput = positionController();
//
//     }

     void setLeftMotorPWM(int pwm) {
         pwm = constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
         if (pwm < 0)
             CCW(MOTOR_A_1, MOTOR_B_2, -pwm);
         else
             CW(MOTOR_A_1, MOTOR_B_2, pwm);
     }

     void setRightMotorPWM(int pwm) {
         pwm = constrain(pwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
         if (pwm < 0)
             CCW(MOTOR_B_1, MOTOR_B_2, -pwm);
         else
             CW(MOTOR_B_1, MOTOR_B_2, pwm);
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
     // TODO: track velocity, controller variables
     float m_velocity;
     float m_fwd_error;
     float m_prev_fwd_error;
     float m_omega;
     float m_rot_error;
     float m_prev_rot_error;


};


#endif
