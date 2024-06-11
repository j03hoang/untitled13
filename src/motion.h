#ifndef UNTITLED13_MOTION_H
#define UNTITLED13_MOTION_H

#include <Arduino.h>
#include "motors.h"
#include "profile.h"

class Motion {

    public:
     void update() {
        // TODO: incorporate positional and angular velocity for smooth turns
     }

     void stop() {
         motors.stop();
     }

     void moveAhead() {
         motors.setLeftMotorPWM(MOTOR_MAX_PWM);
         motors.setRightMotorPWM(MOTOR_MAX_PWM);
         waitUntilAheadFinished(FULL_CELL);
         stop();
     }

     void moveBack() {
         motors.setLeftMotorPWM(-MOTOR_MAX_PWM);
         motors.setRightMotorPWM(-MOTOR_MAX_PWM);
         waitUntilAheadFinished(-FULL_CELL);
         stop();
     }

     void turnRight() {
         motors.setLeftMotorPWM(-MOTOR_MAX_PWM);
         motors.setRightMotorPWM(MOTOR_MAX_PWM);
         waitUntilTurnFinished(90);
         stop();
         moveAhead();
     }

     void turnLeft() {
         motors.setLeftMotorPWM(MOTOR_MAX_PWM);
         motors.setRightMotorPWM(-MOTOR_MAX_PWM);
         waitUntilTurnFinished(-90);
         stop();
         moveAhead();
     }

    static void waitUntilTurnFinished(float omega) {
        int desired = ((int) (gyro.getPrevAngle() + 360 + omega)) % 360;
        while (!(gyro.getRotChange() < desired + 1 && gyro.getRotChange() > desired - 1)) {
            gyro.update();
            delay(2);
        }
    }

    static void waitUntilAheadFinished(float deltaX) {
        float x0 = encoders.getDistance();
        Serial.print("current distance: ");
        Serial.println(x0);
        while (encoders.getDistance() < x0 + deltaX)
            delay(2);

        Serial.print("new distance");
        Serial.println(encoders.getDistance());
    }
};

extern Motion motion;


#endif
