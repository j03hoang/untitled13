#include <Arduino.h>

#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "imu.h"
#include "motion.h"

Sensors sensors;
Motors motors;
Encoders encoders;
IMU gyro;

void stop() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, MOTOR_MAX_PWM);

    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);
}

void waitUntilTurnFinished(float omega) {
    float desired = fmod(gyro.getPrevAngle() + omega, 360);
    Serial.print("desired: ");
    Serial.println(desired);
    while (!(gyro.getRotChange() < desired + 0.5 && gyro.getRotChange() > desired - 0.5)) {
        gyro.update();
        delay(2);
    }
}

void waitUntilAheadFinished(float deltaX) {
    float x0 = encoders.getDistance();
    Serial.print("current distance: ");
    Serial.println(x0);
    while (encoders.getDistance() < x0 + deltaX) {
        encoders.update();
        delay(2);
    }

    Serial.print("new distance");
    Serial.println(encoders.getDistance());
}

void moveAhead() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);

    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, MOTOR_MAX_PWM);

    waitUntilAheadFinished(FULL_CELL);
    stop();
}

void turnRight() {
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, MOTOR_MAX_PWM);

    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_2, 0);

    waitUntilTurnFinished(-90);

    stop();
//    moveAhead();
}

void turnLeft() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, 0);

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);

    waitUntilTurnFinished(90);

    stop();
    moveAhead();
}

void turnBack() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, 0);

    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_2, 0);

    waitUntilAheadFinished(-FULL_CELL);
    stop();
}

void setup() {
    Serial.begin(BAUD_RATE);
    motors.init();
    sensors.init();
    encoders.init();
    gyro.init();
}

void loop() {
    turnRight();
    delay(1000);
    turnRight();
    delay(1000);
    turnRight();
    delay(1000);

//    sensors.update();

    // maze update

//    if (!sensors.see_right_wall)
//        turnRight();
//    else if (!sensors.see_front_wall)
//        moveAhead();
//    else if (!sensors.see_left_wall)
//        turnLeft();
//    else
//        turnBack();

//
//    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
//    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);
//
//    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
//    analogWrite(MOTOR_B_2, MOTOR_MAX_PWM);
//    moveAhead();
//    delay(3000);
//
//    turnBack();
//
//    delay(3000);
}


// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration-Staus-not-stable/td-p/8375
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration/m-p/6039/highlight/true#M70