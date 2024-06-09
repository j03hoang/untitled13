#include <Arduino.h>

#include <Encoder.h>

#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "imu.h"
#include "motion.h"

Sensors sensors;
Motors motors;
Encoders encoders;
IMU gyro;

Encoder myEnc1(ENCODER_A_1, ENCODER_A_2);
Encoder myEnc2(ENCODER_B_1, ENCODER_B_2);

//Adafruit_BNO055 bno = Adafruit_BNO055(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);

//ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
//    encoders.update();
//    gyro.update();
//}

double convert(long numTicks) {
    double numRotations = (double) numTicks / ROT_PER_TICK;
    return numRotations * 135;
}

void stop() {
    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_REVERSE, MOTOR_MAX_PWM);

    analogWrite(MOTOR_A_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_REVERSE, MOTOR_MAX_PWM);
}

void waitUntilTurnFinished(float omega) {
    while (gyro.getRotChange() < omega)
        delay(2);
}

void waitUntilAheadFinished(float distance) {
    while (encoders.getFwdChange() < distance)
        delay(2);
}

void moveAhead() {
    analogWrite(MOTOR_A_FWD, MOTOR_MAX_PWM / 2);
    analogWrite(MOTOR_A_REVERSE, 0);

    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM / 2);
    analogWrite(MOTOR_B_REVERSE, 0);

    waitUntilAheadFinished(FULL_CELL);
    stop();
}

void turnRight() {
    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_REVERSE, 0);

    analogWrite(MOTOR_A_FWD, 0);
    analogWrite(MOTOR_A_REVERSE, 0);

    waitUntilTurnFinished(90);

    stop();
    moveAhead();
}

void turnLeft() {
    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_REVERSE, 0);

    analogWrite(MOTOR_A_FWD, 0);
    analogWrite(MOTOR_A_REVERSE, 0);

    waitUntilTurnFinished(270);

    stop();
    moveAhead();
}

void turnBack() {
    analogWrite(MOTOR_B_FWD, 0);
    analogWrite(MOTOR_B_REVERSE, MOTOR_MAX_PWM / 2);

    analogWrite(MOTOR_A_FWD, 0);
    analogWrite(MOTOR_A_REVERSE, MOTOR_MAX_PWM / 2);

    waitUntilAheadFinished(-FULL_CELL);
    stop();
}

void setup() {
    Serial.begin(BAUD_RATE);
//    sensors.init(); // good
//    encoders.init(); // bad
    gyro.init(); // good
    motors.init(); // good

}

void doEncoder3() {

    long position1 = myEnc1.read();
    Serial.print("1: ");
    Serial.print(position1);
    double actual1 = convert(position1);
    Serial.print(" | ");
    Serial.println(actual1);

    long position2 = myEnc2.read();
    Serial.print("2: ");
    Serial.print(position2);
    double actual2 = convert(position2);
    Serial.print(" | ");
    Serial.println(actual2);
}

void doIMU() {

}

void loop() {
//    if (!sensors.see_right_wall)
//        turnRight();
//    else if (!sensors.see_front_wall)
//        moveAhead();
//    else if (!sensors.see_left_wall)
//        turnLeft();
//    else
//        turnBack();
//

//    sensors.update();


    // https://forums.adafruit.com/viewtopic.php?t=192881
    analogWrite(MOTOR_A_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_REVERSE, MOTOR_MAX_PWM);

    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_REVERSE, MOTOR_MAX_PWM);

    gyro.update();


//
//    doEncoder();
//    encoders.leftInputChange();
//    encoders.rightInputChange();

//    doEncoder3();

//    encoders.leftInputChange();
//    encoders.rightInputChange();

    delay(1000);

}