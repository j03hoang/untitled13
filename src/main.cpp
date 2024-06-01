#include <Arduino.h>
#include <RotaryEncoder.h>


#include "config.h"
#include "sensors.h"
#include "encoders.h"

//void daMotor() {
//    Serial.println("FORWARD");
//    analogWrite(MOTOR_B_FWD, MAX_PWR / 2);
//    analogWrite(MOTOR_B_REVERSE, LOW_);
//
//    analogWrite(MOTOR_A_FWD, MAX_PWR / 2);
//    analogWrite(MOTOR_A_REVERSE, LOW_);
//
//
//    delay(3000);
//
//    Serial.println("Stop");
//    analogWrite(MOTOR_B_FWD, MAX_PWR);
//    analogWrite(MOTOR_B_REVERSE, MAX_PWR);
//
//    analogWrite(MOTOR_A_FWD, MAX_PWR);
//    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
//    delay(3000);
//
//
//    Serial.println("REVERSE");
//    analogWrite(MOTOR_B_FWD, LOW_);
//    analogWrite(MOTOR_B_REVERSE, MAX_PWR);
//
//    analogWrite(MOTOR_A_FWD, LOW_);
//    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
//    delay(3000);
//
//
//    Serial.println("Stop");
//    analogWrite(MOTOR_B_FWD, MAX_PWR);
//    analogWrite(MOTOR_B_REVERSE, MAX_PWR);
//
//    analogWrite(MOTOR_A_FWD, MAX_PWR);
//    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
//    delay(3000);
//
//    Serial.println("TURN AROUND");
//    analogWrite(MOTOR_B_FWD, MAX_PWR);
//    analogWrite(MOTOR_B_REVERSE, LOW_);
//
//    analogWrite(MOTOR_A_FWD, LOW_);
//    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
//    delay(1000);
//
//    Serial.println("TURN RIGHT");
//    analogWrite(MOTOR_B_FWD, MAX_PWR);
//    analogWrite(MOTOR_B_REVERSE, LOW_);
//
//    analogWrite(MOTOR_A_FWD, LOW_);
//    analogWrite(MOTOR_A_REVERSE, LOW_);
//    delay(1000);
//
//    Serial.println("TURN LEFT");
//    analogWrite(MOTOR_B_FWD, LOW_);
//    analogWrite(MOTOR_B_REVERSE, LOW_);
//
//    analogWrite(MOTOR_A_FWD, MAX_PWR);
//    analogWrite(MOTOR_A_REVERSE, LOW_);
//    delay(1000);
//}

double getDistance() {
    unsigned long duration, distance;
    digitalWrite(ULTRASONIC_TRIG_1, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_ECHO_1, LOW);
    duration = pulseIn(ULTRASONIC_ECHO_1, HIGH);
    distance = (duration/2) / 29.1;

    delay(100);

    return distance;
}

double convert(int numTicks) {
    double numRotations = (double) numTicks / ROT_PER_TICK;
    return numRotations * 135;
}

void setup() {
    Serial.begin(9600);
    sensors.init();
}

void loop() {
    if (sensors.do_i_see_a_wall_in_front()) {
        Serial.println("true");
    } else {
        Serial.println("false");
    }
    delay(1000);
}