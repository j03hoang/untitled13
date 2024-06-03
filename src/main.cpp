#include <Arduino.h>
#include <RotaryEncoder.h>


#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "imu.h"
#include "motion.h"

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
    encoders.update();
    motion.update();

}

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

double convert(int numTicks) {
    double numRotations = (double) numTicks / ROT_PER_TICK;
    return numRotations * 135;
}

void setup() {
    Serial.begin(9600);
    sensors.init();
}

void loop() {

}