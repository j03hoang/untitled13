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

RotaryEncoder encoder1(ENCODER_B_1, ENCODER_B_2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder2(ENCODER_B_1, ENCODER_B_2, RotaryEncoder::LatchMode::TWO03);
//Adafruit_BNO055 bno = Adafruit_BNO055(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);

//ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
//    encoders.update();
//    gyro.update();
//}

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

long leftCount2 = 0;
long rightCount2 = 0;

void doEncoder() {
    encoder1.tick();
    encoder2.tick();
    leftCount2 = encoder1.getPosition();
    rightCount2 = encoder2.getPosition();
    Serial.print("RIGHT:");
    Serial.println(rightCount2);
    Serial.print("LEFT:");
    Serial.println(leftCount2);
};

static int lastPos = 0;

void doEncoder2() {
    encoder2.tick();

    int newPos = encoder2.getPosition();


    if (lastPos != newPos) {

        // accelerate when there was a previous rotation in the same direction.

        unsigned long ms = encoder2.getMillisBetweenRotations();

        if (ms < longCutoff) {
            // do some acceleration using factors a and b

            // limit to maximum acceleration
            if (ms < shortCutoff) {
                ms = shortCutoff;
            }

            float ticksActual_float = a * ms + b;
            Serial.print("  f= ");
            Serial.println(ticksActual_float);

            long deltaTicks = (long)ticksActual_float * (newPos - lastPos);
            Serial.print("  d= ");
            Serial.println(deltaTicks);

            newPos = newPos + deltaTicks;
            encoder2.setPosition(newPos);
        }

        Serial.print(newPos);
        Serial.print("  ms: ");
        Serial.println(ms);
        lastPos = newPos;
    }
}


void loop() {
//    if (!sensors.see_right_wall)
//        turnRight();
//    else if (!sensors.see_front_wall)
//        moveAhead();
//    else if (!sensors.see_left_wall)
//        turnLeft();
//    else
//    sensors.update();
//        turnBack();
//

//    gyro.update();

//    analogWrite(MOTOR_A_FWD, MOTOR_MAX_PWM * 0.75);
//    analogWrite(MOTOR_A_REVERSE, 0);
//
//    analogWrite(MOTOR_B_FWD, MOTOR_MAX_PWM * 0.75);
//    analogWrite(MOTOR_B_REVERSE, 0);
//
    doEncoder();

    delay(1000);
}