#include <Arduino.h>

#include <Encoder.h>

#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "imu.h"
#include "motion.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Sensors sensors;
Motors motors;
Encoders encoders;
IMU gyro;

//Encoder myEnc1(ENCODER_A_1, ENCODER_A_2);
//Encoder myEnc2(ENCODER_B_1, ENCODER_B_2);

//ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
//    encoders.update();
//    gyro.update();
//}

double convert(long numTicks) {
    double numRotations = (double) numTicks / ROT_PER_TICK;
    return numRotations * 135;
}

//Adafruit_BNO055 bno = Adafruit_BNO055(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);

void stop() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, MOTOR_MAX_PWM);

    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);
}

void waitUntilTurnFinished(float omega) {
    int desired = ((int) (gyro.getPrevAngle() + 360 + omega)) % 360;
    while (!(gyro.getRotChange() < desired + 1 && gyro.getRotChange() > desired - 1)) {
        gyro.update();
        delay(2);
    }
}

void waitUntilAheadFinished(float deltaX) {
    float x0 = encoders.getDistance();
    Serial.print("current distance: ");
    Serial.println(x0);
    while (encoders.getDistance() < x0 + deltaX)
        delay(2);

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

    waitUntilTurnFinished(90);

    stop();
    moveAhead();
}

void turnLeft() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, 0);

    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, MOTOR_MAX_PWM);

    waitUntilTurnFinished(-90);

    stop();
//    moveAhead();
}

void turnBack() {
    analogWrite(MOTOR_B_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_B_2, 0);

    analogWrite(MOTOR_A_1, MOTOR_MAX_PWM);
    analogWrite(MOTOR_A_2, 0);

    waitUntilAheadFinished(-FULL_CELL);
    stop();
}

//void displaySensorStatus(void)
//{
//    /* Get the system status values (mostly for debugging purposes) */
//    uint8_t system_status, self_test_results, system_error;
//    system_status = self_test_results = system_error = 0;
//    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
//
//    /* Display the results in the Serial Monitor */
//    Serial.println("");
//    Serial.print("System Status: 0x");
//    Serial.println(system_status, HEX);
//    Serial.print("Self Test:     0x");
//    Serial.println(self_test_results, HEX);
//    Serial.print("System Error:  0x");
//    Serial.println(system_error, HEX);
//    Serial.println("");
//    delay(500);
//}

void setup() {
    Serial.begin(BAUD_RATE);
    sensors.init(); // good
    encoders.init(); // bad
    gyro.init(); // good
    motors.init(); // good

//    if(!bno.begin())
//    {
//        /* There was a problem detecting the BNO055 ... check your connections */
//        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//        while(1);
//    }
//    bno.setExtCrystalUse(true);
}

//void doEncoder3() {
//    long position1 = myEnc1.read();
//    Serial.print("1: ");
//    Serial.print(position1);
//    double actual1 = convert(position1);
//    Serial.print(" | ");
//    Serial.println(actual1);
//
//    long position2 = myEnc2.read();
//    Serial.print("2: ");
//    Serial.print(position2);
//    double actual2 = convert(position2);
//    Serial.print(" | ");
//    Serial.println(actual2);
//}

//void displayCalStatus(void)
//{
//    /* Get the four calibration values (0..3) */
//    /* Any sensor data reporting 0 should be ignored, */
//    /* 3 means 'fully calibrated" */
//    uint8_t system, gyro, accel, mag;
//    system = gyro = accel = mag = 0;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//
//    /* The data should be ignored until the system calibration is > 0 */
//    Serial.print("\t");
//    if (!system)
//    {
//        Serial.print("! ");
//    }
//
//    /* Display the individual values */
//    Serial.print("Sys:");
//    Serial.print(system, DEC);
//    Serial.print(" G:");
//    Serial.print(gyro, DEC);
//    Serial.print(" A:");
//    Serial.print(accel, DEC);
//    Serial.print(" M:");
//    Serial.print(mag, DEC);
//}

//int getAngle() {
//    sensors_event_t event;
//    bno.getEvent(&event);
//    return (int) event.orientation.x;
//}

float prevAngle = 0;

void loop() {
    sensors.update();

    // maze update

//    if (!sensors.see_right_wall)
//        turnRight();
//    else if (!sensors.see_front_wall)
//        moveAhead();
//    else if (!sensors.see_left_wall)
//        turnLeft();
//    else
//        turnBack();



    turnRight();

    delay(3000);

//    turnLeft();


//    encoders.leftInputChange();
//    encoders.rightInputChange();

//    while(true) {
//        encoders.leftInputChange();
//        encoders.rightInputChange();
//    }

    // right turn ((int)prevAngle + 360 + 90) % 36

//    gyro.update();
//    int y = (int) gyro.getRotChange();
//    Serial.print("y");
//    Serial.println(y);
//    int desired = ((int) y + 360 - 90) % 360;
//    Serial.print("DESIRED ANGLE: ");
//    Serial.println(desired);
//    while (!(y < desired + 1 && y > desired - 1)) {
//        Serial.println(y);
//        gyro.update();
//        y = (int) gyro.getRotChange();
//        delay(2);
//    }

//    gyro.update();
//    int y = (int) gyro.getRotChange();
//    Serial.print("y");
//    Serial.println(y);
//    int desired = ((int) gyro.getPrevAngle() + 360 - 90) % 360;
//    while (!(gyro.getRotChange() < desired + 1 && gyro.getRotChange() > desired - 1)) {
//        gyro.update();
//        delay(2);
//    }



}

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration-Staus-not-stable/td-p/8375
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration/m-p/6039/highlight/true#M70