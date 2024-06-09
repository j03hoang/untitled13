#ifndef UNTITLED13_CONFIG_H
#define UNTITLED13_CONFIG_H

#include <Arduino.h>

const int BAUD_RATE = 9600;
const int LOOP_FREQUENCY = 500;
const int LOOP_INTERVAL = 1 / LOOP_FREQUENCY;

/**  SENSORS */
const int ULTRASONIC_TRIG_A = 0; // front
const int ULTRASONIC_ECHO_A = PIN_A0;
const int ULTRASONIC_TRIG_B = 1; // left
const int ULTRASONIC_ECHO_B = PIN_A1;
const int ULTRASONIC_TRIG_C = 2; // right
const int ULTRASONIC_ECHO_C = PIN_A2;

const int IR_A = -1;
const int IR_B = -1;

const float SPEED_OF_SOUND = 29.1;

// SENSOR CALIBRATIONS
const int FRONT_THRESHOLD = 20;  // minimum value to register a wall
const int LEFT_THRESHOLD = 20;  // minimum value to register a wall
const int RIGHT_THRESHOLD = 20;  // minimum value to register a wall


//https://components101.com/sites/default/files/component_datasheet/L298N-Motor-Driver-Datasheet.pdf
/** MOTORs */
const int MOTOR_A_FWD = 4; // right
const int MOTOR_A_REVERSE = 5;
const int MOTOR_B_FWD = 6; // left
const int MOTOR_B_REVERSE = 7;

const int MOTOR_MAX_PWM = 255;
const int MOTOR_MIN_PWM = 100;

const int FWD_KP = -1;
const int FWD_KD = -1;

const int MM_PER_COUNT_LEFT = -1;
const int MM_PER_COUNT_RIGHT = -1;

/** ENCODERS */
// uses internal interrupts, refer to: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const int ENCODER_A_1 = 18; // right
const int ENCODER_A_2 = 19;
const int ENCODER_B_1 = 2; // left
const int ENCODER_B_2 = 3;

const int ROT_PER_TICK = 617;

// ENCODER CALIBRATIONS
// the maximum acceleration is 10 times.
constexpr float m = 10;

// at 200ms or slower, there should be no acceleration. (factor 1)
constexpr float longCutoff = 50;

// at 5 ms, we want to have maximum acceleration (factor m)
constexpr float shortCutoff = 5;

// To derive the calc. constants, compute as follows:
// On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
// where  f(4)=10 and f(200)=1
constexpr float a = (m - 1) / (shortCutoff - longCutoff);
constexpr float b = 1 - longCutoff * a;


/** IMU */
// https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
const int32_t BNO_SENSOR_ID = 55;
const uint8_t BNO_ADDRESS = 0x28;


const float FULL_CELL = 180.0f;

#endif
