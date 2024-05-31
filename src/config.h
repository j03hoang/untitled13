#ifndef UNTITLED13_CONFIG_H
#define UNTITLED13_CONFIG_H

#include <Arduino.h>

/**  SENSORS */
const int ULTRASONIC_TRIG_1 = 4;
const int ULTRASONIC_ECHO_1 = PIN_A4;

// SENSOR CALIBRATIONS
const int LEFT_THRESHOLD = 40;   // minimum value to register a wall
const int RIGHT_THRESHOLD = 40;  // minimum value to register a wall
const int FRONT_THRESHOLD = 20;  // minimum value to register a wall
const float LEFT_SCALE = -1;
const float RIGHT_SCALE = -1;
const float FRONT_SCALE = -1;

/** MOTORs */
const int MOTOR_A_FWD = 7;
const int MOTOR_A_REVERSE = 8;
const int MOTOR_B_FWD = 9;
const int MOTOR_B_REVERSE = 10;

const int MOTOR_MAX_PWM = 255;
const int MOTOR_MIN_PWM = 100;

/** ENCODERS */
#define ENCODER_PIN_1 A2
#define ENCODER_PIN_2 A3

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
const int32_t BNO_SENSOR_ID = 55;
const uint8_t BNO_ADDRESS = 0x28;


const float FULL_CELL = 180.0f;

#endif
