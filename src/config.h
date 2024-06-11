#ifndef UNTITLED13_CONFIG_H
#define UNTITLED13_CONFIG_H

#include <Arduino.h>

const int BAUD_RATE = 9600;

// TODO: for PID controllers
const int LOOP_FREQUENCY = 500;
const int LOOP_INTERVAL = 1 / LOOP_FREQUENCY;

/**  SENSORS */
const int ULTRASONIC_TRIG_A = 11; // front
const int ULTRASONIC_ECHO_A = PIN_A11;
const int ULTRASONIC_TRIG_B = 10; // left
const int ULTRASONIC_ECHO_B = PIN_A10;
const int ULTRASONIC_TRIG_C = 12; // right
const int ULTRASONIC_ECHO_C = PIN_A12;

const float SPEED_OF_SOUND = 29.1;

// SENSOR CALIBRATIONS
const int FRONT_THRESHOLD = 2;  // cm, minimum value to register a wall
const int LEFT_THRESHOLD = 2;
const int RIGHT_THRESHOLD = 2;


//https://components101.com/sites/default/files/component_datasheet/L298N-Motor-Driver-Datasheet.pdf
/** MOTORs */
const int MOTOR_A_1 = 4; // right
const int MOTOR_A_2 = 5;
const int MOTOR_B_1 = 6; // left
const int MOTOR_B_2 = 7;

const int MOTOR_MAX_PWM = 255;
const int MOTOR_MIN_PWM = 100;

// TODO: PID controllers
const int FWD_KP = -1;
const int FWD_KD = -1;

/** ENCODERS */
// uses internal interrupts, refer to: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const int ENCODER_A_1 = 18; // right
const int ENCODER_A_2 = 19;
const int ENCODER_B_1 = 2; // left
const int ENCODER_B_2 = 3;

const int ROT_PER_TICK = 617;
const int WHEEL_CIRCUMFERENCE = 135; // mm

/** IMU */
const uint8_t IMU_ADDRESS = 0x68;

const float FULL_CELL = 180.0f; // mm

#endif
