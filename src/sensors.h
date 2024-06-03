#ifndef UNTITLED13_SENSORS_H
#define UNTITLED13_SENSORS_H

#include "config.h"

/* Using 2 Ultrasonic Sensors and a photo resistor paired with an led
 *      - Ultrasonic Sensor has 15 degree range
 *      - We have 2 photoresistors/LED combos
 *          - must be calibrated to ambient light level :/ ie not very precise
 *
 *
 * TODO:
 *      - Figure out where sensors are going to mounted
 *      - calibrate photo resistor and LED
 *      - sense walls
 *          # 18cm x 18 cm unit cell
 *          - trigger sound
 *          - wait for signal to come back
 *          - if distance is 9 cm (middle of cell) there's a wall ahead
 *      - this class should give steering adjustment
 *
 * Tests:
 *      - ID front wall and stop
 *
 * Resources:
 * https://www.instructables.com/Arduino-Ultrasonic-Sensor-HC-sr04-LEDs-Distance-Me/
 * https://en.wikiversity.org/wiki/User:Dstaub/robotcar
 * https://arduinolearn.github.io/ultra.html
 *      - In order to generate the ultrasound, you need to set the Trig on a High State for 10 µs.
 *      - That will send out an 8 cycle sonic burst which will travel at the speed sound and will be received in the Echo pin.
 *      - The Echo pin will output the time of travel of the sound wave in microseconds.
 *
 * Datasheet:
 * https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 *
 * Maze Info:
 * The maze is composed of 18cm x 18cm unit squares
 * The unit squares are arranged to form a 16 x 16 unit grid.
 * The walls of the units of the maze are 5 cm high and 1.2 cm thick (assume 5% tolerancefor mazes).
 * An outside wall encloses the entire maze.
 */

class Sensors;
extern Sensors sensors;

class Sensors {
    public:
     bool see_front_wall;
     bool see_left_wall;
     bool see_right_wall;

     void init() {
         // front sensor
         m_fs_pin_1 = ULTRASONIC_TRIG;
         m_fs_pin_2 = ULTRASONIC_ECHO;
         pinMode(m_fs_pin_1, OUTPUT);
         pinMode(m_fs_pin_2, INPUT);

         // left sensor
         m_ls_pin = IR_1;
         pinMode(m_ls_pin, INPUT);

         // right sensor
         m_rs_pin = IR_2;
         pinMode(m_rs_pin, INPUT); // right sensor
     }

     boolean readInfrared(uint8_t pin) {
         return digitalRead(pin);
     }

     unsigned long readUltraSonic(uint8_t trigPin, uint8_t echoPin) {
         unsigned long duration;
         unsigned long distance;
         digitalWrite(trigPin, LOW);  //clears trigger pin
         delayMicroseconds(2);
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);                 // sets trigger pin HIGH for 10 us
         digitalWrite(echoPin, LOW);
         duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
         distance = (duration/2) / SPEED_OF_SOUND;
         return distance;
     }

     void update() {
        see_left_wall = readInfrared(m_ls_pin);
        see_right_wall = readInfrared(m_rs_pin);
        see_front_wall = readUltraSonic() > FRONT_THRESHOLD;
     }

    private:
     uint8_t m_fs_pin_1;
     uint8_t m_fs_pin_2;
     uint8_t m_ls_pin;
     uint8_t m_rs_pin;
};

#endif