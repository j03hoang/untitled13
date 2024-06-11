#ifndef UNTITLED13_SENSORS_H
#define UNTITLED13_SENSORS_H

#include "config.h"

/* Using 2 Ultrasonic Sensors and a photo resistor paired with an led
 *      - Ultrasonic Sensor has 15 degree range
 *
 * TODO:
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
 *      - That will send out an 8 cycle sonic burst which will travel at the speed sound
 *      and will be received in the Echo pin.
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
         m_fs_pin_out = ULTRASONIC_TRIG_A;
         m_fs_pin_in = ULTRASONIC_ECHO_A;
         pinMode(m_fs_pin_out, OUTPUT);
         pinMode(m_fs_pin_in, INPUT);

         // left sensor
         m_ls_pin_out = ULTRASONIC_TRIG_B;
         m_ls_pin_in = ULTRASONIC_ECHO_B;
         pinMode(m_ls_pin_out, OUTPUT);
         pinMode(m_ls_pin_in, INPUT);

         // right sensor
         m_rs_pin_out = ULTRASONIC_TRIG_C;
         m_rs_pin_in = ULTRASONIC_ECHO_C;
         pinMode(m_rs_pin_out, OUTPUT);
         pinMode(m_rs_pin_in, INPUT);
     }

     static unsigned long readUltraSonic(uint8_t trigPin, uint8_t echoPin) {
         unsigned long duration;
         unsigned long distance;
         digitalWrite(trigPin, LOW);  //clears trigger pin
         delayMicroseconds(2);
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);             // sets trigger pin HIGH for 10 us
         digitalWrite(echoPin, LOW);
         duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
         distance = (duration/2) / SPEED_OF_SOUND;
         return distance;
     }

     void update() {
         unsigned long front = readUltraSonic(m_fs_pin_out, m_fs_pin_in);
         unsigned long left = readUltraSonic(m_ls_pin_out, m_ls_pin_in);
         unsigned long right = readUltraSonic(m_rs_pin_out, m_rs_pin_in);
         see_left_wall = left < LEFT_THRESHOLD;
         see_right_wall = right < RIGHT_THRESHOLD;
         see_front_wall = front < FRONT_THRESHOLD;
     }

    private:
     uint8_t m_fs_pin_out;
     uint8_t m_fs_pin_in;
     uint8_t m_ls_pin_out;
     uint8_t m_ls_pin_in;
     uint8_t m_rs_pin_out;
     uint8_t m_rs_pin_in;
};

#endif