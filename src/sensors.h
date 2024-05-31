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
class Sensors {
    public:

     bool see_front_wall;
     bool see_left_wall;
     bool see_right_wall;

     /** not sure if this should be a const or a float
      * should it even be a const?
      * speed of sound at 20c = 343 m/s = 1/29.1 cm/s
      *
     */
     const float speedOfSound = 29.1;
    // sensor should probably be a parent class or something
     Sensors() {
        see_front_wall = false;
        see_left_wall = false;
        see_right_wall = false;
        initUltrasonicSensor();
     }


     void initUltrasonicSensor() {
         pinMode(ULTRASONIC_TRIG_1, OUTPUT);
         pinMode(ULTRASONIC_ECHO_1, INPUT);
     }

     /**
      * get Distance using Ultrasonic sensor
      * are we sure we want to return an int?
      * returns cm
      */
     unsigned long getDistance() {
         unsigned long duration;
         unsigned long distance;

         digitalWrite(ULTRASONIC_TRIG_1, LOW);  //clears trigger pin
         delayMicroseconds(2);
         digitalWrite(ULTRASONIC_TRIG_1, HIGH);
         delayMicroseconds(10);                 // sets trigger pin HIGH for 10 us
         digitalWrite(ULTRASONIC_ECHO_1, LOW);

         duration = pulseIn(ULTRASONIC_ECHO_1, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
         distance = (duration/2) / speedOfSound;
         // delay(100); // not sure this is necessary

         return distance;
    }
    /**
     * is it a wall?
     */
    bool do_i_see_a_wall_in_front() {
        if (getDistance() < 3) {
           return true;
        }
        return false;
    }

    private:
};

#endif

/**
* //shit happens here
void setup() {
    //init_imu();
    Serial.begin(9600);
    init_motor();
}

Sensors ultraSonic;
void loop(){
    //doGyro();
    Serial.println("My dick is"); Serial.println(ultraSonic.getDistance());
    Serial.println("centimeters long!");
    //do i see a wall?
    if (!ultraSonic.do_i_see_a_wall_in_front()){
       stopMotor();
       delay(100);
    } else {
        Serial.println("Full ahead Full");
        goForward();
        delay(100);
    }
}
*/