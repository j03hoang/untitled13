#ifndef UNTITLED13_SENSORS_H
#define UNTITLED13_SENSORS_H

#include "config.h"

// this class should give steering adjustment
// should tell if there's a wall

class Sensors {
    public:

     bool see_front_wall;
     bool see_left_wall;
     bool see_right_wall;

     void init() {
         pinMode(ULTRASONIC_TRIG_1, OUTPUT);
         pinMode(ULTRASONIC_ECHO_1, INPUT);
     }

     int getDistance() {
         long duration, distance;
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


    private:

};


#endif
