#ifndef UNTITLED13_MOUSE_H
#define UNTITLED13_MOUSE_H

#include <Arduino.h>
#include "config.h"
#include "maze.h"
#include "sensors.h"

class Mouse {
    public:
     void followTo(Location target) {
         m_location = START;
         m_heading = NORTH;

         while (m_location != target) {
             updateMap();

             if (!sensors.see_right_wall) {
                 turnLeft();
             } else if (!sensors.see_front_wall) {
                 moveAhead();
             } else if (!sensors.see_left_wall) {
                 turnRight();
             } else {
                 turnBack();
             }
         }
     }

     void updateMap() {

     }

     void moveAhead() {

     }

     void turnLeft() {

     }

     void turnRight() {

     }

     void turnBack() {

     }


    private:
     Location m_location;
     Heading m_heading;

};


#endif
