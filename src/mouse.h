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
         bool frontWall = sensors.see_front_wall;
         bool leftWall = sensors.see_left_wall;
         bool rightWall = sensors.see_right_wall;

         switch (m_heading) {
             case NORTH:
                 maze.updateWallState(m_location, NORTH, frontWall ? WALL : EXIT);
                 maze.updateWallState(m_location, EAST, rightWall ? WALL : EXIT);
                 maze.updateWallState(m_location, WEST, leftWall ? WALL : EXIT);
                 break;
             case EAST:
                 maze.updateWallState(m_location, EAST, frontWall ? WALL : EXIT);
                 maze.updateWallState(m_location, SOUTH, rightWall? WALL : EXIT);
                 maze.updateWallState(m_location, NORTH, leftWall? WALL : EXIT);
                 break;
             case SOUTH:
                 maze.updateWallState(m_location, SOUTH, frontWall ? WALL : EXIT);
                 maze.updateWallState(m_location, WEST, rightWall ? WALL : EXIT);
                 maze.updateWallState(m_location, EAST, leftWall ? WALL : EXIT);
                 break;
             case WEST:
                 maze.updateWallState(m_location, WEST, frontWall ? WALL : EXIT);
                 maze.updateWallState(m_location, NORTH, rightWall ? WALL : EXIT);
                 maze.updateWallState(m_location, SOUTH, leftWall ? WALL : EXIT);
                 break;
         }

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
