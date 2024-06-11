#ifndef UNTITLED13_MOUSE_H
#define UNTITLED13_MOUSE_H

#include <Arduino.h>
#include "config.h"
#include "maze.h"
#include "sensors.h"
#include "motion.h"

class Mouse {
    public:
     void followTo(Location target) {
         m_location = START;
         m_heading = NORTH;

         while (m_location != target) {
             sensors.update();
             updateMap();

             if (!sensors.see_right_wall) {
                 turnRight();
                 m_location = m_location.east();
             } else if (!sensors.see_front_wall) {
                 moveAhead();
                 m_location = m_location.north();
             } else if (!sensors.see_left_wall) {
                 turnLeft();
                 m_location = m_location.west();
             } else {
                 turnBack();
                 m_location = m_location.south();
             }
             m_location = m_location.neighbour(m_heading);
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
        motion.moveAhead();
     }

     void turnLeft() {
        motion.turnLeft();
        m_heading = leftFrom(m_heading);
     }

     void turnRight() {
        motion.turnRight();
        m_heading = rightFrom(m_heading);
     }

     void turnBack() {
        motion.moveBack();
        m_heading = behindFrom(m_heading);
     }


    private:
     Location m_location;
     Heading m_heading;

};


#endif
