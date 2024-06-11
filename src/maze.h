/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef UNTITLED13_MAZE_H
#define UNTITLED13_MAZE_H

#include <stdint.h>

#define START Location{0, 0}

/**
 * The Maze class holds the map of the maze and the state of all four walls in each cell
 *
 * Maze Info:
 * The maze is composed of 18cm x 18cm unit squares
 * The unit squares are arranged to form a 16 x 16 unit grid.
 * The walls of the units of the maze are 5 cm high and 1.2 cm thick.
 * An outside wall encloses the entire maze.
 */

const int MAZE_WIDTH = 16;
const int MAZE_HEIGHT = 16;
const int MAZE_CELL_COUNT = (MAZE_WIDTH * MAZE_HEIGHT);

/**
 *
 */
enum WallState : uint8_t {
    EXIT = 0,     // a wall that has been seen and confirmed absent
    WALL = 1,     // a wall that has been seen and confirmed present
    UNKNOWN = 2,  // a wall that has not yet been seen
};

/**
 *
 */
struct MazeCell {
    WallState north = static_cast<WallState>(2);
    WallState east = static_cast<WallState>(2);
    WallState south = static_cast<WallState>(2);
    WallState west = static_cast<WallState>(2);
};

/**
 * A Heading represents one of the four cardinal directions and will be used to determine
 * which direction the Mouse intends to go
 *
 * TODO: expand list for diagonals
 */
enum Heading {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    HEADING_COUNT = 4
};

inline Heading rightFrom(const Heading heading) {
    return static_cast<Heading> (heading + 1 % HEADING_COUNT);
}

inline Heading leftFrom(const Heading heading) {
    return static_cast<Heading> (heading - 1 % HEADING_COUNT);
}

inline Heading aheadFrom(const Heading heading) {
    return heading;
}

inline Heading behindFrom(const Heading heading) {
    return static_cast<Heading> (heading - 2 % HEADING_COUNT);
}


/**
 * Position within the maze stored as a (x, y) pair
 */
class Location {
    public:
     uint8_t x;
     uint8_t y;

     Location() : x(0), y(0) {};
     Location(uint8_t x, uint8_t y) : x(x), y(y) {};

    bool operator==(const Location &obj) const {
        return x == obj.x && y == obj.y;
    }

    bool operator!=(const Location &obj) const {
        return x != obj.x || y != obj.y;
    }

    Location north() const {
        return {x, static_cast<uint8_t>((y + 1) % MAZE_HEIGHT)};
    }

    Location east() const {
        return {static_cast<uint8_t>((x + 1) % MAZE_WIDTH), y};
    }

    Location south() const {
        return {x, static_cast<uint8_t>((y + MAZE_HEIGHT - 1) % MAZE_HEIGHT)};
    }

    Location west() const {
        return {static_cast<uint8_t>((x + MAZE_WIDTH - 1) % MAZE_WIDTH), y};
    }

    Location neighbour(const Heading heading) const {
        switch (heading) {
            case NORTH:
                return north();
                break;
            case EAST:
                return east();
                break;
            case SOUTH:
                return south();
                break;
            case WEST:
                return west();
                break;
            default:
                return *this;  // this is actually an error and should be handled
                break;
        }
    }
};

/**
 * The two main data blocks in the class store the wall state of every cell
 * and a cost associated with every cell after the maze is flooded.
 *
 * As the mouse searches the maze updateWallState is called to record changes.
 */
class Maze {
    public:
     Maze() {
         init();
     }

     // set an empty maze with border walls. The mouse should begin in a corner
     // and head north from there. It is assumed S-E-W in the starting cell is a wall
     void init() {
        for (int x = 0; x < MAZE_HEIGHT; x++) {
            for (int y = 0; y < MAZE_WIDTH; y++) {
                m_walls[x][y].north = UNKNOWN;
                m_walls[x][y].east = UNKNOWN;
                m_walls[x][y].south = UNKNOWN;
                m_walls[x][y].west = UNKNOWN;
            }
        }

         for (int x = 0; x < MAZE_WIDTH; x++) {
             m_walls[x][0].south = WALL;
             m_walls[x][MAZE_HEIGHT - 1].north = WALL;
         }

         for (int y = 0; y < MAZE_HEIGHT; y++) {
             m_walls[0][y].west = WALL;
             m_walls[MAZE_WIDTH - 1][y].east = WALL;
         }

         setWallState(START, EAST, WALL);
         setWallState(START, NORTH, EXIT);

     }

     // // This is what you use when exploring. Once seen, a wall should not be changed again.
     void updateWallState(Location cell, Heading heading, WallState state) {
         switch (heading) {
             case NORTH:
                 if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN)
                     return;
                 break;
             case EAST:
                 if ((m_walls[cell.x][cell.y].east & UNKNOWN) != UNKNOWN)
                     return;
                 break;
             case WEST:
                 if ((m_walls[cell.x][cell.y].west & UNKNOWN) != UNKNOWN)
                     return;
                 break;
             case SOUTH:
                 if ((m_walls[cell.x][cell.y].south & UNKNOWN) != UNKNOWN)
                     return;
                 break;
         }
         setWallState(cell, heading, state);
     }

     // unconditionally set a wall state
     void setWallState(Location loc, Heading heading, WallState state) {
        switch (heading) {
            case NORTH:
                m_walls[loc.x][loc.y].north = state;
                m_walls[loc.north().x][loc.north().y].south = state;
                break;
            case EAST:
                m_walls[loc.x][loc.y].east = state;
                m_walls[loc.east().x][loc.east().y].west = state;
                break;
            case SOUTH:
                m_walls[loc.x][loc.y].south = state;
                m_walls[loc.south().x][loc.south().y].north = state;
                break;
            case WEST:
                m_walls[loc.x][loc.y].west = state;
                m_walls[loc.west().x][loc.east().y].east = state;
                break;
        }
     }

     Location goal() {
         return GOAL;
     }

    private:
     Location GOAL{0 , 0};
     MazeCell m_walls[MAZE_WIDTH][MAZE_HEIGHT];
     uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT]; // TODO
};

extern Maze maze;

#endif
