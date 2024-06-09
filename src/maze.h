#ifndef UNTITLED13_MAZE_H
#define UNTITLED13_MAZE_H

#include <stdint.h>

#define START Location{0, 0}

const int MAZE_WIDTH = 16;
const int MAZE_HEIGHT = 16;
const int MAZE_CELL_COUNT = (MAZE_WIDTH * MAZE_HEIGHT);

enum WallState : uint8_t {
    EXIT = 0,     // a wall that has been seen and confirmed absent
    WALL = 1,     // a wall that has been seen and confirmed present
    UNKNOWN = 2,  // a wall that has not yet been seen
    VIRTUAL = 3,  // a wall that has not yet been seen
};

struct MazeCell {
    WallState north = static_cast<WallState>(2);
    WallState east = static_cast<WallState>(2);
    WallState south = static_cast<WallState>(2);
    WallState west = static_cast<WallState>(2);
};

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
};

class Maze;
extern Maze maze;

class Maze {
    public:
     Maze() {
         init();
     }

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
};


#endif
