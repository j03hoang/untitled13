#ifndef UNTITLED13_MAZE_H
#define UNTITLED13_MAZE_H

#include <stdint.h>

enum WallState {
    EXIT = 0,     // a wall that has been seen and confirmed absent
    WALL = 1,     // a wall that has been seen and confirmed present
    UNKNOWN = 2,  // a wall that has not yet been seen
    VIRTUAL = 3,  // a wall that has not yet been seen
};

struct MazeCell {
    WallState north : 2;
    WallState east : 2;
    WallState south : 2;
    WallState west : 2;
};

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

class Location {
    public:
     uint8_t m_x;
     uint8_t m_y;

     Location() : m_x(0), m_y(0) {};
     Location(uint8_t x, uint8_t y) : m_x(x), m_y(y) {};

    bool operator==(const Location &obj) const {
        return m_x == obj.m_x && m_y == obj.m_y;
    }

    bool operator!=(const Location &obj) const {
        return m_x != obj.m_x || m_y != obj.m_y;
    }
};

class Maze {

};


#endif
