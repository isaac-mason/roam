#pragma once

#include <stdint.h>

namespace roam {

constexpr int MESH_NULL_IDX = -1;
constexpr int BORDER_VERTEX = 0x10000;
constexpr int CONTOUR_REG_MASK = 0xffff;
constexpr int AREA_BORDER = 0x20000;

constexpr uint8_t NULL_AREA = 0;
constexpr uint8_t WALKABLE_AREA = 1;
constexpr int BORDER_REG = 0x8000;

constexpr int NOT_CONNECTED = 0x3f;
constexpr int MAX_HEIGHT = 0xffff;

namespace common {

constexpr int DIR_OFFSETS[4][2] = {
    {-1, 0},
    {0, 1},
    {1, 0},
    {0, -1},
};

inline int get_dir_offset_x(int direction) { return DIR_OFFSETS[direction & 0x03][0]; }

inline int get_dir_offset_y(int direction) { return DIR_OFFSETS[direction & 0x03][1]; }

inline int get_dir_for_offset(int x, int y) {
    for (int i = 0; i < 4; i++) {
        if (DIR_OFFSETS[i][0] == x && DIR_OFFSETS[i][1] == y) return i;
    }
    return 0;
}

constexpr int AXIS_X = 0;
constexpr int AXIS_Y = 1;
constexpr int AXIS_Z = 2;

constexpr int MULTIPLE_REGS = 0;
constexpr int MAX_LAYERS = NOT_CONNECTED - 1;

}

}
