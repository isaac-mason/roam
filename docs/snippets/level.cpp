#include "level.h"

namespace roam {

/* SNIPPET_START: level */
// a box's top and four sides, wound to face out
static void add_box(Array<float> &positions, Array<uint32_t> &indices, Vec3 min, Vec3 max) {
    uint32_t base = positions.length / 3;
    for (int i = 0; i < 8; i++) {
        positions.push((i & 1) ? max.x : min.x);
        positions.push((i & 2) ? max.y : min.y);
        positions.push((i & 4) ? max.z : min.z);
    }
    const uint32_t faces[] = {
        2, 6, 7, 2, 7, 3,
        0, 2, 3, 0, 3, 1,
        4, 5, 7, 4, 7, 6,
        0, 4, 6, 0, 6, 2,
        1, 3, 7, 1, 7, 5,
    };
    for (uint32_t index : faces) indices.push(base + index);
}

Level create_level() {
    Level level = {};
    const float floor[] = {-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10};
    for (float value : floor) level.positions.push(value);
    const uint32_t floor_indices[] = {0, 2, 1, 0, 3, 2};
    for (uint32_t index : floor_indices) level.indices.push(index);
    // a block in the middle to walk around, and a platform 1 m up that the floor does not reach
    add_box(level.positions, level.indices, {-2, 0, -2}, {2, 2, 2});
    add_box(level.positions, level.indices, {4, 0, -9}, {9, 1, -5});
    return level;
}
/* SNIPPET_END: level */

}
