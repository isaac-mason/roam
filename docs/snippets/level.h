#pragma once

#include <stdint.h>

#include "roam/math/vec3.h"
#include "roam/util/array.h"

namespace roam {

struct Level {
    Array<float> positions;
    Array<uint32_t> indices;
};

Level create_level();

}
