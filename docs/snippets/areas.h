#pragma once

#include <stdint.h>

namespace roam {

/* SNIPPET_START: area_constants */
// the game's own area ids and flags: roam gives them no meaning
constexpr uint8_t AREA_GROUND = 0;
constexpr uint8_t AREA_WATER = 2;
constexpr uint32_t FLAG_WALK = 1;
constexpr uint32_t FLAG_SWIM = 2;
/* SNIPPET_END: area_constants */

}
