#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh.h"
#include "roam/util/span.h"

namespace roam {

constexpr uint32_t NAV_MESH_TILE_FORMAT_VERSION = 1;
constexpr uint32_t NAV_MESH_FORMAT_VERSION = 1;

struct SaveResult {
    bool success;
    uint32_t byte_count;
    uint32_t byte_total;
    bool truncated;
};

enum LoadStatus : uint32_t {
    LOAD_STATUS_OK = 0,
    LOAD_STATUS_BAD_MAGIC = 1,
    LOAD_STATUS_UNSUPPORTED_VERSION = 2,
    LOAD_STATUS_TRUNCATED = 3,
    LOAD_STATUS_INVALID_DATA = 4,
};

namespace nav_mesh_io {

void save_tile(SaveResult &out, const NavMeshTile &tile, Span<uint8_t> bytes);
LoadStatus load_tile(NavMeshTile &tile, Span<const uint8_t> bytes);

void save_nav_mesh(SaveResult &out, const NavMesh &nav_mesh, Span<uint8_t> bytes);
LoadStatus load_nav_mesh(NavMesh &nav_mesh, Span<const uint8_t> bytes);

}

}
