#pragma once

#include "roam/query/nav_mesh_io.h"
#include "tests/harness/random.h"

using namespace roam;

inline bool snapshot_is_valid(const NavMesh &nav_mesh) {
    SaveResult size = {};
    nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);
    NavMesh loaded = {};
    bool valid = saved.success && nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_OK;
    if (valid) nav_mesh_api::release(loaded);
    bytes.release();
    return valid;
}
