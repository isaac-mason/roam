#pragma once

#include <stdint.h>

#include "roam/util/array.h"

namespace roam {

struct MeshView {
    const float *positions;
    const uint32_t *indices;
    uint32_t index_count;
};

struct MergedMesh {
    Array<float> positions;
    Array<uint32_t> indices;
};

namespace merge_positions_and_indices {

MergedMesh merge_positions_and_indices(const MeshView *meshes, uint32_t mesh_count);

void release(MergedMesh &mesh);

}

}
