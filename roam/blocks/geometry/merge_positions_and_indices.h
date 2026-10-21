#pragma once

#include <stdint.h>

#include "roam/generate/input_triangle_mesh.h"
#include "roam/util/array.h"

namespace roam {

struct MergedMesh {
    Array<float> positions;
    Array<uint32_t> indices;
};

namespace merge_positions_and_indices {

MergedMesh merge_positions_and_indices(Span<const InputTriangleMesh> meshes);

void release(MergedMesh &mesh);

}

}
