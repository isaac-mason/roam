#include "roam/blocks/geometry/merge_positions_and_indices.h"

#include <string.h>

#include "roam/util/map.h"

namespace roam {

namespace merge_positions_and_indices {

static uint32_t position_bits(float value) {
    if (value == 0.0f) value = 0.0f;
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

MergedMesh merge_positions_and_indices(Span<const InputTriangleMesh> meshes) {
    MergedMesh merged = {};

    Map<uint64_t, int32_t> first_by_hash = {};
    Array<int32_t> next_with_same_hash = {};
    Array<uint32_t> merged_bits = {};

    for (const InputTriangleMesh &mesh : meshes) {
        for (uint32_t i = 0; i < mesh.indices.capacity; i++) {
            const float *position = &mesh.positions.data[mesh.indices[i] * 3];
            uint32_t x = position_bits(position[0]);
            uint32_t y = position_bits(position[1]);
            uint32_t z = position_bits(position[2]);
            uint64_t hash = hash_key(((uint64_t)x << 32) | y) ^ z;

            int32_t *first = first_by_hash.get(hash);
            int32_t index = first ? *first : -1;
            while (index >= 0) {
                const uint32_t *bits = &merged_bits[(uint32_t)index * 3];
                if (bits[0] == x && bits[1] == y && bits[2] == z) break;
                index = next_with_same_hash[(uint32_t)index];
            }

            if (index < 0) {
                index = (int32_t)(merged.positions.length / 3);
                merged.positions.push(position[0]);
                merged.positions.push(position[1]);
                merged.positions.push(position[2]);
                merged_bits.push(x);
                merged_bits.push(y);
                merged_bits.push(z);
                next_with_same_hash.push(first ? *first : -1);
                first_by_hash.set(hash, index);
            }

            merged.indices.push((uint32_t)index);
        }
    }

    first_by_hash.release();
    next_with_same_hash.release();
    merged_bits.release();
    return merged;
}

void release(MergedMesh &mesh) {
    mesh.positions.release();
    mesh.indices.release();
}

}

}
