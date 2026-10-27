#include <stdio.h>

#include "snippets.h"
#include "roam/blocks/geometry/chunky_tri_mesh.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"
#include "roam/blocks/search/flood_fill_nav_mesh.h"

namespace roam {

void run_block_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    Level &level = scene.level;
    FindNearestPolyResult spawn = {};
    nav_mesh_api::find_nearest_poly(spawn, nav_mesh, {-8, 0, 0}, {1, 1, 1}, scene.filter);

    /* SNIPPET_START: flood_fill */
    // every poly reachable from the seeds, and every one that is not: the polys to prune
    const NodeRef seeds[] = {spawn.node_ref};
    Array<NodeRef> unreachable = {};
    unreachable.reserve(nav_mesh_api::get_max_node_count(nav_mesh));
    FloodFillNavMeshResult flood = {};
    flood_fill_nav_mesh::flood_fill_nav_mesh(flood, nav_mesh, seeds, {}, unreachable);
    unreachable.resize(flood.unreachable_count);
    printf("%u reachable, %u not\n", flood.reachable_total, flood.unreachable_total);
    /* SNIPPET_END: flood_fill */

    unreachable.release();

    const float extra_positions[] = {12, 0, 0, 14, 0, 0, 14, 0, 2};
    const uint32_t extra_indices[] = {0, 2, 1};

    /* SNIPPET_START: merge */
    const InputTriangleMesh meshes[] = {
        {level.positions, level.indices},
        {extra_positions, extra_indices},
    };
    MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(meshes);
    const InputTriangleMesh input = {merged.positions, merged.indices};
    /* SNIPPET_END: merge */

    /* SNIPPET_START: chunky */
    ChunkyTriMesh chunks = chunky_tri_mesh::create(input, 256);

    uint32_t nodes[64];
    ChunkyTriMeshChunksResult found = {};
    chunky_tri_mesh::get_chunks_overlapping_rect(found, chunks, {-4, -4}, {4, 4}, nodes);
    for (uint32_t n = 0; n < found.chunk_count; n++) {
        const ChunkyTriMeshNode &node = chunks.nodes[nodes[n]];
        for (uint32_t i = 0; i < node.count; i++) {
            const uint32_t triangle = chunks.triangles[(uint32_t)node.index + i];
            (void)triangle;
        }
    }

    chunky_tri_mesh::release(chunks);
    /* SNIPPET_END: chunky */

    merge_positions_and_indices::release(merged);
}

}
