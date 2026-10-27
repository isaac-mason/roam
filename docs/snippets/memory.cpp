#include <stdio.h>
#include <stdlib.h>

#include "snippets.h"
#include "roam/util/scoped.h"

namespace roam {

/* SNIPPET_START: counting_allocator */
struct CountingAllocator {
    size_t bytes;
    size_t peak;
};

void *counting_allocate(size_t size, size_t alignment, void *user_data) {
    (void)alignment;
    CountingAllocator &counter = *static_cast<CountingAllocator *>(user_data);
    counter.bytes += size;
    if (counter.bytes > counter.peak) counter.peak = counter.bytes;
    return malloc(size);
}

void counting_free(void *block, size_t size, size_t alignment, void *user_data) {
    (void)alignment;
    static_cast<CountingAllocator *>(user_data)->bytes -= size;
    free(block);
}
/* SNIPPET_END: counting_allocator */

void run_memory_snippets() {
    /* SNIPPET_START: set_allocator */
    static CountingAllocator counter = {};
    memory::set_allocator({counting_allocate, counting_free, &counter});
    /* SNIPPET_END: set_allocator */

    const SoloNavMeshOptions options = scene_options();
    Level level = create_level();

    /* SNIPPET_START: scoped */
    {
        Scoped<NavMesh> nav_mesh(nav_mesh_api::release);
        const InputTriangleMesh mesh = {level.positions, level.indices};
        SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(mesh, options);
        generate_solo_nav_mesh::release(result.intermediates);
        *nav_mesh = result.nav_mesh;
        printf("%u tiles\n", nav_mesh->tiles.length);
    }
    /* SNIPPET_END: scoped */

    level.positions.release();
    level.indices.release();

    /* SNIPPET_START: memory_stats */
    printf("%zu bytes in use, %zu at most\n", counter.bytes, counter.peak);
    if (memory::tracking_enabled()) {
        MemoryStats stats = memory::outstanding();
        printf("%lld allocations, %lld bytes\n", (long long)stats.allocations, (long long)stats.bytes);
    }
    /* SNIPPET_END: memory_stats */

    memory::reset_allocator();
}

void run_span_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;
    FindNearestPolyResult start = {};
    nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
    FindNearestPolyResult end = {};
    nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, {1, 1, 1}, filter);

    /* SNIPPET_START: spans */
    // a C array: its size is the span's length
    NodeRef fixed[256];
    FindNodePathResult path = {};
    nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref,
                                    start.position, end.position, filter, fixed);

    // an Array: a writable span covers its capacity, so reserve, pass it, then resize to what
    // was written
    Array<NodeRef> grown = {};
    grown.reserve(16);
    nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref,
                                    start.position, end.position, filter, grown);
    grown.resize(path.path_count);

    // a pointer and a length, and a read-only span of the refs written so far
    NodeRef *heap = memory::allocate_array<NodeRef>(256);
    nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref,
                                    start.position, end.position, filter, {heap, 256});
    Span<const NodeRef> written = {heap, path.path_count};
    /* SNIPPET_END: spans */

    (void)written;
    grown.release();
    memory::free_array(heap, 256);
}

}
