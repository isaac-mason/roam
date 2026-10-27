#include <stdio.h>

#include "snippets.h"
#include "roam/util/memory.h"

using namespace roam;

int main() {
    run_memory_snippets();

    Scene scene = create_scene();
    run_span_snippets(scene);
    run_query_snippets(scene);
    run_agent_snippets(scene);
    run_generation_snippets(scene);
    run_area_snippets(scene);
    run_off_mesh_snippets(scene);
    run_advanced_snippets(scene);
    run_block_snippets(scene);
    run_debug_snippets(scene);
    release(scene);

    if (memory::tracking_enabled()) {
        MemoryStats stats = memory::outstanding();
        if (stats.allocations != 0) {
            printf("snippets leaked %lld allocations\n", (long long)stats.allocations);
            return 1;
        }
    }
    return 0;
}
