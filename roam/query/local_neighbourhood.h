#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

struct FindLocalNeighbourhoodResult {
    bool success;
    uint32_t node_ref_count;
    uint32_t full_node_ref_count;
};

struct PolyWallSegmentsResult {
    bool success;
    uint32_t segment_count;
    uint32_t full_segment_count;
};

namespace local_neighbourhood {

void find_local_neighbourhood(FindLocalNeighbourhoodResult &out, SearchWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                              Vec3 position, float radius, const QueryFilter &filter, NodeRef *node_refs, uint32_t max_node_refs);

void get_poly_wall_segments(PolyWallSegmentsResult &out, const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals,
                            float *segment_vertices, NodeRef *segment_refs, uint32_t max_segments);

}

}
