#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

struct FindLocalNeighbourhoodResult {
    bool success;
    uint32_t poly_count;
    uint32_t poly_total;
    bool truncated;
};

struct WallSegment {
    Vec3 start;
    Vec3 end;
};

struct PolyWallSegmentsResult {
    bool success;
    uint32_t segment_count;
    uint32_t segment_total;
    bool truncated;
};

namespace local_neighbourhood {

void find_local_neighbourhood(FindLocalNeighbourhoodResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                              Vec3 position, float radius, const QueryFilter &filter, Span<NodeRef> polys);

void get_poly_wall_segments(PolyWallSegmentsResult &out, const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals,
                            Span<WallSegment> segments, Span<NodeRef> segment_refs = {});

}

}
