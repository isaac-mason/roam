#pragma once

#include <stdint.h>

#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

struct FindLocalNeighbourhoodResult {
    bool success;
    Array<NodeRef> node_refs;
};

struct PolyWallSegmentsResult {
    bool success;
    Array<float> segment_vertices;
    Array<NodeRef> segment_refs;
};

namespace local_neighbourhood {

FindLocalNeighbourhoodResult find_local_neighbourhood(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 position, float radius,
                                                      const QueryFilter &filter, SearchWorkspace *workspace = nullptr);
void release(FindLocalNeighbourhoodResult &result);

PolyWallSegmentsResult get_poly_wall_segments(const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals);
void get_poly_wall_segments_into(const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals,
                                 PolyWallSegmentsResult &result);
void release(PolyWallSegmentsResult &result);

}

}
