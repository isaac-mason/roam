#pragma once

#include <stdint.h>

#include "roam/math/vec3.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

constexpr uint32_t MAX_LOCAL_SEGMENTS = 8;
constexpr uint32_t MAX_LOCAL_POLYS = 16;

struct LocalBoundarySegment {
    Vec3 start;
    Vec3 end;
    float distance_sqr;
};

struct LocalBoundary {
    Vec3 center;
    LocalBoundarySegment segments[MAX_LOCAL_SEGMENTS];
    uint32_t segment_count;
    NodeRef polys[MAX_LOCAL_POLYS];
    uint32_t poly_count;
};

namespace local_boundary {

LocalBoundary create();

void reset_local_boundary(LocalBoundary &boundary);

void update_local_boundary(LocalBoundary &boundary, NodeRef node_ref, Vec3 position, float collision_query_range, const NavMesh &nav_mesh,
                           const QueryFilter &filter, QueryWorkspace &workspace);

bool is_local_boundary_valid(const LocalBoundary &boundary, const NavMesh &nav_mesh, const QueryFilter &filter);

}

}
