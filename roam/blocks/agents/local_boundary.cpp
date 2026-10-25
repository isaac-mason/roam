#include "roam/blocks/agents/local_boundary.h"

#include <float.h>

#include "roam/query/local_neighbourhood.h"

namespace roam {

namespace local_boundary {

LocalBoundary create() {
    LocalBoundary boundary = {};
    reset_local_boundary(boundary);
    return boundary;
}

void reset_local_boundary(LocalBoundary &boundary) {
    boundary.center = {FLT_MAX, FLT_MAX, FLT_MAX};
    boundary.segment_count = 0;
    boundary.poly_count = 0;
}

static float distance_pt_seg_sqr_2d(Vec3 point, Vec3 start, Vec3 end) {
    float pqx = end.x - start.x;
    float pqz = end.z - start.z;
    float dx = point.x - start.x;
    float dz = point.z - start.z;

    float d = pqx * pqx + pqz * pqz;
    float t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    float distance_x = point.x - (start.x + t * pqx);
    float distance_z = point.z - (start.z + t * pqz);
    return distance_x * distance_x + distance_z * distance_z;
}

static void add_segment_to_boundary(LocalBoundary &boundary, float distance_sqr, Vec3 start, Vec3 end) {
    uint32_t insert_index = 0;
    for (uint32_t i = 0; i < boundary.segment_count; i++) {
        if (distance_sqr <= boundary.segments[i].distance_sqr) {
            insert_index = i;
            break;
        }
        insert_index = i + 1;
    }

    if (boundary.segment_count >= MAX_LOCAL_SEGMENTS) {
        if (insert_index >= MAX_LOCAL_SEGMENTS) return;
        boundary.segment_count--;
    }

    for (uint32_t i = boundary.segment_count; i > insert_index; i--) boundary.segments[i] = boundary.segments[i - 1];
    boundary.segments[insert_index] = {start, end, distance_sqr};
    boundary.segment_count++;
}

constexpr uint32_t INLINE_WALL_SEGMENTS = 32;

void update_local_boundary(LocalBoundary &boundary, NodeRef node_ref, Vec3 position, float collision_query_range, const NavMesh &nav_mesh,
                           const QueryFilter &filter, QueryWorkspace &workspace) {
    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, node_ref)) {
        reset_local_boundary(boundary);
        return;
    }

    boundary.center = position;

    FindLocalNeighbourhoodResult neighbourhood;
    local_neighbourhood::find_local_neighbourhood(neighbourhood, workspace, nav_mesh, node_ref, position, collision_query_range, filter, boundary.polys);

    boundary.segment_count = 0;
    boundary.poly_count = 0;

    if (!neighbourhood.success) return;

    boundary.poly_count = neighbourhood.poly_count;

    float collision_query_range_sqr = collision_query_range * collision_query_range;

    WallSegment inline_segments[INLINE_WALL_SEGMENTS];
    Array<WallSegment> overflow_segments = {};
    for (uint32_t p = 0; p < boundary.poly_count; p++) {
        PolyWallSegmentsResult walls;
        Span<WallSegment> segments = inline_segments;
        local_neighbourhood::get_poly_wall_segments(walls, nav_mesh, boundary.polys[p], filter, false, segments);
        if (walls.truncated) {
            overflow_segments.resize(walls.segment_total);
            segments = overflow_segments;
            local_neighbourhood::get_poly_wall_segments(walls, nav_mesh, boundary.polys[p], filter, false, segments);
        }
        if (walls.success) {
            for (uint32_t k = 0; k < walls.segment_count; k++) {
                Vec3 start = segments.data[k].start;
                Vec3 end = segments.data[k].end;

                float distance_sqr = distance_pt_seg_sqr_2d(position, start, end);
                if (distance_sqr > collision_query_range_sqr) continue;

                add_segment_to_boundary(boundary, distance_sqr, start, end);
            }
        }
    }
    overflow_segments.release();
}

bool is_local_boundary_valid(const LocalBoundary &boundary, const NavMesh &nav_mesh, const QueryFilter &filter) {
    if (boundary.poly_count == 0) return false;

    for (uint32_t i = 0; i < boundary.poly_count; i++) {
        NodeRef poly_ref = boundary.polys[i];
        if (!nav_mesh_api::is_valid_node_ref(nav_mesh, poly_ref)) return false;
        if (!nav_mesh_api::passes_filter(filter, poly_ref, nav_mesh)) return false;
    }

    return true;
}

}

}
