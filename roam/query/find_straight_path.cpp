#include "roam/query/find_straight_path.h"

#include "roam/geometry/geometry.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

namespace find_straight_path {

enum AppendVertexStatus : uint32_t {
    APPEND_VERTEX_SUCCESS = 1 << 0,
    APPEND_VERTEX_MAX_POINTS_REACHED = 1 << 1,
    APPEND_VERTEX_IN_PROGRESS = 1 << 2,
};

struct PointBuffer {
    StraightPathPoint *data;
    uint32_t capacity;
    uint32_t length;

    StraightPathPoint &back() { return data[length - 1]; }
    void push(const StraightPathPoint &point) { data[length++] = point; }
};

static uint32_t append_vertex(Vec3 position, NodeRef node_ref, uint32_t flags, PointBuffer &out_points, NodeType node_type,
                              int max_points) {
    if (out_points.length > 0 && vec3::equals(out_points.back().position, position)) {
        StraightPathPoint &last = out_points.back();
        if (last.type == NODE_TYPE_POLY && node_type == NODE_TYPE_POLY) {
            last.node_ref = node_ref;
            last.type = node_type;
            return APPEND_VERTEX_IN_PROGRESS;
        }
    }

    out_points.push({position, node_type, node_ref, flags});

    if (out_points.length >= (uint32_t)max_points) return APPEND_VERTEX_SUCCESS | APPEND_VERTEX_MAX_POINTS_REACHED;

    if (flags & STRAIGHT_PATH_POINT_END) return APPEND_VERTEX_SUCCESS;

    return APPEND_VERTEX_IN_PROGRESS;
}

static uint32_t append_portals(const NavMesh &nav_mesh, int start_index, int end_index, Vec3 end_position, const NodeRef *path,
                               PointBuffer &out_points, uint32_t options, int max_points) {
    Vec3 start_position = out_points.back().position;

    for (int i = start_index; i < end_index; i++) {
        NodeRef from = path[i];
        NodeRef to = path[i + 1];

        if (options & FIND_STRAIGHT_PATH_AREA_CROSSINGS) {
            if (nav_mesh_api::get_node_by_ref(nav_mesh, from).area == nav_mesh_api::get_node_by_ref(nav_mesh, to).area) continue;
        }

        GetPortalPointsResult portal;
        nav_mesh_search::get_portal_points_unchecked(portal, nav_mesh, from, to);
        if (!portal.success) break;

        geometry::IntersectSegSeg2dResult intersection = geometry::intersect_seg_seg_2d(start_position, end_position, portal.left, portal.right);
        if (!intersection.hit) continue;

        Vec3 point = vec3::lerp(portal.left, portal.right, intersection.t);

        uint32_t status = append_vertex(point, to, 0, out_points, node::get_node_ref_type(to), max_points);
        if (status != APPEND_VERTEX_IN_PROGRESS) return status;
    }

    return APPEND_VERTEX_IN_PROGRESS;
}

static FindStraightPathResult make_result(uint32_t flags, const PointBuffer &path) {
    bool ended = path.length > 0 && (path.data[path.length - 1].flags & STRAIGHT_PATH_POINT_END) != 0;
    bool truncated = (flags & FIND_STRAIGHT_PATH_RESULT_MAX_POINTS_REACHED) != 0 && !ended;
    return {(flags & FIND_STRAIGHT_PATH_RESULT_SUCCESS) != 0, flags, path.length, truncated};
}

static FindStraightPathResult stopped(uint32_t status, uint32_t base_flags, const PointBuffer &path) {
    uint32_t flags = base_flags;
    if (status & APPEND_VERTEX_MAX_POINTS_REACHED) flags |= FIND_STRAIGHT_PATH_RESULT_MAX_POINTS_REACHED;
    return make_result(flags, path);
}

static const uint32_t ANY_CROSSINGS = FIND_STRAIGHT_PATH_AREA_CROSSINGS | FIND_STRAIGHT_PATH_ALL_CROSSINGS;
static const uint32_t PARTIAL = FIND_STRAIGHT_PATH_RESULT_SUCCESS | FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH;
static const uint32_t INVALID = FIND_STRAIGHT_PATH_RESULT_NONE | FIND_STRAIGHT_PATH_RESULT_INVALID_INPUT;

static FindStraightPathResult find_straight_path_in(const NavMesh &nav_mesh, Vec3 start, Vec3 end, const NodeRef *path_node_refs,
                                                   uint32_t path_node_count, uint32_t straight_path_options, PointBuffer &path) {
    path.length = 0;
    int max_points = (int)path.capacity;

    if (!vec3::finite(start) || !vec3::finite(end) || path_node_count == 0 || path.capacity == 0) return make_result(INVALID, path);

    GetClosestPointOnPolyBoundaryResult closest_start = {};
    nav_mesh_api::get_closest_point_on_poly_boundary(closest_start, nav_mesh, path_node_refs[0], start);
    if (!closest_start.success) return make_result(INVALID, path);

    GetClosestPointOnPolyBoundaryResult closest_end = {};
    nav_mesh_api::get_closest_point_on_poly_boundary(closest_end, nav_mesh, path_node_refs[path_node_count - 1], end);
    if (!closest_end.success) return make_result(INVALID, path);

    Vec3 closest_start_position = closest_start.position;
    Vec3 closest_end_position = closest_end.position;

    uint32_t start_status = append_vertex(closest_start_position, path_node_refs[0], STRAIGHT_PATH_POINT_START, path,
                                          node::get_node_ref_type(path_node_refs[0]), max_points);
    if (start_status != APPEND_VERTEX_IN_PROGRESS) return stopped(start_status, PARTIAL, path);

    int path_size = (int)path_node_count;
    int valid_count = 0;
    while (valid_count < path_size && nav_mesh_api::is_valid_node_ref(nav_mesh, path_node_refs[valid_count])) valid_count++;

    if (path_size > 1) {
        Vec3 portal_apex = closest_start_position;
        Vec3 portal_left = portal_apex;
        Vec3 portal_right = portal_apex;

        int apex_index = 0;
        int left_index = 0;
        int right_index = 0;

        NodeRef left_node_ref = path_node_refs[0];
        NodeRef right_node_ref = path_node_refs[0];
        NodeType left_node_type = NODE_TYPE_POLY;
        NodeType right_node_type = NODE_TYPE_POLY;

        for (int i = 0; i < path_size; ++i) {
            NodeType to_type = NODE_TYPE_POLY;
            Vec3 left;
            Vec3 right;

            if (i + 1 < path_size) {
                NodeRef to_ref = path_node_refs[i + 1];
                to_type = node::get_node_ref_type(to_ref);

                GetPortalPointsResult portal = {};
                if (i + 1 < valid_count) nav_mesh_search::get_portal_points_unchecked(portal, nav_mesh, path_node_refs[i], to_ref);
                left = portal.left;
                right = portal.right;
                if (!portal.success) {
                    GetClosestPointOnPolyBoundaryResult end_clamp = {};
                    nav_mesh_api::get_closest_point_on_poly_boundary(end_clamp, nav_mesh, path_node_refs[i], end);
                    if (!end_clamp.success) return make_result(INVALID, path);

                    if (straight_path_options & ANY_CROSSINGS) {
                        append_portals(nav_mesh, apex_index, i, end_clamp.position, path_node_refs, path, straight_path_options, max_points);
                    }

                    append_vertex(end_clamp.position, path_node_refs[i], 0, path, node::get_node_ref_type(path_node_refs[i]), max_points);

                    return make_result(PARTIAL, path);
                }

                if (i == 0 && to_type == NODE_TYPE_POLY) {
                    if (geometry::distance_pt_seg_sqr_2d(portal_apex, left, right).dist_sqr < 1e-6f) continue;
                }

                if (to_type == NODE_TYPE_OFFMESH) {
                    OffMeshCrossing crossing;
                    nav_mesh_api::get_off_mesh_crossing(crossing, nav_mesh, path_node_refs[i], to_ref, portal_apex);
                    Vec3 off_mesh_start = crossing.start;
                    Vec3 off_mesh_end = crossing.end;
                    NodeRef to_poly_ref = crossing.end_poly_ref;

                    if (straight_path_options & ANY_CROSSINGS) {
                        uint32_t status =
                            append_portals(nav_mesh, apex_index, i, off_mesh_start, path_node_refs, path, straight_path_options, max_points);
                        if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, PARTIAL, path);
                    }

                    Vec3 last_point_added = path.back().position;
                    if (!vec3::equals(last_point_added, portal_right) && !vec3::equals(last_point_added, portal_left)) {
                        float right_area = geometry::tri_area_2d(last_point_added, portal_right, off_mesh_start);
                        float left_area = geometry::tri_area_2d(last_point_added, portal_left, off_mesh_start);
                        uint32_t status = 0;

                        if (right_area <= 0 && left_area < 0) {
                            status = append_vertex(portal_left, left_node_ref, 0, path, left_node_ref != INVALID_NODE_REF ? left_node_type : NODE_TYPE_POLY,
                                                   max_points);
                        } else if (left_area >= 0 && right_area > 0) {
                            status = append_vertex(portal_right, right_node_ref, 0, path,
                                                   right_node_ref != INVALID_NODE_REF ? right_node_type : NODE_TYPE_POLY, max_points);
                        }

                        if (status && status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);
                    }

                    uint32_t start_append = append_vertex(off_mesh_start, to_ref, STRAIGHT_PATH_POINT_OFFMESH, path, NODE_TYPE_OFFMESH, max_points);
                    if (start_append != APPEND_VERTEX_IN_PROGRESS) return stopped(start_append, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);

                    uint32_t end_append = append_vertex(off_mesh_end, to_poly_ref, 0, path, NODE_TYPE_POLY, max_points);
                    if (end_append != APPEND_VERTEX_IN_PROGRESS) return stopped(end_append, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);

                    portal_apex = off_mesh_end;
                    portal_left = off_mesh_end;
                    portal_right = off_mesh_end;
                    apex_index = i + 2;
                    left_index = i + 2;
                    right_index = i + 2;
                    left_node_ref = to_poly_ref;
                    right_node_ref = to_poly_ref;
                    left_node_type = NODE_TYPE_POLY;
                    right_node_type = NODE_TYPE_POLY;

                    i++;
                    continue;
                }
            } else {
                left = closest_end_position;
                right = closest_end_position;
                to_type = NODE_TYPE_POLY;
            }

            if (geometry::tri_area_2d(portal_apex, portal_right, right) <= 0.0f) {
                if (vec3::equals(portal_apex, portal_right) || geometry::tri_area_2d(portal_apex, portal_left, right) > 0.0f) {
                    portal_right = right;
                    right_node_ref = i + 1 < path_size ? path_node_refs[i + 1] : INVALID_NODE_REF;
                    right_node_type = to_type;
                    right_index = i;
                } else {
                    if (straight_path_options & ANY_CROSSINGS) {
                        uint32_t status =
                            append_portals(nav_mesh, apex_index, left_index, portal_left, path_node_refs, path, straight_path_options, max_points);
                        if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, PARTIAL, path);
                    }

                    portal_apex = portal_left;
                    apex_index = left_index;

                    uint32_t point_flags = left_node_ref == INVALID_NODE_REF ? STRAIGHT_PATH_POINT_END : 0;
                    uint32_t status = append_vertex(portal_apex, left_node_ref, point_flags, path,
                                                    left_node_ref != INVALID_NODE_REF ? left_node_type : NODE_TYPE_POLY, max_points);
                    if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);

                    portal_left = portal_apex;
                    portal_right = portal_apex;
                    left_index = apex_index;
                    right_index = apex_index;

                    i = apex_index;
                    continue;
                }
            }

            if (geometry::tri_area_2d(portal_apex, portal_left, left) >= 0.0f) {
                if (vec3::equals(portal_apex, portal_left) || geometry::tri_area_2d(portal_apex, portal_right, left) < 0.0f) {
                    portal_left = left;
                    left_node_ref = i + 1 < path_size ? path_node_refs[i + 1] : INVALID_NODE_REF;
                    left_node_type = to_type;
                    left_index = i;
                } else {
                    if (straight_path_options & ANY_CROSSINGS) {
                        uint32_t status =
                            append_portals(nav_mesh, apex_index, right_index, portal_right, path_node_refs, path, straight_path_options, max_points);
                        if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, PARTIAL, path);
                    }

                    portal_apex = portal_right;
                    apex_index = right_index;

                    uint32_t point_flags = right_node_ref == INVALID_NODE_REF ? STRAIGHT_PATH_POINT_END : 0;
                    uint32_t status = append_vertex(portal_apex, right_node_ref, point_flags, path,
                                                    right_node_ref != INVALID_NODE_REF ? right_node_type : NODE_TYPE_POLY, max_points);
                    if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);

                    portal_left = portal_apex;
                    portal_right = portal_apex;
                    left_index = apex_index;
                    right_index = apex_index;

                    i = apex_index;
                    continue;
                }
            }
        }

        if (straight_path_options & ANY_CROSSINGS) {
            uint32_t status =
                append_portals(nav_mesh, apex_index, path_size - 1, closest_end_position, path_node_refs, path, straight_path_options, max_points);
            if (status != APPEND_VERTEX_IN_PROGRESS) return stopped(status, PARTIAL, path);
        }
    }

    uint32_t end_status = append_vertex(closest_end_position, path_node_refs[path_node_count - 1], STRAIGHT_PATH_POINT_END, path, NODE_TYPE_POLY, max_points);
    return stopped(end_status, FIND_STRAIGHT_PATH_RESULT_SUCCESS, path);
}

void find_straight_path(FindStraightPathResult &out, const NavMesh &nav_mesh, Vec3 start, Vec3 end, Span<const NodeRef> node_path,
                        Span<StraightPathPoint> points, uint32_t straight_path_options) {
    PointBuffer buffer = {points.data, points.capacity, 0};
    out = find_straight_path_in(nav_mesh, start, end, node_path.data, node_path.capacity, straight_path_options, buffer);
}

}

}
