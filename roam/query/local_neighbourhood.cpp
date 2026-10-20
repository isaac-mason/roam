#include "roam/query/local_neighbourhood.h"

#include "roam/generate/common.h"
#include "roam/geometry/geometry.h"

namespace roam {

namespace local_neighbourhood {

struct SegmentInterval {
    NodeRef node_ref;
    int tmin;
    int tmax;
};

static void insert_interval(Array<SegmentInterval> &intervals, int tmin, int tmax, NodeRef node_ref) {
    uint32_t index = 0;
    while (index < intervals.length && tmax > intervals[index].tmin) index++;
    intervals.push({});
    for (uint32_t i = intervals.length - 1; i > index; i--) intervals[i] = intervals[i - 1];
    intervals[index] = {node_ref, tmin, tmax};
}

static uint32_t gather_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, Array<float> &out) {
    out.resize(poly.vertex_count * 3);
    for (uint32_t k = 0; k < poly.vertex_count; ++k) {
        vec3::store(out.data, k * 3, vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + k] * 3));
    }
    return poly.vertex_count;
}

static bool links_to(const NavMesh &nav_mesh, const NavMeshNode &node, NodeRef ref) {
    for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
        if (nav_mesh.links[(uint32_t)link].to_node_ref == ref) return true;
    }
    return false;
}

static FindLocalNeighbourhoodResult find_local_neighbourhood_in(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 position, float radius,
                                                                const QueryFilter &filter, SearchWorkspace &workspace) {
    FindLocalNeighbourhoodResult result = {};
    SearchNodePool &nodes = workspace.local_nodes;
    nav_mesh_search::clear(nodes);

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(position) || radius < 0 || !isfinite(radius)) return result;

    SearchNode start = {};
    start.position = position;
    start.parent_node_ref = INVALID_NODE_REF;
    start.parent_state = NO_PARENT_STATE;
    start.node_ref = start_node_ref;
    start.flags = NODE_FLAG_CLOSED;
    uint32_t start_index = nav_mesh_search::add_search_node(nodes, start);

    Array<uint32_t> &queue = workspace.local_queue;
    queue.clear();
    queue.push(start_index);

    float radius_sqr = radius * radius;

    result.node_refs.push(start_node_ref);

    Array<float> &vertices_a = workspace.vertices;
    Array<float> &vertices_b = workspace.other_vertices;

    for (uint32_t head = 0; head < queue.length; head++) {
        NodeRef current_ref = nodes.nodes[queue[head]].node_ref;

        if (!nav_mesh_api::get_tile_and_poly_by_ref(current_ref, nav_mesh).success) continue;

        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref);

        for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            NodeRef neighbour_ref = nav_mesh.links[(uint32_t)link_index].to_node_ref;
            if (neighbour_ref == INVALID_NODE_REF) continue;

            int32_t existing = nav_mesh_search::get_search_node(nodes, neighbour_ref, 0);
            if (existing != NULL_SEARCH_NODE && nodes.nodes[(uint32_t)existing].flags & NODE_FLAG_CLOSED) continue;

            TileAndPoly neighbour = nav_mesh_api::get_tile_and_poly_by_ref(neighbour_ref, nav_mesh);
            if (!neighbour.success) continue;

            if (node::get_node_ref_type(neighbour_ref) == NODE_TYPE_OFFMESH) continue;
            if (!filter.pass_filter(filter, neighbour_ref, nav_mesh)) continue;

            Vec3 va;
            Vec3 vb;
            if (!nav_mesh_search::get_portal_points(nav_mesh, current_ref, neighbour_ref, va, vb)) continue;

            if (geometry::distance_pt_seg_sqr_2d(position, va, vb).dist_sqr > radius_sqr) continue;

            SearchNode neighbour_node = {};
            neighbour_node.position = position;
            neighbour_node.parent_node_ref = current_ref;
            neighbour_node.parent_state = 0;
            neighbour_node.node_ref = neighbour_ref;
            neighbour_node.flags = NODE_FLAG_CLOSED;
            uint32_t neighbour_index = nav_mesh_search::add_search_node(nodes, neighbour_node);

            uint32_t vertex_count_a = gather_vertices(*neighbour.tile, *neighbour.poly, vertices_a);

            bool overlap = false;
            for (uint32_t j = 0; j < result.node_refs.length; ++j) {
                NodeRef past_ref = result.node_refs[j];

                if (links_to(nav_mesh, node, past_ref)) continue;

                TileAndPoly past = nav_mesh_api::get_tile_and_poly_by_ref(past_ref, nav_mesh);
                if (!past.success) continue;

                uint32_t vertex_count_b = gather_vertices(*past.tile, *past.poly, vertices_b);

                if (geometry::overlap_poly_poly_2d(vertices_a.data, (int)vertex_count_a, vertices_b.data, (int)vertex_count_b)) {
                    overlap = true;
                    break;
                }
            }

            if (overlap) continue;

            result.node_refs.push(neighbour_ref);
            queue.push(neighbour_index);
        }
    }

    result.success = true;
    return result;
}

FindLocalNeighbourhoodResult find_local_neighbourhood(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 position, float radius,
                                                      const QueryFilter &filter, SearchWorkspace *workspace) {
    if (workspace) return find_local_neighbourhood_in(nav_mesh, start_node_ref, position, radius, filter, *workspace);
    SearchWorkspace temporary = {};
    FindLocalNeighbourhoodResult result = find_local_neighbourhood_in(nav_mesh, start_node_ref, position, radius, filter, temporary);
    nav_mesh_search::release(temporary);
    return result;
}

void release(FindLocalNeighbourhoodResult &result) { result.node_refs.release(); }

static void push_segment(PolyWallSegmentsResult &result, Vec3 start, Vec3 end, NodeRef ref) {
    result.segment_vertices.push(start.x);
    result.segment_vertices.push(start.y);
    result.segment_vertices.push(start.z);
    result.segment_vertices.push(end.x);
    result.segment_vertices.push(end.y);
    result.segment_vertices.push(end.z);
    result.segment_refs.push(ref);
}

PolyWallSegmentsResult get_poly_wall_segments(const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals) {
    PolyWallSegmentsResult result = {};

    TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(node_ref, nav_mesh);
    if (!tile_and_poly.success) return result;

    const NavMeshTile &tile = *tile_and_poly.tile;
    const NavMeshPoly &poly = *tile_and_poly.poly;

    Array<SegmentInterval> intervals = {};

    for (uint32_t i = 0, j = poly.vertex_count - 1; i < poly.vertex_count; j = i++) {
        intervals.clear();

        int nei = tile.poly_neis[poly.vertex_base + j];
        Vec3 vj = vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + j] * 3);
        Vec3 vi = vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3);

        if (nei & POLY_NEIS_FLAG_EXT_LINK) {
            const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);
            for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
                const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
                if ((uint32_t)link.edge != j) continue;
                if (link.to_node_ref == INVALID_NODE_REF) continue;
                if (!nav_mesh_api::get_tile_and_poly_by_ref(link.to_node_ref, nav_mesh).success) continue;
                if (filter.pass_filter(filter, link.to_node_ref, nav_mesh)) insert_interval(intervals, link.bmin, link.bmax, link.to_node_ref);
            }
        } else {
            NodeRef neighbour_ref = INVALID_NODE_REF;
            if (nei) {
                neighbour_ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, tile, nei - 1).ref;
                if (nav_mesh_api::get_tile_and_poly_by_ref(neighbour_ref, nav_mesh).success && !filter.pass_filter(filter, neighbour_ref, nav_mesh)) {
                    neighbour_ref = INVALID_NODE_REF;
                }
            }

            if (neighbour_ref != INVALID_NODE_REF && !include_portals) continue;

            push_segment(result, vj, vi, neighbour_ref);
            continue;
        }

        insert_interval(intervals, -1, 0, INVALID_NODE_REF);
        insert_interval(intervals, 255, 256, INVALID_NODE_REF);

        for (uint32_t k = 1; k < intervals.length; ++k) {
            if (include_portals && intervals[k].node_ref != INVALID_NODE_REF) {
                float tmin = (float)intervals[k].tmin / 255.0f;
                float tmax = (float)intervals[k].tmax / 255.0f;
                push_segment(result, vec3::lerp(vj, vi, tmin), vec3::lerp(vj, vi, tmax), intervals[k].node_ref);
            }

            int imin = intervals[k - 1].tmax;
            int imax = intervals[k].tmin;
            if (imin != imax) {
                float tmin = (float)imin / 255.0f;
                float tmax = (float)imax / 255.0f;
                push_segment(result, vec3::lerp(vj, vi, tmin), vec3::lerp(vj, vi, tmax), INVALID_NODE_REF);
            }
        }
    }

    intervals.release();
    result.success = true;
    return result;
}

void release(PolyWallSegmentsResult &result) {
    result.segment_vertices.release();
    result.segment_refs.release();
}

}

}
