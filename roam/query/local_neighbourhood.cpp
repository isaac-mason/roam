#include "roam/query/local_neighbourhood.h"

#include "roam/geometry/geometry.h"

namespace roam {

namespace local_neighbourhood {

struct SegmentInterval {
    NodeRef node_ref;
    int tmin;
    int tmax;
};

constexpr uint32_t INLINE_INTERVALS = 16;

struct SegmentIntervals {
    SegmentInterval inline_values[INLINE_INTERVALS];
    Array<SegmentInterval> overflow;
    SegmentInterval *data;
    uint32_t length;
    uint32_t capacity;
};

static void insert_interval(SegmentIntervals &intervals, int tmin, int tmax, NodeRef node_ref) {
    if (intervals.length == intervals.capacity) {
        uint32_t capacity = intervals.capacity * 2;
        if (intervals.data == intervals.inline_values) {
            intervals.overflow.resize(capacity);
            for (uint32_t i = 0; i < intervals.length; i++) intervals.overflow[i] = intervals.inline_values[i];
        } else {
            intervals.overflow.resize(capacity);
        }
        intervals.data = intervals.overflow.data;
        intervals.capacity = capacity;
    }
    uint32_t index = 0;
    while (index < intervals.length && tmax > intervals.data[index].tmin) index++;
    for (uint32_t i = intervals.length; i > index; i--) intervals.data[i] = intervals.data[i - 1];
    intervals.data[index] = {node_ref, tmin, tmax};
    intervals.length++;
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

void find_local_neighbourhood(FindLocalNeighbourhoodResult &result, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                              Vec3 position, float radius, const QueryFilter &filter, Span<NodeRef> polys) {
    result = {false, 0, 0, false};
    Array<NodeRef> &found = workspace.local_refs;
    found.clear();
    SearchNodePool &nodes = workspace.local_nodes;
    nav_mesh_search::clear(nodes);

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(position) || radius < 0 || !isfinite(radius)) return;

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

    found.push(start_node_ref);

    Array<float> &vertices_a = workspace.vertices;
    Array<float> &vertices_b = workspace.other_vertices;

    for (uint32_t head = 0; head < queue.length; head++) {
        NodeRef current_ref = nodes.nodes[queue[head]].node_ref;

        if (!nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, current_ref).success) continue;

        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref);

        for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            NodeRef neighbour_ref = nav_mesh.links[(uint32_t)link_index].to_node_ref;
            if (neighbour_ref == INVALID_NODE_REF) continue;

            int32_t existing = nav_mesh_search::get_search_node(nodes, neighbour_ref, 0);
            if (existing != NULL_SEARCH_NODE && nodes.nodes[(uint32_t)existing].flags & NODE_FLAG_CLOSED) continue;

            TileAndPoly neighbour = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, neighbour_ref);
            if (!neighbour.success) continue;

            if (node::get_node_ref_type(neighbour_ref) == NODE_TYPE_OFFMESH) continue;
            if (!nav_mesh_api::passes_filter(filter, neighbour_ref, nav_mesh)) continue;

            GetPortalPointsResult portal;
            nav_mesh_search::get_portal_points_unchecked(portal, nav_mesh, current_ref, neighbour_ref);
            if (!portal.success) continue;

            if (geometry::distance_pt_seg_sqr_2d(position, portal.left, portal.right).dist_sqr > radius_sqr) continue;

            SearchNode neighbour_node = {};
            neighbour_node.position = position;
            neighbour_node.parent_node_ref = current_ref;
            neighbour_node.parent_state = 0;
            neighbour_node.node_ref = neighbour_ref;
            neighbour_node.flags = NODE_FLAG_CLOSED;
            uint32_t neighbour_index = nav_mesh_search::add_search_node(nodes, neighbour_node);

            uint32_t vertex_count_a = gather_vertices(*neighbour.tile, *neighbour.poly, vertices_a);

            bool overlap = false;
            for (uint32_t j = 0; j < found.length; ++j) {
                NodeRef past_ref = found[j];

                if (links_to(nav_mesh, node, past_ref)) continue;

                TileAndPoly past = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, past_ref);
                if (!past.success) continue;

                uint32_t vertex_count_b = gather_vertices(*past.tile, *past.poly, vertices_b);

                if (geometry::overlap_poly_poly_2d(vertices_a.data, (int)vertex_count_a, vertices_b.data, (int)vertex_count_b)) {
                    overlap = true;
                    break;
                }
            }

            if (overlap) continue;

            found.push(neighbour_ref);
            queue.push(neighbour_index);
        }
    }

    result.success = true;
    result.poly_total = found.length;
    result.poly_count = found.length < polys.capacity ? found.length : polys.capacity;
    result.truncated = result.poly_count < result.poly_total;
    for (uint32_t i = 0; i < result.poly_count; i++) polys.data[i] = found[i];
}

struct SegmentWriter {
    Span<WallSegment> segments;
    Span<NodeRef> refs;
    uint32_t count;
};

static void push_segment(SegmentWriter &writer, Vec3 start, Vec3 end, NodeRef ref) {
    if (writer.count < writer.segments.capacity) writer.segments.data[writer.count] = {start, end};
    if (writer.count < writer.refs.capacity) writer.refs.data[writer.count] = ref;
    writer.count++;
}

void get_poly_wall_segments(PolyWallSegmentsResult &result, const NavMesh &nav_mesh, NodeRef node_ref, const QueryFilter &filter, bool include_portals,
                            Span<WallSegment> segments, Span<NodeRef> segment_refs) {
    result = {false, 0, 0, false};
    SegmentWriter writer = {segments, segment_refs, 0};

    TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, node_ref);
    if (!tile_and_poly.success) return;

    const NavMeshTile &tile = *tile_and_poly.tile;
    const NavMeshPoly &poly = *tile_and_poly.poly;

    SegmentIntervals intervals;
    intervals.overflow = {};
    intervals.data = intervals.inline_values;
    intervals.length = 0;
    intervals.capacity = INLINE_INTERVALS;

    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);

    for (uint32_t i = 0, j = poly.vertex_count - 1; i < poly.vertex_count; j = i++) {
        intervals.length = 0;

        int nei = tile.poly_neis[poly.vertex_base + j];
        Vec3 vj = vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + j] * 3);
        Vec3 vi = vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3);

        if (nei & POLY_NEIS_FLAG_EXT_LINK) {
            for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
                const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
                if ((uint32_t)link.edge != j) continue;
                if (link.to_node_ref == INVALID_NODE_REF) continue;
                if (!nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, link.to_node_ref).success) continue;
                if (nav_mesh_api::passes_filter(filter, link.to_node_ref, nav_mesh)) insert_interval(intervals, link.bmin, link.bmax, link.to_node_ref);
            }
        } else {
            NodeRef neighbour_ref = INVALID_NODE_REF;
            if (nei) {
                neighbour_ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, tile, nei - 1).ref;
                if (!nav_mesh_api::passes_filter(filter, neighbour_ref, nav_mesh)) neighbour_ref = INVALID_NODE_REF;
            }

            if (neighbour_ref != INVALID_NODE_REF && !include_portals) continue;

            push_segment(writer, vj, vi, neighbour_ref);
            continue;
        }

        insert_interval(intervals, -1, 0, INVALID_NODE_REF);
        insert_interval(intervals, 255, 256, INVALID_NODE_REF);

        for (uint32_t k = 1; k < intervals.length; ++k) {
            if (include_portals && intervals.data[k].node_ref != INVALID_NODE_REF) {
                float tmin = (float)intervals.data[k].tmin / 255.0f;
                float tmax = (float)intervals.data[k].tmax / 255.0f;
                push_segment(writer, vec3::lerp(vj, vi, tmin), vec3::lerp(vj, vi, tmax), intervals.data[k].node_ref);
            }

            int imin = intervals.data[k - 1].tmax;
            int imax = intervals.data[k].tmin;
            if (imin != imax) {
                float tmin = (float)imin / 255.0f;
                float tmax = (float)imax / 255.0f;
                push_segment(writer, vec3::lerp(vj, vi, tmin), vec3::lerp(vj, vi, tmax), INVALID_NODE_REF);
            }
        }
    }

    intervals.overflow.release();
    result.success = true;
    result.segment_total = writer.count;
    result.segment_count = writer.count < segments.capacity ? writer.count : segments.capacity;
    result.truncated = result.segment_count < result.segment_total;
}

}

}
