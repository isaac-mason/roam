#include "roam/query/nav_mesh_search.h"

#include <float.h>
#include <math.h>

#include "roam/geometry/geometry.h"

namespace roam {

namespace nav_mesh_search {

static const float HEURISTIC_SCALE = 0.999f;

static const uint32_t MIN_SEARCH_INDEX_TABLE = 256;

QueryWorkspace create_query_workspace() {
    QueryWorkspace workspace = {};
    workspace.node_path.reserve(INITIAL_PATH_CAPACITY);
    workspace.visited.reserve(INITIAL_PATH_CAPACITY);
    return workspace;
}

void release(QueryWorkspace &workspace) {
    release(workspace.nodes);
    release(workspace.open_list);
    release(workspace.local_nodes);
    workspace.local_queue.release();
    workspace.vertices.release();
    workspace.other_vertices.release();
    workspace.neighbours.release();
    workspace.node_path.release();
    workspace.visited.release();
    workspace.local_refs.release();
    workspace.cluster_stamps.release();
    workspace.cluster_queue.release();
}

uint32_t get_search_node_count(const QueryWorkspace &workspace) { return workspace.nodes.nodes.length; }

static int32_t first_with_ref(const SearchNodePool &pool, NodeRef node_ref) {
    uint32_t index = node::get_node_ref_index(node_ref);
    if (index >= pool.stamp_by_index.length || pool.stamp_by_index[index] != pool.stamp) return NULL_SEARCH_NODE;
    return pool.first_by_index[index];
}

int32_t get_search_node(const SearchNodePool &pool, NodeRef node_ref, int state) {
    for (int32_t index = first_with_ref(pool, node_ref); index != NULL_SEARCH_NODE; index = pool.nodes[(uint32_t)index].next_with_same_ref) {
        const SearchNode &node = pool.nodes[(uint32_t)index];
        if (node.node_ref == node_ref && node.state == state) return index;
    }
    return NULL_SEARCH_NODE;
}

int32_t get_first_search_node(const SearchNodePool &pool, NodeRef node_ref) {
    int32_t first = NULL_SEARCH_NODE;
    for (int32_t index = first_with_ref(pool, node_ref); index != NULL_SEARCH_NODE; index = pool.nodes[(uint32_t)index].next_with_same_ref) {
        if (pool.nodes[(uint32_t)index].node_ref == node_ref) first = index;
    }
    return first;
}

uint32_t add_search_node(SearchNodePool &pool, const SearchNode &node) {
    uint32_t ref_index = node::get_node_ref_index(node.node_ref);
    if (ref_index >= pool.stamp_by_index.length) {
        uint32_t length = pool.stamp_by_index.length > 0 ? pool.stamp_by_index.length : MIN_SEARCH_INDEX_TABLE;
        while (length <= ref_index) length *= 2;
        pool.first_by_index.resize(length, NULL_SEARCH_NODE);
        pool.stamp_by_index.resize(length, pool.stamp - 1);
    }
    if (pool.stamp_by_index[ref_index] != pool.stamp) {
        pool.stamp_by_index[ref_index] = pool.stamp;
        pool.first_by_index[ref_index] = NULL_SEARCH_NODE;
    }

    uint32_t index = pool.nodes.length;
    pool.nodes.push(node);
    pool.nodes[index].next_with_same_ref = pool.first_by_index[ref_index];
    pool.first_by_index[ref_index] = (int32_t)index;
    return index;
}

void clear(SearchNodePool &pool) {
    pool.nodes.clear();
    pool.stamp++;
    if (pool.stamp == 0) {
        for (uint32_t &stamp : pool.stamp_by_index) stamp = UINT32_MAX;
    }
}

void release(SearchNodePool &pool) {
    pool.nodes.release();
    pool.first_by_index.release();
    pool.stamp_by_index.release();
}

static SearchNode make_search_node(NodeRef node_ref, int state, uint32_t flags, Vec3 position) {
    SearchNode node = {};
    node.position = position;
    node.parent_node_ref = INVALID_NODE_REF;
    node.parent_state = NO_PARENT_STATE;
    node.state = state;
    node.flags = flags;
    node.node_ref = node_ref;
    node.next_with_same_ref = NULL_SEARCH_NODE;
    return node;
}

static float total_of(const SearchNodePool &pool, uint32_t node) { return pool.nodes[node].total; }

static void bubble_up_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t i, uint32_t node) {
    float node_total = total_of(pool, node);
    while (i > 0) {
        uint32_t parent = (i - 1) / 2;
        if (!(total_of(pool, queue.items[parent]) > node_total)) break;
        queue.items[i] = queue.items[parent];
        i = parent;
    }
    queue.items[i] = node;
}

static void trickle_down_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t i, uint32_t node) {
    uint32_t count = queue.items.length;
    uint32_t child = 2 * i + 1;
    float node_total = total_of(pool, node);

    while (child < count) {
        if (child + 1 < count && total_of(pool, queue.items[child + 1]) < total_of(pool, queue.items[child])) child++;
        if (node_total <= total_of(pool, queue.items[child])) break;
        queue.items[i] = queue.items[child];
        i = child;
        child = i * 2 + 1;
    }

    queue.items[i] = node;
}

void push_node_to_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node) {
    queue.items.push(node);
    bubble_up_queue(queue, pool, queue.items.length - 1, node);
}

int32_t pop_node_from_queue(SearchNodeQueue &queue, const SearchNodePool &pool) {
    if (queue.items.length == 0) return NULL_SEARCH_NODE;

    uint32_t node = queue.items[0];
    uint32_t last = queue.items.pop();

    if (queue.items.length > 0) {
        queue.items[0] = last;
        trickle_down_queue(queue, pool, 0, last);
    }

    return (int32_t)node;
}

void reindex_node_in_queue(SearchNodeQueue &queue, const SearchNodePool &pool, uint32_t node) {
    for (uint32_t i = 0; i < queue.items.length; i++) {
        if (queue.items[i] != node) continue;
        bubble_up_queue(queue, pool, i, node);
        return;
    }
}

void release(SearchNodeQueue &queue) { queue.items.release(); }

static const NavMeshLink *find_link_to(const NavMesh &nav_mesh, const NavMeshNode &from_node, NodeRef to_node_ref) {
    for (int32_t link = from_node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
        if (nav_mesh.links[(uint32_t)link].to_node_ref == to_node_ref) return &nav_mesh.links[(uint32_t)link];
    }
    return nullptr;
}

static const OffMeshConnection *off_mesh_connection_of(const NavMesh &nav_mesh, const NavMeshNode &node) {
    if (node.off_mesh_connection_id < 0 || (uint32_t)node.off_mesh_connection_id >= nav_mesh.off_mesh_connections.length) return nullptr;
    const OffMeshConnection &connection = nav_mesh.off_mesh_connections[(uint32_t)node.off_mesh_connection_id];
    return connection.allocated ? &connection : nullptr;
}

static Vec3 tile_vertex(const NavMeshTile &tile, const NavMeshPoly &poly, uint32_t i) {
    return vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3);
}

void get_portal_points_unchecked(GetPortalPointsResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    out = {};
    const NavMeshNode &from_node = nav_mesh_api::get_node_by_ref(nav_mesh, from_node_ref);

    const NavMeshLink *to_link = find_link_to(nav_mesh, from_node, to_node_ref);
    if (!to_link) return;

    NodeType from_type = node::get_node_ref_type(from_node_ref);
    NodeType to_type = node::get_node_ref_type(to_node_ref);

    if (from_type == NODE_TYPE_POLY && to_type == NODE_TYPE_POLY) {
        const NavMeshTile &from_tile = nav_mesh.tiles[(uint32_t)from_node.tile_id];
        const NavMeshPoly &from_poly = from_tile.polys[(uint32_t)from_node.poly_index];

        Vec3 v0 = tile_vertex(from_tile, from_poly, (uint32_t)to_link->edge);
        Vec3 v1 = tile_vertex(from_tile, from_poly, ((uint32_t)to_link->edge + 1) % from_poly.vertex_count);

        if (to_link->side != 0xff && (to_link->bmin != 0 || to_link->bmax != 255)) {
            const float s = 1.0f / 255.0f;
            out.left = vec3::lerp(v0, v1, (float)to_link->bmin * s);
            out.right = vec3::lerp(v0, v1, (float)to_link->bmax * s);
        } else {
            out.left = v0;
            out.right = v1;
        }

        out.success = true;
        return;
    }

    if (from_type == NODE_TYPE_POLY && to_type == NODE_TYPE_OFFMESH) {
        const OffMeshConnection *connection = off_mesh_connection_of(nav_mesh, nav_mesh_api::get_node_by_ref(nav_mesh, to_node_ref));
        if (!connection) return;
        Vec3 position = to_link->edge == 0 ? connection->start : connection->end;
        out.left = position;
        out.right = position;
        out.success = true;
        return;
    }

    if (from_type == NODE_TYPE_OFFMESH && to_type == NODE_TYPE_POLY) {
        const OffMeshConnection *connection = off_mesh_connection_of(nav_mesh, from_node);
        if (!connection) return;
        Vec3 position = to_link->edge == 0 ? connection->start : connection->end;
        out.left = position;
        out.right = position;
        out.success = true;
    }
}

static void edge_mid_point(GetEdgeMidPointResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    GetPortalPointsResult portal;
    get_portal_points_unchecked(portal, nav_mesh, from_node_ref, to_node_ref);
    out.success = portal.success;
    out.position = portal.success ? vec3::scale(vec3::add(portal.left, portal.right), 0.5f) : Vec3{0, 0, 0};
}

static bool valid_pair(const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    return nav_mesh_api::is_valid_node_ref(nav_mesh, from_node_ref) && nav_mesh_api::is_valid_node_ref(nav_mesh, to_node_ref);
}

void get_portal_points(GetPortalPointsResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    out = {};
    if (valid_pair(nav_mesh, from_node_ref, to_node_ref)) get_portal_points_unchecked(out, nav_mesh, from_node_ref, to_node_ref);
}

void get_edge_mid_point(GetEdgeMidPointResult &out, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    out = {};
    if (valid_pair(nav_mesh, from_node_ref, to_node_ref)) edge_mid_point(out, nav_mesh, from_node_ref, to_node_ref);
}

struct RefWriter {
    NodeRef *data;
    uint32_t capacity;
    uint32_t count;
    NodeRef last;
};

static void write_ref(RefWriter &writer, NodeRef ref) {
    if (writer.count < writer.capacity) writer.data[writer.count] = ref;
    writer.count++;
    writer.last = ref;
}

static void pop_ref(RefWriter &writer) {
    writer.count--;
    writer.last = writer.count > 0 && writer.count <= writer.capacity ? writer.data[writer.count - 1] : INVALID_NODE_REF;
}

static uint32_t written(uint32_t count, uint32_t capacity) { return count < capacity ? count : capacity; }

static void set_counts(uint32_t &count, uint32_t &total, bool &truncated, uint32_t all, uint32_t capacity) {
    total = all;
    count = written(all, capacity);
    truncated = all > capacity;
}

static int32_t path_parent(const SearchNodePool &pool, int32_t current) {
    const SearchNode &node = pool.nodes[(uint32_t)current];
    return node.parent_node_ref != INVALID_NODE_REF && node.parent_state != NO_PARENT_STATE
               ? get_search_node(pool, node.parent_node_ref, node.parent_state)
               : NULL_SEARCH_NODE;
}

static uint32_t collect_path(const SearchNodePool &pool, int32_t last_node, NodeRef *path, uint32_t max_path) {
    uint32_t count = 0;
    for (int32_t current = last_node; current != NULL_SEARCH_NODE; current = path_parent(pool, current)) count++;
    uint32_t index = count;
    for (int32_t current = last_node; current != NULL_SEARCH_NODE; current = path_parent(pool, current)) {
        index--;
        if (index < max_path) path[index] = pool.nodes[(uint32_t)current].node_ref;
    }
    return count;
}

static void raycast_base(RaycastResult &out, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                         const QueryFilter &filter, bool calculate_costs, NodeRef previous_ref, RefWriter *path);

static int link_state(const NavMeshLink &link) { return link.side != 0xff ? link.side >> 1 : 0; }

static uint32_t get_or_add_neighbour(const NavMesh &nav_mesh, SearchNodePool &pool, NodeRef best_node_ref, NodeRef neighbour_node_ref, int state) {
    int32_t existing = get_search_node(pool, neighbour_node_ref, state);
    if (existing != NULL_SEARCH_NODE) return (uint32_t)existing;

    SearchNode node = make_search_node(neighbour_node_ref, state, 0, {0, 0, 0});
    GetEdgeMidPointResult mid;
    edge_mid_point(mid, nav_mesh, best_node_ref, neighbour_node_ref);
    node.position = mid.position;
    return add_search_node(pool, node);
}

static int32_t search_node_path(FindNodePathResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                                NodeRef end_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter, FindNodePathOptions options) {
    out.success = false;
    out.flags = FIND_NODE_PATH_RESULT_NONE;
    out.path_count = 0;
    out.path_total = 0;
    out.truncated = false;
    float raycast_distance = options.raycast_distance;
    SearchNodePool &nodes = workspace.nodes;
    SearchNodeQueue &open_list = workspace.open_list;
    clear(nodes);
    open_list.items.clear();

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !nav_mesh_api::is_valid_node_ref(nav_mesh, end_node_ref) ||
        !vec3::finite(start_position) || !vec3::finite(end_position)) {
        out.flags = FIND_NODE_PATH_RESULT_NONE | FIND_NODE_PATH_RESULT_INVALID_INPUT;
        return NULL_SEARCH_NODE;
    }

    if (start_node_ref == end_node_ref) {
        out.flags = FIND_NODE_PATH_RESULT_SUCCESS | FIND_NODE_PATH_RESULT_COMPLETE_PATH;
        out.success = true;
        return (int32_t)add_search_node(nodes, make_search_node(start_node_ref, 0, NODE_FLAG_CLOSED, start_position));
    }

    SearchNode start = make_search_node(start_node_ref, 0, NODE_FLAG_OPEN, start_position);
    start.total = vec3::distance(start_position, end_position) * HEURISTIC_SCALE;
    uint32_t start_index = add_search_node(nodes, start);
    push_node_to_queue(open_list, nodes, start_index);

    int32_t last_best_node = (int32_t)start_index;
    float last_best_node_cost = start.total;

    while (open_list.items.length > 0) {
        uint32_t best_index = (uint32_t)pop_node_from_queue(open_list, nodes);
        nodes.nodes[best_index].flags &= ~NODE_FLAG_OPEN;
        nodes.nodes[best_index].flags |= NODE_FLAG_CLOSED;

        SearchNode best = nodes.nodes[best_index];
        NodeRef best_node_ref = best.node_ref;
        bool best_node_is_poly = node::get_node_ref_type(best_node_ref) == NODE_TYPE_POLY;
        if (best_node_ref == end_node_ref) {
            last_best_node = (int32_t)best_index;
            break;
        }

        const NavMeshNode &best_nav_node = nav_mesh_api::get_node_by_ref(nav_mesh, best_node_ref);
        NodeRef parent_node_ref = best.parent_node_ref;

        for (int32_t link_index = best_nav_node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
            NodeRef neighbour_node_ref = link.to_node_ref;

            if (neighbour_node_ref == parent_node_ref) continue;
            if (!nav_mesh_api::passes_filter(filter, neighbour_node_ref, nav_mesh)) continue;

            int state = link_state(link);
            uint32_t neighbour_index = get_or_add_neighbour(nav_mesh, nodes, best_node_ref, neighbour_node_ref, state);
            Vec3 neighbour_position = nodes.nodes[neighbour_index].position;

            float cost = 0;
            float heuristic = 0;

            bool found_shortcut = false;
            if (raycast_distance > 0 && best_node_is_poly && parent_node_ref != INVALID_NODE_REF &&
                node::get_node_ref_type(parent_node_ref) == NODE_TYPE_POLY && best.parent_state != NO_PARENT_STATE) {
                int32_t grandparent_index = get_search_node(nodes, parent_node_ref, best.parent_state);
                if (grandparent_index != NULL_SEARCH_NODE) {
                    SearchNode grandparent = nodes.nodes[(uint32_t)grandparent_index];
                    float ray_length = vec3::distance(grandparent.position, neighbour_position);
                    if (ray_length < raycast_distance) {
                        RaycastResult ray;
                        raycast_base(ray, nav_mesh, grandparent.node_ref, grandparent.position, neighbour_position, filter, true,
                                     grandparent.parent_node_ref, nullptr);
                        if (ray.t >= 1.0f) {
                            found_shortcut = true;
                            cost = grandparent.cost + ray.path_cost;
                        }
                    }
                }
            }

            if (!found_shortcut) {
                float current_cost =
                    nav_mesh_api::filter_cost(filter, best.position, neighbour_position, nav_mesh, parent_node_ref, best_node_ref, neighbour_node_ref);
                cost = best.cost + current_cost;
            }

            if (neighbour_node_ref == end_node_ref) {
                float end_cost = nav_mesh_api::filter_cost(filter, neighbour_position, end_position, nav_mesh, best_node_ref, neighbour_node_ref, INVALID_NODE_REF);
                cost = cost + end_cost;
                heuristic = 0;
            } else {
                heuristic = vec3::distance(neighbour_position, end_position) * HEURISTIC_SCALE;
            }

            float total = cost + heuristic;

            SearchNode &neighbour = nodes.nodes[neighbour_index];
            if (neighbour.flags & NODE_FLAG_OPEN && total >= neighbour.total) continue;
            if (neighbour.flags & NODE_FLAG_CLOSED && total >= neighbour.total) continue;

            if (found_shortcut) {
                neighbour.parent_node_ref = best.parent_node_ref;
                neighbour.parent_state = best.parent_state;
            } else {
                neighbour.parent_node_ref = best.node_ref;
                neighbour.parent_state = best.state;
            }
            neighbour.node_ref = neighbour_node_ref;
            neighbour.flags = neighbour.flags & ~NODE_FLAG_CLOSED;
            neighbour.cost = cost;
            neighbour.total = total;

            if (neighbour.flags & NODE_FLAG_OPEN) {
                reindex_node_in_queue(open_list, nodes, neighbour_index);
            } else {
                neighbour.flags |= NODE_FLAG_OPEN;
                push_node_to_queue(open_list, nodes, neighbour_index);
            }

            if (heuristic < last_best_node_cost) {
                last_best_node = (int32_t)neighbour_index;
                last_best_node_cost = heuristic;
            }
        }
    }

    out.success = true;

    if (nodes.nodes[(uint32_t)last_best_node].node_ref != end_node_ref) {
        out.flags = FIND_NODE_PATH_RESULT_PARTIAL_PATH;
        return last_best_node;
    }

    out.flags = FIND_NODE_PATH_RESULT_SUCCESS | FIND_NODE_PATH_RESULT_COMPLETE_PATH;
    return last_best_node;
}

void find_node_path(FindNodePathResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref,
                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, Span<NodeRef> path, FindNodePathOptions options) {
    workspace.last_path_node = search_node_path(out, workspace, nav_mesh, start_node_ref, end_node_ref, start_position, end_position, filter, options);
    copy_last_node_path(out, workspace, path);
}

void copy_last_node_path(FindNodePathResult &out, const QueryWorkspace &workspace, Span<NodeRef> path) {
    int32_t last_node = workspace.last_path_node;
    uint32_t total = last_node == NULL_SEARCH_NODE ? 0 : collect_path(workspace.nodes, last_node, path.data, path.capacity);
    set_counts(out.path_count, out.path_total, out.truncated, total, path.capacity);
}

SlicedNodePathQuery create_sliced_node_path_query() {
    SlicedNodePathQuery query = {};
    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
    query.start_node_ref = INVALID_NODE_REF;
    query.end_node_ref = INVALID_NODE_REF;
    query.filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    query.last_best_node = NULL_SEARCH_NODE;
    query.last_best_node_cost = INFINITY;
    return query;
}

void init_sliced_find_node_path(SlicedNodePathQuery &query, const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref,
                                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, uint32_t flags) {
    query.start_node_ref = start_node_ref;
    query.end_node_ref = end_node_ref;
    query.start_position = start_position;
    query.end_position = end_position;
    query.filter = filter;

    query.status = SLICED_FIND_NODE_PATH_FAILURE;
    query.finalized_status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
    clear(query.nodes);
    query.open_list.items.clear();
    query.last_best_node = NULL_SEARCH_NODE;
    query.last_best_node_cost = INFINITY;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !nav_mesh_api::is_valid_node_ref(nav_mesh, end_node_ref) ||
        !vec3::finite(start_position) || !vec3::finite(end_position)) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE | SLICED_FIND_NODE_PATH_INVALID_PARAM;
        return;
    }

    if (flags & SLICED_FIND_NODE_PATH_ANY_ANGLE) {
        query.has_raycast_limit = true;
        query.raycast_limit_sqr = 25.0f;
    } else {
        query.has_raycast_limit = false;
        query.raycast_limit_sqr = 0;
    }

    SearchNode start = make_search_node(start_node_ref, 0, NODE_FLAG_OPEN, start_position);
    start.total = vec3::distance(start_position, end_position) * HEURISTIC_SCALE;
    uint32_t start_index = add_search_node(query.nodes, start);
    query.last_best_node = (int32_t)start_index;
    query.last_best_node_cost = start.total;

    if (start_node_ref == end_node_ref) {
        query.status = SLICED_FIND_NODE_PATH_SUCCESS;
        return;
    }

    push_node_to_queue(query.open_list, query.nodes, start_index);
    query.status = SLICED_FIND_NODE_PATH_IN_PROGRESS;
}

int update_sliced_find_node_path(SlicedNodePathQuery &query, const NavMesh &nav_mesh, int max_iterations) {
    int iterations_done = 0;

    if (!(query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS)) return iterations_done;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, query.start_node_ref) || !nav_mesh_api::is_valid_node_ref(nav_mesh, query.end_node_ref)) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE;
        return iterations_done;
    }

    const QueryFilter &filter = query.filter;
    SearchNodePool &nodes = query.nodes;

    while (iterations_done < max_iterations && query.open_list.items.length > 0) {
        iterations_done++;

        uint32_t best_index = (uint32_t)pop_node_from_queue(query.open_list, nodes);
        nodes.nodes[best_index].flags &= ~NODE_FLAG_OPEN;
        nodes.nodes[best_index].flags |= NODE_FLAG_CLOSED;

        SearchNode best = nodes.nodes[best_index];

        if (best.node_ref == query.end_node_ref) {
            query.last_best_node = (int32_t)best_index;
            query.status = SLICED_FIND_NODE_PATH_SUCCESS;
            return iterations_done;
        }

        NodeRef best_node_ref = best.node_ref;
        const NavMeshNode &best_nav_node = nav_mesh_api::get_node_by_ref(nav_mesh, best_node_ref);
        NodeRef parent_node_ref = best.parent_node_ref;

        for (int32_t link_index = best_nav_node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
            NodeRef neighbour_node_ref = link.to_node_ref;

            if (neighbour_node_ref == parent_node_ref) continue;
            if (!nav_mesh_api::passes_filter(filter, neighbour_node_ref, nav_mesh)) continue;

            int state = link_state(link);
            uint32_t neighbour_index = get_or_add_neighbour(nav_mesh, nodes, best_node_ref, neighbour_node_ref, state);
            Vec3 neighbour_position = nodes.nodes[neighbour_index].position;

            float cost = 0;
            float heuristic = 0;

            bool found_shortcut = false;
            if (query.has_raycast_limit && query.raycast_limit_sqr != 0 && best.parent_node_ref != INVALID_NODE_REF &&
                best.parent_state != NO_PARENT_STATE) {
                int32_t grandparent_index = get_search_node(nodes, best.parent_node_ref, best.parent_state);
                if (grandparent_index != NULL_SEARCH_NODE) {
                    SearchNode grandparent = nodes.nodes[(uint32_t)grandparent_index];
                    float ray_length = vec3::distance(grandparent.position, neighbour_position);
                    if (ray_length < sqrtf(query.raycast_limit_sqr)) {
                        RaycastResult ray;
                        raycast_base(ray, nav_mesh, grandparent.node_ref, grandparent.position, neighbour_position, filter, true,
                                     grandparent.parent_node_ref, nullptr);
                        if (ray.t >= 1.0f) {
                            found_shortcut = true;
                            cost = grandparent.cost + ray.path_cost;
                        }
                    }
                }
            }

            if (!found_shortcut) {
                float current_cost =
                    nav_mesh_api::filter_cost(filter, best.position, neighbour_position, nav_mesh, parent_node_ref, best_node_ref, neighbour_node_ref);
                cost = best.cost + current_cost;
            }

            if (neighbour_node_ref == query.end_node_ref) {
                float end_cost =
                    nav_mesh_api::filter_cost(filter, neighbour_position, query.end_position, nav_mesh, best_node_ref, neighbour_node_ref, INVALID_NODE_REF);
                cost = cost + end_cost;
                heuristic = 0;
            } else {
                heuristic = vec3::distance(neighbour_position, query.end_position) * HEURISTIC_SCALE;
            }

            float total = cost + heuristic;

            SearchNode &neighbour = nodes.nodes[neighbour_index];
            if ((neighbour.flags & NODE_FLAG_OPEN && total >= neighbour.total) || (neighbour.flags & NODE_FLAG_CLOSED && total >= neighbour.total)) {
                continue;
            }

            if (found_shortcut) {
                neighbour.parent_node_ref = best.parent_node_ref;
                neighbour.parent_state = best.parent_state;
            } else {
                neighbour.parent_node_ref = best.node_ref;
                neighbour.parent_state = best.state;
            }
            neighbour.cost = cost;
            neighbour.total = total;
            neighbour.flags &= ~NODE_FLAG_CLOSED;

            if (found_shortcut) {
                neighbour.flags |= NODE_FLAG_PARENT_DETACHED;
            } else {
                neighbour.flags &= ~NODE_FLAG_PARENT_DETACHED;
            }

            if (neighbour.flags & NODE_FLAG_OPEN) {
                reindex_node_in_queue(query.open_list, nodes, neighbour_index);
            } else {
                neighbour.flags |= NODE_FLAG_OPEN;
                push_node_to_queue(query.open_list, nodes, neighbour_index);
            }

            if (heuristic < query.last_best_node_cost) {
                query.last_best_node_cost = heuristic;
                query.last_best_node = (int32_t)neighbour_index;
            }
        }
    }

    if (query.open_list.items.length == 0) query.status = SLICED_FIND_NODE_PATH_SUCCESS | SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    return iterations_done;
}

static void build_sliced_path(RefWriter &path, const NavMesh &nav_mesh, SlicedNodePathQuery &query, int32_t last_node) {
    Array<uint32_t> &forward = query.path_nodes;
    forward.clear();
    for (int32_t current = last_node; current != NULL_SEARCH_NODE; current = path_parent(query.nodes, current)) forward.push((uint32_t)current);
    for (uint32_t i = 0, j = forward.length; i + 1 < j; i++, j--) {
        uint32_t swap = forward[i];
        forward[i] = forward[j - 1];
        forward[j - 1] = swap;
    }

    for (uint32_t i = 0; i < forward.length; i++) {
        const SearchNode &current = query.nodes.nodes[forward[i]];
        bool was_detached = (current.flags & NODE_FLAG_PARENT_DETACHED) != 0;
        bool has_next = i + 1 < forward.length;

        if (query.has_raycast_limit && has_next && was_detached) {
            const SearchNode &next = query.nodes.nodes[forward[i + 1]];
            RaycastResult ray;
            raycast_base(ray, nav_mesh, current.node_ref, current.position, next.position, query.filter, false, INVALID_NODE_REF, &path);
            if (path.count > 0 && path.last == next.node_ref) pop_ref(path);
        } else {
            write_ref(path, current.node_ref);
        }
    }
}

static void finish_sliced_path(SlicedFindNodePathResult &out, const NavMesh &nav_mesh, SlicedNodePathQuery &query, int32_t last_node,
                               Span<NodeRef> path) {
    RefWriter writer = {path.data, path.capacity, 0, INVALID_NODE_REF};
    build_sliced_path(writer, nav_mesh, query, last_node);
    set_counts(out.path_count, out.path_total, out.truncated, writer.count, path.capacity);
}

static void single_node_path(SlicedFindNodePathResult &out, SlicedNodePathQuery &query, Span<NodeRef> path) {
    if (path.capacity > 0) path.data[0] = query.start_node_ref;
    set_counts(out.path_count, out.path_total, out.truncated, 1, path.capacity);
    out.status = SLICED_FIND_NODE_PATH_SUCCESS;
    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
}

void finalize_sliced_find_node_path(SlicedFindNodePathResult &out, const NavMesh &nav_mesh, SlicedNodePathQuery &query, Span<NodeRef> path) {
    out = {SLICED_FIND_NODE_PATH_FAILURE, 0, 0, false};

    if (query.last_best_node == NULL_SEARCH_NODE) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE;
        out.status = query.status;
        return;
    }

    if (query.start_node_ref == query.end_node_ref) {
        single_node_path(out, query, path);
        return;
    }

    query.status |= query.finalized_status;
    if (query.nodes.nodes[(uint32_t)query.last_best_node].node_ref != query.end_node_ref) query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    finish_sliced_path(out, nav_mesh, query, query.last_best_node, path);
    out.status = SLICED_FIND_NODE_PATH_SUCCESS | (query.status & SLICED_FIND_NODE_PATH_PARTIAL_RESULT);

    query.finalized_status = query.status & SLICED_FIND_NODE_PATH_PARTIAL_RESULT;
    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
}

static int32_t furthest_existing_node(const SlicedNodePathQuery &query, Span<const NodeRef> existing_path) {
    for (uint32_t i = existing_path.capacity; i-- > 0;) {
        int32_t first = get_first_search_node(query.nodes, existing_path[i]);
        if (first != NULL_SEARCH_NODE) return first;
    }
    return NULL_SEARCH_NODE;
}

void finalize_sliced_find_node_path_partial(SlicedFindNodePathResult &out, const NavMesh &nav_mesh, SlicedNodePathQuery &query,
                                            Span<const NodeRef> existing_path, Span<NodeRef> path) {
    out = {SLICED_FIND_NODE_PATH_FAILURE, 0, 0, false};

    int32_t furthest_node = furthest_existing_node(query, existing_path);

    if (furthest_node == NULL_SEARCH_NODE) {
        furthest_node = query.last_best_node;
        query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;
    }

    if (furthest_node == NULL_SEARCH_NODE) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE;
        out.status = query.status;
        return;
    }

    if (query.start_node_ref == query.end_node_ref) {
        single_node_path(out, query, path);
        return;
    }

    query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    finish_sliced_path(out, nav_mesh, query, furthest_node, path);
    out.status = SLICED_FIND_NODE_PATH_SUCCESS | SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
}

void release(SlicedNodePathQuery &query) {
    release(query.nodes);
    release(query.open_list);
    query.path_nodes.release();
}


static uint32_t gather_poly_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, Array<float> &out_vertices) {
    out_vertices.resize(poly.vertex_count * 3);
    for (uint32_t i = 0; i < poly.vertex_count; ++i) vec3::store(out_vertices.data, i * 3, tile_vertex(tile, poly, i));
    return poly.vertex_count;
}

static int32_t surface_parent(const SearchNodePool &pool, int32_t current) {
    const SearchNode &node = pool.nodes[(uint32_t)current];
    return node.parent_node_ref != INVALID_NODE_REF ? get_search_node(pool, node.parent_node_ref, 0) : NULL_SEARCH_NODE;
}

void move_along_surface(MoveAlongSurfaceResult &result, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                        Vec3 start_position, Vec3 end_position, const QueryFilter &filter, Span<NodeRef> visited) {
    result.visited_count = 0;
    result.visited_total = 0;
    result.truncated = false;
    result.success = false;
    result.position = start_position;
    result.node_ref = start_node_ref;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(start_position) || !vec3::finite(end_position)) return;

    result.success = true;

    SearchNodePool &nodes = workspace.local_nodes;
    clear(nodes);
    uint32_t start_index = add_search_node(nodes, make_search_node(start_node_ref, 0, NODE_FLAG_CLOSED, start_position));

    Vec3 best_position = start_position;
    float best_distance = INFINITY;
    int32_t best_node = (int32_t)start_index;

    Vec3 search_position = vec3::lerp(start_position, end_position, 0.5f);
    float search_radius = vec3::distance(start_position, end_position) / 2.0f + 0.001f;
    float search_radius_sqr = search_radius * search_radius;

    Array<uint32_t> &queue = workspace.local_queue;
    queue.clear();
    queue.push(start_index);
    Array<float> &vertices = workspace.vertices;
    Array<NodeRef> &neighbours = workspace.neighbours;

    for (uint32_t head = 0; head < queue.length; head++) {
        uint32_t current_index = queue[head];
        NodeRef current_ref = nodes.nodes[current_index].node_ref;

        TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, current_ref);
        if (!tile_and_poly.success) continue;

        const NavMeshTile &tile = *tile_and_poly.tile;
        const NavMeshPoly &poly = *tile_and_poly.poly;

        uint32_t vertex_count = gather_poly_vertices(tile, poly, vertices);

        float min_z = INFINITY;
        float max_z = -INFINITY;
        for (uint32_t k = 0; k < vertex_count; k++) {
            min_z = fminf(min_z, vertices[k * 3 + 2]);
            max_z = fmaxf(max_z, vertices[k * 3 + 2]);
        }
        bool within_z = end_position.z >= min_z && end_position.z <= max_z;

        if (within_z && geometry::point_in_poly(end_position, vertices.data, (int)vertex_count)) {
            best_node = (int32_t)current_index;
            best_position = end_position;
            break;
        }

        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref);

        for (uint32_t i = 0, j = vertex_count - 1; i < vertex_count; j = i++) {
            neighbours.clear();

            for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
                const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];
                NodeRef neighbour_ref = link.to_node_ref;
                if (node::get_node_ref_type(neighbour_ref) == NODE_TYPE_OFFMESH) continue;
                if ((uint32_t)link.edge != j) continue;
                if (!nav_mesh_api::passes_filter(filter, neighbour_ref, nav_mesh)) continue;
                neighbours.push(neighbour_ref);
            }

            Vec3 vj = vec3::load(vertices.data, j * 3);
            Vec3 vi = vec3::load(vertices.data, i * 3);

            if (neighbours.length == 0) {
                geometry::DistancePtSegSqr2dResult distance = geometry::distance_pt_seg_sqr_2d(end_position, vj, vi);
                if (distance.dist_sqr < best_distance) {
                    best_position = vec3::lerp(vj, vi, distance.t);
                    best_distance = distance.dist_sqr;
                    best_node = (int32_t)current_index;
                }
                continue;
            }

            for (NodeRef neighbour_ref : neighbours) {
                int32_t neighbour_index = get_search_node(nodes, neighbour_ref, 0);
                if (neighbour_index == NULL_SEARCH_NODE) {
                    neighbour_index = (int32_t)add_search_node(nodes, make_search_node(neighbour_ref, 0, 0, end_position));
                }

                SearchNode &neighbour = nodes.nodes[(uint32_t)neighbour_index];
                if (neighbour.flags & NODE_FLAG_CLOSED) continue;

                float distance_sqr = geometry::distance_pt_seg_sqr_2d(search_position, vj, vi).dist_sqr;
                if (distance_sqr > search_radius_sqr) continue;

                neighbour.parent_node_ref = nodes.nodes[current_index].node_ref;
                neighbour.parent_state = nodes.nodes[current_index].state;
                neighbour.flags |= NODE_FLAG_CLOSED;
                queue.push((uint32_t)neighbour_index);
            }
        }
    }

    uint32_t visited_count = 0;
    for (int32_t current = best_node; current != NULL_SEARCH_NODE; current = surface_parent(nodes, current)) visited_count++;
    uint32_t index = visited_count;
    for (int32_t current = best_node; current != NULL_SEARCH_NODE; current = surface_parent(nodes, current)) {
        index--;
        if (index < visited.capacity) visited.data[index] = nodes.nodes[(uint32_t)current].node_ref;
    }
    set_counts(result.visited_count, result.visited_total, result.truncated, visited_count, visited.capacity);

    result.position = best_position;
    result.node_ref = nodes.nodes[(uint32_t)best_node].node_ref;

    TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, result.node_ref);
    if (tile_and_poly.success) {
        GetPolyHeightResult height;
        nav_mesh_api::get_poly_height(height, *tile_and_poly.tile, *tile_and_poly.poly, tile_and_poly.poly_index, result.position);
        if (height.success) result.position.y = height.height;
    }
}


static void raycast_base(RaycastResult &result, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                         const QueryFilter &filter, bool calculate_costs, NodeRef previous_ref, RefWriter *path) {
    uint32_t path_start = path ? path->count : 0;
    result.path_count = 0;
    result.path_total = 0;
    result.truncated = false;
    result.t = 0;
    result.hit_normal = {0, 0, 0};
    result.hit_edge_index = -1;
    result.path_cost = 0;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(start_position) || !vec3::finite(end_position)) return;

    PolyVertices vertices;

    NodeRef current_ref = start_node_ref;
    NodeRef previous_ref_tracking = previous_ref;

    Vec3 direction = {0, 0, 0};
    Vec3 current_position = {0, 0, 0};
    if (calculate_costs) {
        direction = vec3::subtract(end_position, start_position);
        current_position = start_position;
    }

    while (current_ref != INVALID_NODE_REF) {
        TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, current_ref);
        if (!tile_and_poly.success) break;
        const NavMeshTile &tile = *tile_and_poly.tile;
        const NavMeshPoly &poly = *tile_and_poly.poly;

        uint32_t vertex_count = nav_mesh_api::gather_poly_vertices(tile, poly, vertices);

        geometry::IntersectSegmentPoly2dResult intersection =
            geometry::intersect_segment_poly_2d(start_position, end_position, (int)vertex_count, vertices.data);
        if (!intersection.intersects) break;

        result.hit_edge_index = intersection.seg_max;

        if (intersection.tmax > result.t) result.t = intersection.tmax;

        if (path) write_ref(*path, current_ref);

        if (intersection.seg_max == -1) {
            result.t = FLT_MAX;
            if (calculate_costs) {
                result.path_cost +=
                    nav_mesh_api::filter_cost(filter, current_position, end_position, nav_mesh, previous_ref_tracking, current_ref, INVALID_NODE_REF);
            }
            break;
        }

        NodeRef next_ref = INVALID_NODE_REF;

        const NavMeshNode &current_node = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref);

        for (int32_t link_index = current_node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];

            if (link.edge != intersection.seg_max) continue;
            if (node::get_node_ref_type(link.to_node_ref) == NODE_TYPE_OFFMESH) continue;
            if (!nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, link.to_node_ref).success) continue;
            if (!nav_mesh_api::passes_filter(filter, link.to_node_ref, nav_mesh)) continue;

            if (link.side == 0xff) {
                next_ref = link.to_node_ref;
                break;
            }

            if (link.bmin == 0 && link.bmax == 255) {
                next_ref = link.to_node_ref;
                break;
            }

            Vec3 left = tile_vertex(tile, poly, (uint32_t)link.edge);
            Vec3 right = tile_vertex(tile, poly, ((uint32_t)link.edge + 1) % poly.vertex_count);
            const float s = 1.0f / 255.0f;

            if (link.side == 0 || link.side == 4) {
                float lmin = left.z + (right.z - left.z) * ((float)link.bmin * s);
                float lmax = left.z + (right.z - left.z) * ((float)link.bmax * s);
                if (lmin > lmax) {
                    float swap = lmin;
                    lmin = lmax;
                    lmax = swap;
                }
                float z = start_position.z + (end_position.z - start_position.z) * intersection.tmax;
                if (z >= lmin && z <= lmax) {
                    next_ref = link.to_node_ref;
                    break;
                }
            } else if (link.side == 2 || link.side == 6) {
                float lmin = left.x + (right.x - left.x) * ((float)link.bmin * s);
                float lmax = left.x + (right.x - left.x) * ((float)link.bmax * s);
                if (lmin > lmax) {
                    float swap = lmin;
                    lmin = lmax;
                    lmax = swap;
                }
                float x = start_position.x + (end_position.x - start_position.x) * intersection.tmax;
                if (x >= lmin && x <= lmax) {
                    next_ref = link.to_node_ref;
                    break;
                }
            }
        }

        if (next_ref == INVALID_NODE_REF) {
            if (intersection.seg_max >= 0) {
                uint32_t a = (uint32_t)intersection.seg_max;
                uint32_t b = a + 1 < poly.vertex_count ? a + 1 : 0;
                Vec3 va = vec3::load(vertices.data, a * 3);
                Vec3 vb = vec3::load(vertices.data, b * 3);
                result.hit_normal = vec3::normalize({vb.z - va.z, 0, -(vb.x - va.x)});
            }

            if (calculate_costs) {
                Vec3 last_position = current_position;
                current_position = vec3::scale_and_add(start_position, direction, result.t);
                result.path_cost +=
                    nav_mesh_api::filter_cost(filter, last_position, current_position, nav_mesh, previous_ref_tracking, current_ref, INVALID_NODE_REF);
            }

            break;
        }

        if (calculate_costs) {
            Vec3 last_position = current_position;
            current_position = vec3::scale_and_add(start_position, direction, result.t);

            Vec3 e0 = tile_vertex(tile, poly, (uint32_t)intersection.seg_max);
            Vec3 e1 = tile_vertex(tile, poly, ((uint32_t)intersection.seg_max + 1) % poly.vertex_count);
            Vec3 edge_direction = vec3::subtract(e1, e0);
            Vec3 difference = vec3::subtract(current_position, e0);

            float s = edge_direction.x * edge_direction.x > edge_direction.z * edge_direction.z ? difference.x / edge_direction.x
                                                                                                 : difference.z / edge_direction.z;

            current_position.y = e0.y + edge_direction.y * s;

            result.path_cost += nav_mesh_api::filter_cost(filter, last_position, current_position, nav_mesh, previous_ref_tracking, current_ref, next_ref);
        }

        previous_ref_tracking = current_ref;
        current_ref = next_ref;
    }

    nav_mesh_api::release(vertices);
    if (path) {
        result.path_total = path->count - path_start;
        result.path_count = written(path->count, path->capacity) - written(path_start, path->capacity);
        result.truncated = path->count > path->capacity;
    }
}

void raycast(RaycastResult &out, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
             Span<NodeRef> path) {
    RefWriter writer = {path.data, path.capacity, 0, INVALID_NODE_REF};
    raycast_base(out, nav_mesh, start_node_ref, start_position, end_position, filter, false, INVALID_NODE_REF, &writer);
}

void raycast_with_costs(RaycastResult &out, const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                        const QueryFilter &filter, NodeRef previous_ref, Span<NodeRef> path) {
    RefWriter writer = {path.data, path.capacity, 0, INVALID_NODE_REF};
    raycast_base(out, nav_mesh, start_node_ref, start_position, end_position, filter, true, previous_ref, &writer);
}


static float poly_area(const NavMeshTile &tile, const NavMeshPoly &poly) {
    float area = 0;
    for (uint32_t j = 2; j < poly.vertex_count; j++) {
        area += geometry::tri_area_2d(tile_vertex(tile, poly, 0), tile_vertex(tile, poly, j - 1), tile_vertex(tile, poly, j));
    }
    return area;
}

static void random_point_in_poly(FindRandomPointResult &out, const NavMesh &nav_mesh, const NavMeshTile &tile, const NavMeshPoly &poly,
                                 NodeRef poly_ref, RandomFunction random, void *random_user_data) {
    PolyVertices vertices;
    uint32_t vertex_count = nav_mesh_api::gather_poly_vertices(tile, poly, vertices);

    float s = random(random_user_data);
    float t = random(random_user_data);
    float inline_areas[INLINE_POLY_VERTICES];
    Array<float> overflow_areas = {};
    float *areas = inline_areas;
    if (vertex_count > INLINE_POLY_VERTICES) {
        overflow_areas.resize(vertex_count, 0);
        areas = overflow_areas.data;
    }

    Vec3 point = geometry::random_point_in_convex_poly((int)vertex_count, vertices.data, areas, s, t);

    GetClosestPointOnPolyResult closest;
    nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, poly_ref, point);

    overflow_areas.release();
    nav_mesh_api::release(vertices);
    out = {true, poly_ref, closest.success ? closest.position : point};
}

void find_random_point(FindRandomPointResult &out, const NavMesh &nav_mesh, const QueryFilter &filter, RandomFunction random, void *random_user_data) {
    out = {false, INVALID_NODE_REF, {0, 0, 0}};

    const NavMeshTile *selected_tile = nullptr;
    float tile_sum = 0;

    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        const float area = 1.0f;
        tile_sum += area;
        float u = random(random_user_data);
        if (u * tile_sum <= area) selected_tile = &tile;
    }

    if (!selected_tile) return;

    const NavMeshPoly *selected_poly = nullptr;
    NodeRef selected_poly_ref = INVALID_NODE_REF;
    float area_sum = 0;

    for (uint32_t i = 0; i < selected_tile->polys.length; i++) {
        const NavMeshPoly &poly = selected_tile->polys[i];
        NodeRef ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, *selected_tile, (int)i).ref;

        if (!nav_mesh_api::passes_filter(filter, ref, nav_mesh)) continue;

        float area = poly_area(*selected_tile, poly);
        area_sum += area;
        float u = random(random_user_data);
        if (u * area_sum <= area) {
            selected_poly = &poly;
            selected_poly_ref = ref;
        }
    }

    if (!selected_poly || selected_poly_ref == INVALID_NODE_REF) return;

    random_point_in_poly(out, nav_mesh, *selected_tile, *selected_poly, selected_poly_ref, random, random_user_data);
}

void find_random_point_around_circle(FindRandomPointResult &out, QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef start_node_ref,
                                     Vec3 position, float max_radius, const QueryFilter &filter, RandomFunction random, void *random_user_data) {
    out = {false, INVALID_NODE_REF, {0, 0, 0}};

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(position) || max_radius < 0 || !isfinite(max_radius)) return;

    if (!nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, start_node_ref).success) return;
    if (!nav_mesh_api::passes_filter(filter, start_node_ref, nav_mesh)) return;

    SearchNodePool &nodes = workspace.nodes;
    SearchNodeQueue &open_list = workspace.open_list;
    clear(nodes);
    open_list.items.clear();
    workspace.last_path_node = NULL_SEARCH_NODE;

    uint32_t start_index = add_search_node(nodes, make_search_node(start_node_ref, 0, NODE_FLAG_OPEN, position));
    push_node_to_queue(open_list, nodes, start_index);

    float radius_sqr = max_radius * max_radius;
    float area_sum = 0;

    const NavMeshTile *random_tile = nullptr;
    const NavMeshPoly *random_poly = nullptr;
    NodeRef random_poly_ref = INVALID_NODE_REF;

    while (open_list.items.length > 0) {
        uint32_t best_index = (uint32_t)pop_node_from_queue(open_list, nodes);
        nodes.nodes[best_index].flags &= ~NODE_FLAG_OPEN;
        nodes.nodes[best_index].flags |= NODE_FLAG_CLOSED;
        SearchNode best = nodes.nodes[best_index];

        NodeRef best_ref = best.node_ref;
        TileAndPoly best_tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, best_ref);
        if (!best_tile_and_poly.success) continue;

        float area = poly_area(*best_tile_and_poly.tile, *best_tile_and_poly.poly);
        area_sum += area;
        float u = random(random_user_data);
        if (u * area_sum <= area) {
            random_tile = best_tile_and_poly.tile;
            random_poly = best_tile_and_poly.poly;
            random_poly_ref = best_ref;
        }

        NodeRef parent_ref = best.parent_node_ref;
        const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, best_ref);

        for (int32_t link_index = node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            NodeRef neighbour_ref = nav_mesh.links[(uint32_t)link_index].to_node_ref;

            if (neighbour_ref == INVALID_NODE_REF || neighbour_ref == parent_ref) continue;
            if (!nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, neighbour_ref).success) continue;
            if (!nav_mesh_api::passes_filter(filter, neighbour_ref, nav_mesh)) continue;

            GetPortalPointsResult portal;
            get_portal_points_unchecked(portal, nav_mesh, best_ref, neighbour_ref);
            if (!portal.success) continue;

            if (geometry::distance_pt_seg_sqr_2d(position, portal.left, portal.right).dist_sqr > radius_sqr) continue;

            int32_t neighbour_index = get_search_node(nodes, neighbour_ref, 0);
            if (neighbour_index == NULL_SEARCH_NODE) neighbour_index = (int32_t)add_search_node(nodes, make_search_node(neighbour_ref, 0, 0, {0, 0, 0}));

            SearchNode &neighbour = nodes.nodes[(uint32_t)neighbour_index];
            if (neighbour.flags & NODE_FLAG_CLOSED) continue;

            if (neighbour.flags == 0) neighbour.position = vec3::lerp(portal.left, portal.right, 0.5f);

            float total = best.total + vec3::distance(best.position, neighbour.position);

            if (neighbour.flags & NODE_FLAG_OPEN && total >= neighbour.total) continue;

            neighbour.parent_node_ref = best_ref;
            neighbour.parent_state = 0;
            neighbour.flags = neighbour.flags & ~NODE_FLAG_CLOSED;
            neighbour.total = total;

            if (neighbour.flags & NODE_FLAG_OPEN) {
                reindex_node_in_queue(open_list, nodes, (uint32_t)neighbour_index);
            } else {
                neighbour.flags = NODE_FLAG_OPEN;
                push_node_to_queue(open_list, nodes, (uint32_t)neighbour_index);
            }
        }
    }

    if (!random_poly || !random_tile || random_poly_ref == INVALID_NODE_REF) return;

    random_point_in_poly(out, nav_mesh, *random_tile, *random_poly, random_poly_ref, random, random_user_data);
}



static uint32_t gather_reachability_clusters(const NavMesh &nav_mesh, NodeRef node_ref, bool as_target, uint32_t *clusters) {
    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, node_ref)) return 0;
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);
    if (node.type == NODE_TYPE_POLY) {
        clusters[0] = nav_mesh.tiles[(uint32_t)node.tile_id].poly_clusters[(uint32_t)node.poly_index];
        return 1;
    }

    uint32_t id = (uint32_t)node.off_mesh_connection_id;
    const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[id];
    bool bidirectional = nav_mesh.off_mesh_connections[id].direction == OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL;
    uint32_t count = 0;
    if (as_target || bidirectional) {
        const NavMeshNode &start = nav_mesh_api::get_node_by_ref(nav_mesh, attachment.start_poly_node);
        clusters[count++] = nav_mesh.tiles[(uint32_t)start.tile_id].poly_clusters[(uint32_t)start.poly_index];
    }
    if (!as_target || bidirectional) {
        const NavMeshNode &end = nav_mesh_api::get_node_by_ref(nav_mesh, attachment.end_poly_node);
        clusters[count++] = nav_mesh.tiles[(uint32_t)end.tile_id].poly_clusters[(uint32_t)end.poly_index];
    }
    return count;
}

bool is_reachable(QueryWorkspace &workspace, const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref) {
    uint32_t sources[2];
    uint32_t targets[2];
    uint32_t source_count = gather_reachability_clusters(nav_mesh, from_node_ref, false, sources);
    uint32_t target_count = gather_reachability_clusters(nav_mesh, to_node_ref, true, targets);
    if (source_count == 0 || target_count == 0) return false;
    if (from_node_ref == to_node_ref) return true;

    for (uint32_t i = 0; i < source_count; i++) {
        for (uint32_t j = 0; j < target_count; j++) {
            if (sources[i] == targets[j]) return true;
        }
    }

    Array<uint32_t> &stamps = workspace.cluster_stamps;
    if (stamps.length < nav_mesh.clusters.length) stamps.resize(nav_mesh.clusters.length, 0);
    if (++workspace.cluster_stamp == 0) {
        for (uint32_t &stamp : stamps) stamp = 0;
        workspace.cluster_stamp = 1;
    }
    uint32_t stamp = workspace.cluster_stamp;

    Array<uint32_t> &queue = workspace.cluster_queue;
    queue.clear();
    for (uint32_t i = 0; i < source_count; i++) {
        stamps[sources[i]] = stamp;
        queue.push(sources[i]);
    }

    for (uint32_t head = 0; head < queue.length; head++) {
        for (int32_t i = nav_mesh.clusters[queue[head]].first_link; i != NULL_LINK; i = nav_mesh.cluster_links[(uint32_t)i].next) {
            uint32_t cluster = nav_mesh.cluster_links[(uint32_t)i].to_cluster;
            if (stamps[cluster] == stamp) continue;
            if (cluster == targets[0] || (target_count > 1 && cluster == targets[1])) return true;
            stamps[cluster] = stamp;
            queue.push(cluster);
        }
    }

    return false;
}

}

}
