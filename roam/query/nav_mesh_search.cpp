#include "roam/query/nav_mesh_search.h"

#include <float.h>
#include <math.h>

#include "roam/geometry/geometry.h"

namespace roam {

namespace nav_mesh_search {

static const float HEURISTIC_SCALE = 0.999f;

int32_t get_search_node(const SearchNodePool &pool, NodeRef node_ref, int state) {
    const int32_t *first = pool.first_by_ref.get(node_ref);
    if (!first) return NULL_SEARCH_NODE;
    for (int32_t index = *first; index != NULL_SEARCH_NODE; index = pool.nodes[(uint32_t)index].next_with_same_ref) {
        if (pool.nodes[(uint32_t)index].state == state) return index;
    }
    return NULL_SEARCH_NODE;
}

uint32_t add_search_node(SearchNodePool &pool, const SearchNode &node) {
    uint32_t index = pool.nodes.length;
    pool.nodes.push(node);
    pool.nodes[index].next_with_same_ref = NULL_SEARCH_NODE;

    int32_t *first = pool.first_by_ref.get(node.node_ref);
    if (!first) {
        pool.first_by_ref.set(node.node_ref, (int32_t)index);
        return index;
    }

    int32_t last = *first;
    while (pool.nodes[(uint32_t)last].next_with_same_ref != NULL_SEARCH_NODE) last = pool.nodes[(uint32_t)last].next_with_same_ref;
    pool.nodes[(uint32_t)last].next_with_same_ref = (int32_t)index;
    return index;
}

void release(SearchNodePool &pool) {
    pool.nodes.release();
    pool.first_by_ref.release();
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

bool get_portal_points(const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref, Vec3 &out_left, Vec3 &out_right) {
    const NavMeshNode &from_node = nav_mesh_api::get_node_by_ref(nav_mesh, from_node_ref);

    const NavMeshLink *to_link = find_link_to(nav_mesh, from_node, to_node_ref);
    if (!to_link) return false;

    NodeType from_type = node::get_node_ref_type(from_node_ref);
    NodeType to_type = node::get_node_ref_type(to_node_ref);

    if (from_type == NODE_TYPE_POLY && to_type == NODE_TYPE_POLY) {
        const NavMeshTile &from_tile = nav_mesh.tiles[(uint32_t)from_node.tile_id];
        const NavMeshPoly &from_poly = from_tile.polys[(uint32_t)from_node.poly_index];

        Vec3 v0 = tile_vertex(from_tile, from_poly, (uint32_t)to_link->edge);
        Vec3 v1 = tile_vertex(from_tile, from_poly, ((uint32_t)to_link->edge + 1) % from_poly.vertex_count);

        if (to_link->side != 0xff && (to_link->bmin != 0 || to_link->bmax != 255)) {
            const float s = 1.0f / 255.0f;
            out_left = vec3::lerp(v0, v1, (float)to_link->bmin * s);
            out_right = vec3::lerp(v0, v1, (float)to_link->bmax * s);
        } else {
            out_left = v0;
            out_right = v1;
        }

        return true;
    }

    if (from_type == NODE_TYPE_POLY && to_type == NODE_TYPE_OFFMESH) {
        const OffMeshConnection *connection = off_mesh_connection_of(nav_mesh, nav_mesh_api::get_node_by_ref(nav_mesh, to_node_ref));
        if (!connection) return false;
        Vec3 position = to_link->edge == 0 ? connection->start : connection->end;
        out_left = position;
        out_right = position;
        return true;
    }

    if (from_type == NODE_TYPE_OFFMESH && to_type == NODE_TYPE_POLY) {
        const OffMeshConnection *connection = off_mesh_connection_of(nav_mesh, from_node);
        if (!connection) return false;
        Vec3 position = to_link->edge == 0 ? connection->start : connection->end;
        out_left = position;
        out_right = position;
        return true;
    }

    return false;
}

bool get_edge_mid_point(const NavMesh &nav_mesh, NodeRef from_node_ref, NodeRef to_node_ref, Vec3 &out_mid_point) {
    Vec3 left;
    Vec3 right;
    if (!get_portal_points(nav_mesh, from_node_ref, to_node_ref, left, right)) return false;
    out_mid_point = vec3::scale(vec3::add(left, right), 0.5f);
    return true;
}

static Array<NodeRef> collect_path(const SearchNodePool &pool, int32_t last_node) {
    Array<NodeRef> path = {};
    int32_t current = last_node;
    while (current != NULL_SEARCH_NODE) {
        const SearchNode &node = pool.nodes[(uint32_t)current];
        path.push(node.node_ref);
        current = node.parent_node_ref != INVALID_NODE_REF && node.parent_state != NO_PARENT_STATE
                      ? get_search_node(pool, node.parent_node_ref, node.parent_state)
                      : NULL_SEARCH_NODE;
    }
    for (uint32_t i = 0, j = path.length; i + 1 < j; i++, j--) {
        NodeRef swap = path[i];
        path[i] = path[j - 1];
        path[j - 1] = swap;
    }
    return path;
}

static RaycastResult raycast_base(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
                                  bool calculate_costs, NodeRef previous_ref);

static int link_state(const NavMeshLink &link) { return link.side != 0xff ? link.side >> 1 : 0; }

static uint32_t get_or_add_neighbour(const NavMesh &nav_mesh, SearchNodePool &pool, NodeRef best_node_ref, NodeRef neighbour_node_ref, int state) {
    int32_t existing = get_search_node(pool, neighbour_node_ref, state);
    if (existing != NULL_SEARCH_NODE) return (uint32_t)existing;

    SearchNode node = make_search_node(neighbour_node_ref, state, 0, {0, 0, 0});
    get_edge_mid_point(nav_mesh, best_node_ref, neighbour_node_ref, node.position);
    return add_search_node(pool, node);
}

FindNodePathResult find_node_path(const NavMesh &nav_mesh, NodeRef start_node_ref, NodeRef end_node_ref, Vec3 start_position, Vec3 end_position,
                                  const QueryFilter &filter, FindNodePathOptions options) {
    FindNodePathResult result = {};
    float raycast_distance = options.raycast_distance;
    SearchNodePool &nodes = result.nodes;
    SearchNodeQueue &open_list = result.open_list;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !nav_mesh_api::is_valid_node_ref(nav_mesh, end_node_ref) ||
        !vec3::finite(start_position) || !vec3::finite(end_position)) {
        result.flags = FIND_NODE_PATH_RESULT_NONE | FIND_NODE_PATH_RESULT_INVALID_INPUT;
        return result;
    }

    if (start_node_ref == end_node_ref) {
        result.flags = FIND_NODE_PATH_RESULT_SUCCESS | FIND_NODE_PATH_RESULT_COMPLETE_PATH;
        result.success = true;
        result.path.push(start_node_ref);
        return result;
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
            if (!filter.pass_filter(filter, neighbour_node_ref, nav_mesh)) continue;

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
                        RaycastResult ray = raycast_base(nav_mesh, grandparent.node_ref, grandparent.position, neighbour_position, filter, true,
                                                         grandparent.parent_node_ref);
                        if (ray.t >= 1.0f) {
                            found_shortcut = true;
                            cost = grandparent.cost + ray.path_cost;
                        }
                        release(ray);
                    }
                }
            }

            if (!found_shortcut) {
                float current_cost =
                    filter.get_cost(filter, best.position, neighbour_position, nav_mesh, parent_node_ref, best_node_ref, neighbour_node_ref);
                cost = best.cost + current_cost;
            }

            if (neighbour_node_ref == end_node_ref) {
                float end_cost = filter.get_cost(filter, neighbour_position, end_position, nav_mesh, best_node_ref, neighbour_node_ref, INVALID_NODE_REF);
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

    result.path = collect_path(nodes, last_best_node);
    result.success = true;

    if (nodes.nodes[(uint32_t)last_best_node].node_ref != end_node_ref) {
        result.flags = FIND_NODE_PATH_RESULT_PARTIAL_PATH;
        return result;
    }

    result.flags = FIND_NODE_PATH_RESULT_SUCCESS | FIND_NODE_PATH_RESULT_COMPLETE_PATH;
    return result;
}

void release(FindNodePathResult &result) {
    result.path.release();
    release(result.nodes);
    release(result.open_list);
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

uint32_t init_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query, NodeRef start_node_ref, NodeRef end_node_ref,
                                    Vec3 start_position, Vec3 end_position, const QueryFilter &filter, uint32_t flags) {
    query.start_node_ref = start_node_ref;
    query.end_node_ref = end_node_ref;
    query.start_position = start_position;
    query.end_position = end_position;
    query.filter = filter;

    query.status = SLICED_FIND_NODE_PATH_FAILURE;
    query.nodes.nodes.clear();
    query.nodes.first_by_ref.clear();
    query.open_list.items.clear();
    query.last_best_node = NULL_SEARCH_NODE;
    query.last_best_node_cost = INFINITY;

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !nav_mesh_api::is_valid_node_ref(nav_mesh, end_node_ref) ||
        !vec3::finite(start_position) || !vec3::finite(end_position)) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE | SLICED_FIND_NODE_PATH_INVALID_PARAM;
        return query.status;
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
        return query.status;
    }

    push_node_to_queue(query.open_list, query.nodes, start_index);
    query.status = SLICED_FIND_NODE_PATH_IN_PROGRESS;

    return query.status;
}

int update_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query, int max_iterations) {
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
            if (!filter.pass_filter(filter, neighbour_node_ref, nav_mesh)) continue;

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
                        RaycastResult ray = raycast_base(nav_mesh, grandparent.node_ref, grandparent.position, neighbour_position, filter, true,
                                                         grandparent.parent_node_ref);
                        if (ray.t >= 1.0f) {
                            found_shortcut = true;
                            cost = grandparent.cost + ray.path_cost;
                        }
                        release(ray);
                    }
                }
            }

            if (!found_shortcut) {
                float current_cost =
                    filter.get_cost(filter, best.position, neighbour_position, nav_mesh, parent_node_ref, best_node_ref, neighbour_node_ref);
                cost = best.cost + current_cost;
            }

            if (neighbour_node_ref == query.end_node_ref) {
                float end_cost =
                    filter.get_cost(filter, neighbour_position, query.end_position, nav_mesh, best_node_ref, neighbour_node_ref, INVALID_NODE_REF);
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

static Array<NodeRef> build_sliced_path(const NavMesh &nav_mesh, const SlicedNodePathQuery &query, int32_t last_node) {
    Array<uint32_t> forward = {};
    for (int32_t current = last_node; current != NULL_SEARCH_NODE;) {
        forward.push((uint32_t)current);
        const SearchNode &node = query.nodes.nodes[(uint32_t)current];
        current = node.parent_node_ref != INVALID_NODE_REF && node.parent_state != NO_PARENT_STATE
                      ? get_search_node(query.nodes, node.parent_node_ref, node.parent_state)
                      : NULL_SEARCH_NODE;
    }
    for (uint32_t i = 0, j = forward.length; i + 1 < j; i++, j--) {
        uint32_t swap = forward[i];
        forward[i] = forward[j - 1];
        forward[j - 1] = swap;
    }

    Array<NodeRef> path = {};

    for (uint32_t i = 0; i < forward.length; i++) {
        const SearchNode &current = query.nodes.nodes[forward[i]];
        bool was_detached = (current.flags & NODE_FLAG_PARENT_DETACHED) != 0;
        bool has_next = i + 1 < forward.length;

        if (query.has_raycast_limit && has_next && was_detached) {
            const SearchNode &next = query.nodes.nodes[forward[i + 1]];
            RaycastResult ray = raycast(nav_mesh, current.node_ref, current.position, next.position, query.filter);
            for (NodeRef ref : ray.path) path.push(ref);
            if (path.length > 0 && path.back() == next.node_ref) path.pop();
            release(ray);
        } else {
            path.push(current.node_ref);
        }
    }

    forward.release();
    return path;
}

SlicedFindNodePathResult finalize_sliced_find_node_path(const NavMesh &nav_mesh, SlicedNodePathQuery &query) {
    SlicedFindNodePathResult result = {SLICED_FIND_NODE_PATH_FAILURE, {}};

    if (query.last_best_node == NULL_SEARCH_NODE) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE;
        result.status = query.status;
        return result;
    }

    if (query.start_node_ref == query.end_node_ref) {
        result.path.push(query.start_node_ref);
        result.status = SLICED_FIND_NODE_PATH_SUCCESS;
        query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
        return result;
    }

    if (query.nodes.nodes[(uint32_t)query.last_best_node].node_ref != query.end_node_ref) query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    result.path = build_sliced_path(nav_mesh, query, query.last_best_node);
    result.status = SLICED_FIND_NODE_PATH_SUCCESS | (query.status & SLICED_FIND_NODE_PATH_PARTIAL_RESULT);

    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;

    return result;
}

SlicedFindNodePathResult finalize_sliced_find_node_path_partial(const NavMesh &nav_mesh, SlicedNodePathQuery &query, const NodeRef *existing_path,
                                                                uint32_t existing_path_count) {
    SlicedFindNodePathResult result = {SLICED_FIND_NODE_PATH_FAILURE, {}};

    int32_t furthest_node = NULL_SEARCH_NODE;

    for (uint32_t i = existing_path_count; i-- > 0;) {
        const int32_t *first = query.nodes.first_by_ref.get(existing_path[i]);
        if (first) {
            furthest_node = *first;
            break;
        }
    }

    if (furthest_node == NULL_SEARCH_NODE) {
        furthest_node = query.last_best_node;
        query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;
    }

    if (furthest_node == NULL_SEARCH_NODE) {
        query.status = SLICED_FIND_NODE_PATH_FAILURE;
        result.status = query.status;
        return result;
    }

    if (query.start_node_ref == query.end_node_ref) {
        result.path.push(query.start_node_ref);
        result.status = SLICED_FIND_NODE_PATH_SUCCESS;
        query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;
        return result;
    }

    query.status |= SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    result.path = build_sliced_path(nav_mesh, query, furthest_node);
    result.status = SLICED_FIND_NODE_PATH_SUCCESS | SLICED_FIND_NODE_PATH_PARTIAL_RESULT;

    query.status = SLICED_FIND_NODE_PATH_NOT_INITIALIZED;

    return result;
}

void release(SlicedNodePathQuery &query) {
    release(query.nodes);
    release(query.open_list);
}

void release(SlicedFindNodePathResult &result) { result.path.release(); }

static uint32_t gather_poly_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, Array<float> &out_vertices) {
    out_vertices.resize(poly.vertex_count * 3);
    for (uint32_t i = 0; i < poly.vertex_count; ++i) vec3::store(out_vertices.data, i * 3, tile_vertex(tile, poly, i));
    return poly.vertex_count;
}

MoveAlongSurfaceResult move_along_surface(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position,
                                          const QueryFilter &filter) {
    MoveAlongSurfaceResult result = {false, start_position, start_node_ref, {}};

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(start_position) || !vec3::finite(end_position)) return result;

    result.success = true;

    SearchNodePool nodes = {};
    uint32_t start_index = add_search_node(nodes, make_search_node(start_node_ref, 0, NODE_FLAG_CLOSED, start_position));

    Vec3 best_position = start_position;
    float best_distance = INFINITY;
    int32_t best_node = (int32_t)start_index;

    Vec3 search_position = vec3::lerp(start_position, end_position, 0.5f);
    float search_radius = vec3::distance(start_position, end_position) / 2.0f + 0.001f;
    float search_radius_sqr = search_radius * search_radius;

    Array<uint32_t> queue = {};
    queue.push(start_index);
    Array<float> vertices = {};
    Array<NodeRef> neighbours = {};

    for (uint32_t head = 0; head < queue.length; head++) {
        uint32_t current_index = queue[head];
        NodeRef current_ref = nodes.nodes[current_index].node_ref;

        TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(current_ref, nav_mesh);
        if (!tile_and_poly.success) continue;

        const NavMeshTile &tile = *tile_and_poly.tile;
        const NavMeshPoly &poly = *tile_and_poly.poly;

        uint32_t vertex_count = gather_poly_vertices(tile, poly, vertices);

        if (geometry::point_in_poly(end_position, vertices.data, (int)vertex_count)) {
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
                if (!filter.pass_filter(filter, neighbour_ref, nav_mesh)) continue;
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

    for (int32_t current = best_node; current != NULL_SEARCH_NODE;) {
        const SearchNode &node = nodes.nodes[(uint32_t)current];
        result.visited.push(node.node_ref);
        current = node.parent_node_ref != INVALID_NODE_REF ? get_search_node(nodes, node.parent_node_ref, 0) : NULL_SEARCH_NODE;
    }
    for (uint32_t i = 0, j = result.visited.length; i + 1 < j; i++, j--) {
        NodeRef swap = result.visited[i];
        result.visited[i] = result.visited[j - 1];
        result.visited[j - 1] = swap;
    }

    result.position = best_position;
    result.node_ref = nodes.nodes[(uint32_t)best_node].node_ref;

    TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(result.node_ref, nav_mesh);
    if (tile_and_poly.success) {
        GetPolyHeightResult height = nav_mesh_api::get_poly_height(*tile_and_poly.tile, *tile_and_poly.poly, tile_and_poly.poly_index, result.position);
        if (height.success) result.position.y = height.height;
    }

    neighbours.release();
    vertices.release();
    queue.release();
    release(nodes);
    return result;
}

void release(MoveAlongSurfaceResult &result) { result.visited.release(); }

static RaycastResult raycast_base(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
                                  bool calculate_costs, NodeRef previous_ref) {
    RaycastResult result = {0, {0, 0, 0}, -1, {}, 0};

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(start_position) || !vec3::finite(end_position)) return result;

    NodeRef current_ref = start_node_ref;
    NodeRef previous_ref_tracking = previous_ref;

    Vec3 direction = {0, 0, 0};
    Vec3 current_position = {0, 0, 0};
    if (calculate_costs) {
        direction = vec3::subtract(end_position, start_position);
        current_position = start_position;
    }

    Array<float> vertices = {};

    while (current_ref != INVALID_NODE_REF) {
        TileAndPoly tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(current_ref, nav_mesh);
        if (!tile_and_poly.success) break;
        const NavMeshTile &tile = *tile_and_poly.tile;
        const NavMeshPoly &poly = *tile_and_poly.poly;

        uint32_t vertex_count = gather_poly_vertices(tile, poly, vertices);

        geometry::IntersectSegmentPoly2dResult intersection =
            geometry::intersect_segment_poly_2d(start_position, end_position, (int)vertex_count, vertices.data);
        if (!intersection.intersects) break;

        result.hit_edge_index = intersection.seg_max;

        if (intersection.tmax > result.t) result.t = intersection.tmax;

        result.path.push(current_ref);

        if (intersection.seg_max == -1) {
            result.t = FLT_MAX;
            if (calculate_costs) {
                result.path_cost +=
                    filter.get_cost(filter, current_position, end_position, nav_mesh, previous_ref_tracking, current_ref, INVALID_NODE_REF);
            }
            break;
        }

        NodeRef next_ref = INVALID_NODE_REF;

        const NavMeshNode &current_node = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref);

        for (int32_t link_index = current_node.first_link; link_index != NULL_LINK; link_index = nav_mesh.links[(uint32_t)link_index].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)link_index];

            if (link.edge != intersection.seg_max) continue;
            if (node::get_node_ref_type(link.to_node_ref) == NODE_TYPE_OFFMESH) continue;
            if (!nav_mesh_api::get_tile_and_poly_by_ref(link.to_node_ref, nav_mesh).success) continue;
            if (!filter.pass_filter(filter, link.to_node_ref, nav_mesh)) continue;

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
                    filter.get_cost(filter, last_position, current_position, nav_mesh, previous_ref_tracking, current_ref, INVALID_NODE_REF);
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

            result.path_cost += filter.get_cost(filter, last_position, current_position, nav_mesh, previous_ref_tracking, current_ref, next_ref);
        }

        previous_ref_tracking = current_ref;
        current_ref = next_ref;
    }

    vertices.release();
    return result;
}

RaycastResult raycast(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter) {
    return raycast_base(nav_mesh, start_node_ref, start_position, end_position, filter, false, INVALID_NODE_REF);
}

RaycastResult raycast_with_costs(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 start_position, Vec3 end_position, const QueryFilter &filter,
                                 NodeRef previous_ref) {
    return raycast_base(nav_mesh, start_node_ref, start_position, end_position, filter, true, previous_ref);
}

void release(RaycastResult &result) { result.path.release(); }

static float poly_area(const NavMeshTile &tile, const NavMeshPoly &poly) {
    float area = 0;
    for (uint32_t j = 2; j < poly.vertex_count; j++) {
        area += geometry::tri_area_2d(tile_vertex(tile, poly, 0), tile_vertex(tile, poly, j - 1), tile_vertex(tile, poly, j));
    }
    return area;
}

static FindRandomPointResult random_point_in_poly(const NavMesh &nav_mesh, const NavMeshTile &tile, const NavMeshPoly &poly, NodeRef poly_ref,
                                                  RandomFunction random, void *random_user_data) {
    Array<float> vertices = {};
    uint32_t vertex_count = gather_poly_vertices(tile, poly, vertices);

    float s = random(random_user_data);
    float t = random(random_user_data);
    Array<float> areas = {};
    areas.resize(vertex_count, 0);

    Vec3 point = geometry::random_point_in_convex_poly((int)vertex_count, vertices.data, areas.data, s, t);

    GetClosestPointOnPolyResult closest = nav_mesh_api::get_closest_point_on_poly(nav_mesh, poly_ref, point);

    areas.release();
    vertices.release();
    return {true, poly_ref, closest.success ? closest.position : point};
}

FindRandomPointResult find_random_point(const NavMesh &nav_mesh, const QueryFilter &filter, RandomFunction random, void *random_user_data) {
    FindRandomPointResult result = {false, INVALID_NODE_REF, {0, 0, 0}};

    const NavMeshTile *selected_tile = nullptr;
    float tile_sum = 0;

    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        const float area = 1.0f;
        tile_sum += area;
        float u = random(random_user_data);
        if (u * tile_sum <= area) selected_tile = &tile;
    }

    if (!selected_tile) return result;

    const NavMeshPoly *selected_poly = nullptr;
    NodeRef selected_poly_ref = INVALID_NODE_REF;
    float area_sum = 0;

    for (uint32_t i = 0; i < selected_tile->polys.length; i++) {
        const NavMeshPoly &poly = selected_tile->polys[i];
        NodeRef ref = nav_mesh_api::get_node_by_tile_and_poly(nav_mesh, *selected_tile, (int)i).ref;

        if (!filter.pass_filter(filter, ref, nav_mesh)) continue;

        float area = poly_area(*selected_tile, poly);
        area_sum += area;
        float u = random(random_user_data);
        if (u * area_sum <= area) {
            selected_poly = &poly;
            selected_poly_ref = ref;
        }
    }

    if (!selected_poly || selected_poly_ref == INVALID_NODE_REF) return result;

    return random_point_in_poly(nav_mesh, *selected_tile, *selected_poly, selected_poly_ref, random, random_user_data);
}

FindRandomPointResult find_random_point_around_circle(const NavMesh &nav_mesh, NodeRef start_node_ref, Vec3 position, float max_radius,
                                                      const QueryFilter &filter, RandomFunction random, void *random_user_data) {
    FindRandomPointResult result = {false, INVALID_NODE_REF, {0, 0, 0}};

    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, start_node_ref) || !vec3::finite(position) || max_radius < 0 || !isfinite(max_radius)) {
        return result;
    }

    if (!nav_mesh_api::get_tile_and_poly_by_ref(start_node_ref, nav_mesh).success) return result;
    if (!filter.pass_filter(filter, start_node_ref, nav_mesh)) return result;

    SearchNodePool nodes = {};
    SearchNodeQueue open_list = {};

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
        TileAndPoly best_tile_and_poly = nav_mesh_api::get_tile_and_poly_by_ref(best_ref, nav_mesh);
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
            if (!nav_mesh_api::get_tile_and_poly_by_ref(neighbour_ref, nav_mesh).success) continue;
            if (!filter.pass_filter(filter, neighbour_ref, nav_mesh)) continue;

            Vec3 va;
            Vec3 vb;
            if (!get_portal_points(nav_mesh, best_ref, neighbour_ref, va, vb)) continue;

            if (geometry::distance_pt_seg_sqr_2d(position, va, vb).dist_sqr > radius_sqr) continue;

            int32_t neighbour_index = get_search_node(nodes, neighbour_ref, 0);
            if (neighbour_index == NULL_SEARCH_NODE) neighbour_index = (int32_t)add_search_node(nodes, make_search_node(neighbour_ref, 0, 0, {0, 0, 0}));

            SearchNode &neighbour = nodes.nodes[(uint32_t)neighbour_index];
            if (neighbour.flags & NODE_FLAG_CLOSED) continue;

            if (neighbour.flags == 0) neighbour.position = vec3::lerp(va, vb, 0.5f);

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

    release(nodes);
    release(open_list);

    if (!random_poly || !random_tile || random_poly_ref == INVALID_NODE_REF) return result;

    return random_point_in_poly(nav_mesh, *random_tile, *random_poly, random_poly_ref, random, random_user_data);
}

}

}
