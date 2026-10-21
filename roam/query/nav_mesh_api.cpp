#include "roam/query/nav_mesh_api.h"

#include <float.h>
#include <math.h>

#include "roam/generate/common.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/geometry/geometry.h"
#include "roam/query/bv_tree.h"

namespace roam {

namespace nav_mesh_api {

float distance_cost(const QueryFilter &, Vec3 pa, Vec3 pb, const NavMesh &, NodeRef, NodeRef, NodeRef) { return vec3::distance(pa, pb); }

static bool pass_any(const QueryFilter &, NodeRef, const NavMesh &) { return true; }

bool pass_flags(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) { return passes_flags(filter, nav_mesh.nodes[node::get_node_ref_index(node_ref)]); }

const QueryFilter ANY_QUERY_FILTER = {pass_any, distance_cost, 0xffffffffu, 0, nullptr};
const QueryFilter DEFAULT_QUERY_FILTER = {pass_flags, distance_cost, 0xffffffffu, 0, nullptr};

QueryFilter create_default_query_filter() { return DEFAULT_QUERY_FILTER; }

NavMesh create_nav_mesh() { return {}; }

static void release_tile_data(NavMeshTile &tile) {
    tile.vertices.release();
    tile.polys.release();
    tile.poly_vertices.release();
    tile.poly_neis.release();
    tile.detail_meshes.release();
    tile.detail_vertices.release();
    tile.detail_triangles.release();
    tile.poly_nodes.release();
    tile.portals.release();
    tile.bv_tree.nodes.release();
}

void release(NavMesh &nav_mesh) {
    for (NavMeshTile &tile : nav_mesh.tiles) release_tile_data(tile);
    nav_mesh.tiles.release();
    nav_mesh.nodes.release();
    nav_mesh.links.release();
    nav_mesh.off_mesh_connections.release();
    nav_mesh.off_mesh_connection_attachments.release();
    nav_mesh.tile_position_to_tile_id.release();
    for (uint32_t i = 0; i < nav_mesh.tile_column_to_tile_ids.capacity; i++) {
        if (nav_mesh.tile_column_to_tile_ids.slots[i] == MAP_SLOT_OCCUPIED) nav_mesh.tile_column_to_tile_ids.values[i].release();
    }
    nav_mesh.tile_column_to_tile_ids.release();
    nav_mesh.tile_position_to_sequence_counter.release();
    index_pool::release(nav_mesh.node_index_pool);
    index_pool::release(nav_mesh.tile_index_pool);
    index_pool::release(nav_mesh.off_mesh_connection_index_pool);
    index_pool::release(nav_mesh.link_index_pool);
}

static const NavMeshTile *tile_by_id(const NavMesh &nav_mesh, int32_t tile_id) {
    if (tile_id < 0 || (uint32_t)tile_id >= nav_mesh.tiles.length) return nullptr;
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)tile_id];
    return tile.allocated ? &tile : nullptr;
}

static const OffMeshConnection *off_mesh_connection_by_id(const NavMesh &nav_mesh, int32_t id) {
    if (id < 0 || (uint32_t)id >= nav_mesh.off_mesh_connections.length) return nullptr;
    const OffMeshConnection &connection = nav_mesh.off_mesh_connections[(uint32_t)id];
    return connection.allocated ? &connection : nullptr;
}

static const OffMeshConnectionAttachment *attachment_by_id(const NavMesh &nav_mesh, uint32_t id) {
    if (id >= nav_mesh.off_mesh_connection_attachments.length) return nullptr;
    const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[id];
    return attachment.attached ? &attachment : nullptr;
}

bool is_valid_node_ref(const NavMesh &nav_mesh, NodeRef node_ref) {
    if (node_ref == INVALID_NODE_REF) return false;

    uint32_t node_index = node::get_node_ref_index(node_ref);
    if (node_index >= nav_mesh.nodes.length) return false;
    const NavMeshNode &node = nav_mesh.nodes[node_index];
    uint32_t sequence = node::get_node_ref_sequence(node_ref);

    if (node::get_node_ref_type(node_ref) == NODE_TYPE_POLY) {
        const NavMeshTile *tile = tile_by_id(nav_mesh, node.tile_id);
        if (!tile) return false;
        if (tile->sequence != sequence) return false;
        if (node.poly_index < 0 || (uint32_t)node.poly_index >= tile->polys.length) return false;
        return true;
    }

    const OffMeshConnection *connection = off_mesh_connection_by_id(nav_mesh, node.off_mesh_connection_id);
    if (!connection) return false;
    if (connection->sequence != sequence) return false;
    return is_off_mesh_connection_connected(nav_mesh, connection->id);
}

static uint64_t tile_position_key(int x, int y, int layer) {
    return ((uint64_t)((uint32_t)x & 0xffffff) << 40) | ((uint64_t)((uint32_t)y & 0xffffff) << 16) | ((uint64_t)layer & 0xffff);
}

static uint64_t tile_column_key(int x, int y) { return ((uint64_t)(uint32_t)x << 32) | (uint64_t)(uint32_t)y; }

static const Array<uint32_t> *tile_column(const NavMesh &nav_mesh, int x, int y) { return nav_mesh.tile_column_to_tile_ids.get(tile_column_key(x, y)); }

const NavMeshTile *get_tile_at(const NavMesh &nav_mesh, int x, int y, int layer) {
    const uint32_t *tile_id = nav_mesh.tile_position_to_tile_id.get(tile_position_key(x, y, layer));
    if (!tile_id) return nullptr;
    return tile_by_id(nav_mesh, (int32_t)*tile_id);
}

Span<const uint32_t> get_tile_ids_at(const NavMesh &nav_mesh, int x, int y) {
    const Array<uint32_t> *column = tile_column(nav_mesh, x, y);
    if (!column) return {};
    return *column;
}

static void neighbour_position(int x, int y, int side, int &out_x, int &out_y) {
    out_x = x;
    out_y = y;
    switch (side) {
        case 0:
            out_x++;
            break;
        case 1:
            out_x++;
            out_y++;
            break;
        case 2:
            out_y++;
            break;
        case 3:
            out_x--;
            out_y++;
            break;
        case 4:
            out_x--;
            break;
        case 5:
            out_x--;
            out_y--;
            break;
        case 6:
            out_y--;
            break;
        case 7:
            out_x++;
            out_y--;
            break;
    }
}

static const Array<uint32_t> *neighbour_tile_column(const NavMesh &nav_mesh, int x, int y, int side) {
    int nx;
    int ny;
    neighbour_position(x, y, side, nx, ny);
    return tile_column(nav_mesh, nx, ny);
}

Vec2 world_to_tile_position(const NavMesh &nav_mesh, Vec3 world_position) {
    return {floorf((world_position.x - nav_mesh.origin.x) / nav_mesh.tile_width), floorf((world_position.z - nav_mesh.origin.z) / nav_mesh.tile_height)};
}

static Vec3 tile_poly_vertex(const NavMeshTile &tile, const NavMeshPoly &poly, uint32_t i) {
    return vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3);
}

static Vec3 detail_triangle_vertex(const NavMeshTile &tile, const NavMeshPoly &poly, const NavMeshPolyDetail &detail_mesh, int vertex_index) {
    if ((uint32_t)vertex_index < poly.vertex_count) return tile_poly_vertex(tile, poly, (uint32_t)vertex_index);
    return vec3::load(tile.detail_vertices.data, (detail_mesh.vertices_base + ((uint32_t)vertex_index - poly.vertex_count)) * 3);
}

static int get_detail_tri_edge_flags(int triangle_flags, int edge_index) { return (triangle_flags >> (edge_index * 2)) & 0x3; }

void get_closest_point_on_detail_edges(GetClosestPointOnDetailEdgesResult &out, const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position,
                                                            bool only_boundary) {
    float dmin = FLT_MAX;
    float tmin = 0;
    Vec3 pmin = {0, 0, 0};
    Vec3 pmax = {0, 0, 0};

    if ((uint32_t)poly_index < tile.detail_meshes.length) {
        const NavMeshPolyDetail &detail_mesh = tile.detail_meshes[(uint32_t)poly_index];

        for (uint32_t i = 0; i < detail_mesh.triangles_count; i++) {
            uint32_t t = (detail_mesh.triangles_base + i) * 4;
            const uint8_t *triangle = &tile.detail_triangles[t];

            if (only_boundary) {
                const int ANY_BOUNDARY_EDGE = (DETAIL_EDGE_BOUNDARY << 0) | (DETAIL_EDGE_BOUNDARY << 2) | (DETAIL_EDGE_BOUNDARY << 4);
                if ((triangle[3] & ANY_BOUNDARY_EDGE) == 0) continue;
            }

            Vec3 vertices[3];
            for (int j = 0; j < 3; ++j) vertices[j] = detail_triangle_vertex(tile, poly, detail_mesh, triangle[j]);

            for (int k = 0, j = 2; k < 3; j = k++) {
                int edge_flags = get_detail_tri_edge_flags(triangle[3], j);
                if ((edge_flags & DETAIL_EDGE_BOUNDARY) == 0 && (only_boundary || triangle[j] < triangle[k])) continue;

                geometry::DistancePtSegSqr2dResult result = geometry::distance_pt_seg_sqr_2d(position, vertices[j], vertices[k]);
                if (result.dist_sqr < dmin) {
                    dmin = result.dist_sqr;
                    tmin = result.t;
                    pmin = vertices[j];
                    pmax = vertices[k];
                }
            }
        }
    }

    out = {vec3::lerp(pmin, pmax, tmin), dmin};
}

static float get_detail_mesh_height(const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position) {
    if ((uint32_t)poly_index < tile.detail_meshes.length) {
        const NavMeshPolyDetail &detail_mesh = tile.detail_meshes[(uint32_t)poly_index];
        for (uint32_t j = 0; j < detail_mesh.triangles_count; ++j) {
            const uint8_t *triangle = &tile.detail_triangles[(detail_mesh.triangles_base + j) * 4];
            Vec3 a = detail_triangle_vertex(tile, poly, detail_mesh, triangle[0]);
            Vec3 b = detail_triangle_vertex(tile, poly, detail_mesh, triangle[1]);
            Vec3 c = detail_triangle_vertex(tile, poly, detail_mesh, triangle[2]);

            float height = geometry::closest_height_point_triangle(position, a, b, c);
            if (!isnan(height)) return height;
        }
    }

    GetClosestPointOnDetailEdgesResult closest;
    get_closest_point_on_detail_edges(closest, tile, poly, poly_index, position, false);
    return closest.position.y;
}

static uint32_t gather_poly_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, float *out_vertices) {
    for (uint32_t i = 0; i < poly.vertex_count; ++i) vec3::store(out_vertices, i * 3, tile_poly_vertex(tile, poly, i));
    return poly.vertex_count;
}

uint32_t gather_poly_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, PolyVertices &vertices) {
    vertices.data = vertices.inline_values;
    if (poly.vertex_count > INLINE_POLY_VERTICES) {
        vertices.overflow.resize(poly.vertex_count * 3);
        vertices.data = vertices.overflow.data;
    }
    return gather_poly_vertices(tile, poly, vertices.data);
}

void release(PolyVertices &vertices) {
    vertices.overflow.release();
    vertices.data = vertices.inline_values;
}

void get_poly_height(GetPolyHeightResult &out, const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position) {
    PolyVertices vertices;
    gather_poly_vertices(tile, poly, vertices);

    out = {false, 0};
    if (geometry::point_in_poly(position, vertices.data, (int)poly.vertex_count)) {
        out.height = get_detail_mesh_height(tile, poly, poly_index, position);
        out.success = true;
    }

    release(vertices);
}

void get_closest_point_on_poly(GetClosestPointOnPolyResult &out, const NavMesh &nav_mesh, NodeRef node_ref, Vec3 position) {
    out = {false, false, position};

    TileAndPoly tile_and_poly = get_tile_and_poly_by_ref(nav_mesh, node_ref);
    if (!tile_and_poly.success) return;

    out.success = true;

    GetPolyHeightResult poly_height;
    get_poly_height(poly_height, *tile_and_poly.tile, *tile_and_poly.poly, tile_and_poly.poly_index, position);
    if (poly_height.success) {
        out.position = {position.x, poly_height.height, position.z};
        out.is_over_poly = true;
        return;
    }

    GetClosestPointOnDetailEdgesResult closest;
    get_closest_point_on_detail_edges(closest, *tile_and_poly.tile, *tile_and_poly.poly, tile_and_poly.poly_index, position, true);
    out.position = closest.position;
}

void get_closest_point_on_poly_boundary(GetClosestPointOnPolyBoundaryResult &out, const NavMesh &nav_mesh, NodeRef node_ref, Vec3 point) {
    out = {false, point};
    TileAndPoly tile_and_poly = get_tile_and_poly_by_ref(nav_mesh, node_ref);
    if (!tile_and_poly.success || !vec3::finite(point)) return;

    const NavMeshTile &tile = *tile_and_poly.tile;
    const NavMeshPoly &poly = *tile_and_poly.poly;

    PolyVertices vertices;
    gather_poly_vertices(tile, poly, vertices);
    uint32_t vertex_count = poly.vertex_count;

    out.success = true;

    if (geometry::point_in_poly(point, vertices.data, (int)vertex_count)) {
        out.position.y = get_detail_mesh_height(tile, poly, tile_and_poly.poly_index, point);
        release(vertices);
        return;
    }

    float dmin = FLT_MAX;
    uint32_t imin = 0;
    for (uint32_t i = 0; i < vertex_count; ++i) {
        uint32_t j = (i + 1) % vertex_count;
        geometry::DistancePtSegSqr2dResult distance =
            geometry::distance_pt_seg_sqr_2d(point, vec3::load(vertices.data, i * 3), vec3::load(vertices.data, j * 3));
        if (distance.dist_sqr < dmin) {
            dmin = distance.dist_sqr;
            imin = i;
        }
    }

    Vec3 va = vec3::load(vertices.data, imin * 3);
    Vec3 vb = vec3::load(vertices.data, ((imin + 1) % vertex_count) * 3);

    float pqx = vb.x - va.x;
    float pqz = vb.z - va.z;
    float dx = point.x - va.x;
    float dz = point.z - va.z;
    float denominator = pqx * pqx + pqz * pqz;
    float t = denominator > 0 ? (pqx * dx + pqz * dz) / denominator : 0;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    out.position = {va.x + (vb.x - va.x) * t, va.y + (vb.y - va.y) * t, va.z + (vb.z - va.z) * t};
    release(vertices);
}

using PolyVisitor = void (*)(const NavMeshTile &tile, int poly_index, NodeRef ref, void *user_data);

static void visit_polygons_in_tile(const NavMesh &nav_mesh, const NavMeshTile &tile, Box3 bounds, const QueryFilter &filter, PolyVisitor visit,
                                   void *user_data) {
    const Array<NavMeshBvNode> &nodes = tile.bv_tree.nodes;
    float quant_factor = tile.bv_tree.quant_factor;
    Vec3 tile_min = tile.bounds.min;
    Vec3 tile_max = tile.bounds.max;

    Vec3 min = vec3::subtract(vec3::max(vec3::min(bounds.min, tile_max), tile_min), tile_min);
    Vec3 max = vec3::subtract(vec3::max(vec3::min(bounds.max, tile_max), tile_min), tile_min);

    int bmin[3] = {(int)floorf(quant_factor * min.x) & 0xfffe, (int)floorf(quant_factor * min.y) & 0xfffe, (int)floorf(quant_factor * min.z) & 0xfffe};
    int bmax[3] = {(int)floorf(quant_factor * max.x + 1) | 1, (int)floorf(quant_factor * max.y + 1) | 1, (int)floorf(quant_factor * max.z + 1) | 1};

    uint32_t node_index = 0;
    while (node_index < nodes.length) {
        const NavMeshBvNode &bv_node = nodes[node_index];

        bool overlap = bmin[0] <= bv_node.bmax[0] && bmax[0] >= bv_node.bmin[0] && bmin[1] <= bv_node.bmax[1] && bmax[1] >= bv_node.bmin[1] &&
                       bmin[2] <= bv_node.bmax[2] && bmax[2] >= bv_node.bmin[2];

        bool is_leaf_node = bv_node.i >= 0;

        if (is_leaf_node && overlap) {
            NodeRef ref = get_node_by_tile_and_poly(nav_mesh, tile, bv_node.i).ref;
            if (passes_filter(filter, ref, nav_mesh)) visit(tile, bv_node.i, ref, user_data);
        }

        if (overlap || is_leaf_node) {
            node_index++;
        } else {
            node_index += (uint32_t)(-bv_node.i);
        }
    }
}

static void visit_polygons(const NavMesh &nav_mesh, Box3 bounds, const QueryFilter &filter, PolyVisitor visit, void *user_data) {
    Vec2 min_tile = world_to_tile_position(nav_mesh, bounds.min);
    Vec2 max_tile = world_to_tile_position(nav_mesh, bounds.max);

    if (!vec2::finite(min_tile) || !vec2::finite(max_tile)) return;

    for (int x = (int)min_tile.x; x <= (int)max_tile.x; x++) {
        for (int y = (int)min_tile.y; y <= (int)max_tile.y; y++) {
            const Array<uint32_t> *column = tile_column(nav_mesh, x, y);
            if (!column) continue;
            for (uint32_t tile_id : *column) visit_polygons_in_tile(nav_mesh, nav_mesh.tiles[tile_id], bounds, filter, visit, user_data);
        }
    }
}

struct NearestPolySearch {
    const NavMesh *nav_mesh;
    Vec3 center;
    float nearest_distance_sqr;
    FindNearestPolyResult *out;
};

static const float NEAREST_POLY_BOUNDS_SLACK = 1e-3f;

static float squared_distance_to_poly_bounds_2d(Vec3 point, const float *vertices, uint32_t vertex_count) {
    float min_x = vertices[0];
    float max_x = vertices[0];
    float min_z = vertices[2];
    float max_z = vertices[2];
    for (uint32_t i = 1; i < vertex_count; i++) {
        min_x = scalar::min(min_x, vertices[i * 3]);
        max_x = scalar::max(max_x, vertices[i * 3]);
        min_z = scalar::min(min_z, vertices[i * 3 + 2]);
        max_z = scalar::max(max_z, vertices[i * 3 + 2]);
    }
    float dx = scalar::max(scalar::max(min_x - NEAREST_POLY_BOUNDS_SLACK - point.x, point.x - max_x - NEAREST_POLY_BOUNDS_SLACK), 0.0f);
    float dz = scalar::max(scalar::max(min_z - NEAREST_POLY_BOUNDS_SLACK - point.z, point.z - max_z - NEAREST_POLY_BOUNDS_SLACK), 0.0f);
    return dx * dx + dz * dz;
}

static void visit_nearest_poly(const NavMeshTile &tile, int poly_index, NodeRef ref, void *user_data) {
    NearestPolySearch &search = *(NearestPolySearch *)user_data;
    const NavMeshPoly &poly = tile.polys[(uint32_t)poly_index];

    PolyVertices vertices;
    uint32_t vertex_count = gather_poly_vertices(tile, poly, vertices);
    float lower_bound = squared_distance_to_poly_bounds_2d(search.center, vertices.data, vertex_count);
    bool over_poly = lower_bound == 0 && geometry::point_in_poly(search.center, vertices.data, (int)vertex_count);
    release(vertices);
    if (!over_poly && lower_bound >= search.nearest_distance_sqr) return;

    float distance_sqr;
    Vec3 closest;
    if (over_poly) {
        float height = get_detail_mesh_height(tile, poly, poly_index, search.center);
        closest = {search.center.x, height, search.center.z};
        float height_difference = fabsf(search.center.y - height) - tile.walkable_climb;
        distance_sqr = height_difference > 0 ? height_difference * height_difference : 0;
    } else {
        GetClosestPointOnDetailEdgesResult edge;
        get_closest_point_on_detail_edges(edge, tile, poly, poly_index, search.center, true);
        closest = edge.position;
        distance_sqr = vec3::squared_length(vec3::subtract(search.center, closest));
    }

    if (distance_sqr < search.nearest_distance_sqr) {
        search.nearest_distance_sqr = distance_sqr;
        search.out->node_ref = ref;
        search.out->position = closest;
        search.out->success = true;
    }
}

void find_nearest_poly(FindNearestPolyResult &out, const NavMesh &nav_mesh, Vec3 center, Vec3 half_extents, const QueryFilter &filter) {
    out = {false, INVALID_NODE_REF, center};

    Box3 bounds = {vec3::subtract(center, half_extents), vec3::add(center, half_extents)};
    NearestPolySearch search = {&nav_mesh, center, FLT_MAX, &out};
    visit_polygons(nav_mesh, bounds, filter, visit_nearest_poly, &search);
}

struct PolyWriter {
    NodeRef *polys;
    uint32_t capacity;
    uint32_t count;
};

static void write_poly(const NavMeshTile &, int, NodeRef ref, void *user_data) {
    PolyWriter &writer = *(PolyWriter *)user_data;
    if (writer.count < writer.capacity) writer.polys[writer.count] = ref;
    writer.count++;
}

static void finish_query(QueryPolygonsResult &out, const PolyWriter &writer) {
    out.poly_total = writer.count;
    out.poly_count = writer.count < writer.capacity ? writer.count : writer.capacity;
    out.truncated = writer.count > writer.capacity;
}

void query_polygons_in_tile(QueryPolygonsResult &out, const NavMesh &nav_mesh, const NavMeshTile &tile, Box3 bounds, const QueryFilter &filter,
                            Span<NodeRef> polys) {
    PolyWriter writer = {polys.data, polys.capacity, 0};
    visit_polygons_in_tile(nav_mesh, tile, bounds, filter, write_poly, &writer);
    finish_query(out, writer);
}

void query_polygons(QueryPolygonsResult &out, const NavMesh &nav_mesh, Box3 bounds, const QueryFilter &filter, Span<NodeRef> polys) {
    PolyWriter writer = {polys.data, polys.capacity, 0};
    visit_polygons(nav_mesh, bounds, filter, write_poly, &writer);
    finish_query(out, writer);
}

uint32_t get_max_node_count(const NavMesh &nav_mesh) { return nav_mesh.nodes.length; }

static uint32_t allocate_node(NavMesh &nav_mesh) {
    uint32_t node_index = index_pool::request(nav_mesh.node_index_pool);
    if (node_index == nav_mesh.nodes.length) nav_mesh.nodes.push({});

    NavMeshNode &node = nav_mesh.nodes[node_index];
    node = {};
    node.allocated = true;
    node.first_link = NULL_LINK;
    node.tile_id = -1;
    node.poly_index = -1;
    node.off_mesh_connection_id = -1;
    return node_index;
}

static void release_node(NavMesh &nav_mesh, uint32_t index) {
    NavMeshNode &node = nav_mesh.nodes[index];
    node = {};
    node.first_link = NULL_LINK;
    node.tile_id = -1;
    node.poly_index = -1;
    node.off_mesh_connection_id = -1;
    index_pool::release_index(nav_mesh.node_index_pool, index);
}

static uint32_t allocate_link(NavMesh &nav_mesh) {
    uint32_t link_index = index_pool::request(nav_mesh.link_index_pool);
    if (link_index == nav_mesh.links.length) nav_mesh.links.push({});

    NavMeshLink &link = nav_mesh.links[link_index];
    link = {};
    link.next = NULL_LINK;
    link.to_node_ref = INVALID_NODE_REF;
    return link_index;
}

static void release_link(NavMesh &nav_mesh, uint32_t index) {
    NavMeshLink &link = nav_mesh.links[index];
    link = {};
    link.next = NULL_LINK;
    link.to_node_ref = INVALID_NODE_REF;
    index_pool::release_index(nav_mesh.link_index_pool, index);
}

static void append_link(NavMesh &nav_mesh, uint32_t node_index, uint32_t link_index) {
    nav_mesh.links[link_index].next = NULL_LINK;
    int32_t *tail = &nav_mesh.nodes[node_index].first_link;
    while (*tail != NULL_LINK) tail = &nav_mesh.links[(uint32_t)*tail].next;
    *tail = (int32_t)link_index;
}

template <typename ShouldRelease>
static void release_links_where(NavMesh &nav_mesh, uint32_t node_index, ShouldRelease should_release) {
    int32_t *tail = &nav_mesh.nodes[node_index].first_link;
    while (*tail != NULL_LINK) {
        uint32_t link_index = (uint32_t)*tail;
        if (should_release(nav_mesh.links[link_index])) {
            *tail = nav_mesh.links[link_index].next;
            release_link(nav_mesh, link_index);
        } else {
            tail = &nav_mesh.links[link_index].next;
        }
    }
}

static void add_link(NavMesh &nav_mesh, uint32_t from_node_index, NodeRef to_node_ref, uint8_t edge, uint8_t side, uint8_t bmin, uint8_t bmax) {
    uint32_t link_index = allocate_link(nav_mesh);
    NavMeshLink &link = nav_mesh.links[link_index];
    link.to_node_ref = to_node_ref;
    link.edge = edge;
    link.side = side;
    link.bmin = bmin;
    link.bmax = bmax;
    append_link(nav_mesh, from_node_index, link_index);
}

static void connect_internal_links(NavMesh &nav_mesh, const NavMeshTile &tile) {
    for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
        const NavMeshPoly &poly = tile.polys[poly_index];
        uint32_t node_index = tile.poly_nodes[poly_index];

        for (uint32_t edge_index = 0; edge_index < poly.vertex_count; edge_index++) {
            int nei = tile.poly_neis[poly.vertex_base + edge_index];
            if (nei == 0 || nei & POLY_NEIS_FLAG_EXT_LINK) continue;

            int neighbour_poly_index = nei - 1;
            if (neighbour_poly_index < 0 || (uint32_t)neighbour_poly_index >= tile.polys.length) continue;

            uint32_t neighbour_node_index = tile.poly_nodes[(uint32_t)neighbour_poly_index];
            add_link(nav_mesh, node_index, nav_mesh.nodes[neighbour_node_index].ref, (uint8_t)edge_index, 0xff, 0, 0);
        }
    }
}

static int opposite_tile(int side) { return (side + 4) & 0x7; }

static float get_slab_coord(Vec3 v, int side) {
    if (side == 0 || side == 4) return v.x;
    if (side == 2 || side == 6) return v.z;
    return 0;
}

static void calc_slab_end_points(Vec3 va, Vec3 vb, Vec2 &bmin, Vec2 &bmax, int side) {
    if (side == 0 || side == 4) {
        if (va.z < vb.z) {
            bmin = {va.z, va.y};
            bmax = {vb.z, vb.y};
        } else {
            bmin = {vb.z, vb.y};
            bmax = {va.z, va.y};
        }
    } else if (side == 2 || side == 6) {
        if (va.x < vb.x) {
            bmin = {va.x, va.y};
            bmax = {vb.x, vb.y};
        } else {
            bmin = {vb.x, vb.y};
            bmax = {va.x, va.y};
        }
    }
}

static bool overlap_slabs(Vec2 amin, Vec2 amax, Vec2 bmin, Vec2 bmax, float px, float py) {
    float minx = fmaxf(amin.x + px, bmin.x + px);
    float maxx = fminf(amax.x - px, bmax.x - px);
    if (minx > maxx) return false;

    float ad = (amax.y - amin.y) / (amax.x - amin.x);
    float ak = amin.y - ad * amin.x;
    float bd = (bmax.y - bmin.y) / (bmax.x - bmin.x);
    float bk = bmin.y - bd * bmin.x;
    float aminy = ad * minx + ak;
    float amaxy = ad * maxx + ak;
    float bminy = bd * minx + bk;
    float bmaxy = bd * maxx + bk;
    float dmin = bminy - aminy;
    float dmax = bmaxy - amaxy;
    if (dmin * dmax < 0) return true;
    float threshold = py * 2 * (py * 2);
    return dmin * dmin <= threshold || dmax * dmax <= threshold;
}

struct ConnectingPoly {
    NodeRef node_ref;
    float umin;
    float umax;
};

static void build_tile_portals(NavMeshTile &tile) {
    uint32_t counts[8] = {};
    for (const NavMeshPoly &poly : tile.polys) {
        for (uint32_t j = 0; j < poly.vertex_count; j++) {
            int nei = tile.poly_neis[poly.vertex_base + j];
            if (nei & POLY_NEIS_FLAG_EXT_LINK) counts[nei & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK]++;
        }
    }
    tile.portal_starts[0] = 0;
    for (int direction = 0; direction < 8; direction++) tile.portal_starts[direction + 1] = tile.portal_starts[direction] + counts[direction];

    tile.portals.resize(tile.portal_starts[8]);
    uint32_t next[8];
    for (int direction = 0; direction < 8; direction++) next[direction] = tile.portal_starts[direction];
    for (uint32_t i = 0; i < tile.polys.length; i++) {
        const NavMeshPoly &poly = tile.polys[i];
        uint32_t vertex_count = poly.vertex_count;
        for (uint32_t j = 0; j < vertex_count; j++) {
            int nei = tile.poly_neis[poly.vertex_base + j];
            if ((nei & POLY_NEIS_FLAG_EXT_LINK) == 0) continue;
            int direction = nei & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK;
            NavMeshTilePortal &portal = tile.portals[next[direction]++];
            portal.poly_index = i;
            portal.edge = (uint8_t)j;
            portal.start = tile_poly_vertex(tile, poly, j);
            portal.end = tile_poly_vertex(tile, poly, (j + 1) % vertex_count);
            portal.position = get_slab_coord(portal.start, direction);
            portal.bmin = {0, 0};
            portal.bmax = {0, 0};
            calc_slab_end_points(portal.start, portal.end, portal.bmin, portal.bmax, direction);
        }
    }
}

static void find_connecting_polys(Array<ConnectingPoly> &results, const NavMesh &nav_mesh, const NavMeshTilePortal &source, const NavMeshTile &target,
                                  int side) {
    results.clear();

    Vec2 amin = source.bmin;
    Vec2 amax = source.bmax;
    float apos = source.position;

    uint32_t connected_poly = UINT32_MAX;
    for (uint32_t k = target.portal_starts[side]; k < target.portal_starts[side + 1]; k++) {
        const NavMeshTilePortal &portal = target.portals[k];
        if (portal.poly_index == connected_poly) continue;
        if (fabsf(apos - portal.position) > 0.01f) continue;
        if (!overlap_slabs(amin, amax, portal.bmin, portal.bmax, 0.01f, target.walkable_climb)) continue;

        results.push({get_node_by_tile_and_poly(nav_mesh, target, (int)portal.poly_index).ref, fmaxf(amin.x, portal.bmin.x), fminf(amax.x, portal.bmax.x)});
        connected_poly = portal.poly_index;
    }
}

static uint8_t portal_fraction(float t) { return (uint8_t)floorf(t * 255 + 0.5f); }

static void connect_portal(NavMesh &nav_mesh, Array<ConnectingPoly> &overlaps, const NavMeshTile &tile, const NavMeshTilePortal &portal,
                           const NavMeshTile &target, int direction) {
    find_connecting_polys(overlaps, nav_mesh, portal, target, opposite_tile(direction));

    Vec3 va = portal.start;
    Vec3 vb = portal.end;
    uint32_t node_index = tile.poly_nodes[portal.poly_index];
    for (const ConnectingPoly &overlap : overlaps) {
        float tmin;
        float tmax;
        if (direction == 0 || direction == 4) {
            tmin = (overlap.umin - va.z) / (vb.z - va.z);
            tmax = (overlap.umax - va.z) / (vb.z - va.z);
        } else {
            tmin = (overlap.umin - va.x) / (vb.x - va.x);
            tmax = (overlap.umax - va.x) / (vb.x - va.x);
        }

        if (tmin > tmax) {
            float swap = tmin;
            tmin = tmax;
            tmax = swap;
        }

        tmin = fmaxf(0, fminf(1, tmin));
        tmax = fmaxf(0, fminf(1, tmax));

        add_link(nav_mesh, node_index, overlap.node_ref, portal.edge, (uint8_t)direction, portal_fraction(tmin), portal_fraction(tmax));
    }
}

static void connect_external_links(NavMesh &nav_mesh, const NavMeshTile &tile, const NavMeshTile &target, int side) {
    Array<ConnectingPoly> overlaps = {};

    if (side != -1) {
        for (uint32_t k = tile.portal_starts[side]; k < tile.portal_starts[side + 1]; k++) connect_portal(nav_mesh, overlaps, tile, tile.portals[k], target, side);
    } else {
        uint32_t next[8];
        for (int direction = 0; direction < 8; direction++) next[direction] = tile.portal_starts[direction];
        for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
            const NavMeshPoly &poly = tile.polys[poly_index];
            for (uint32_t j = 0; j < poly.vertex_count; j++) {
                int nei = tile.poly_neis[poly.vertex_base + j];
                if ((nei & POLY_NEIS_FLAG_EXT_LINK) == 0) continue;
                int direction = nei & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK;
                connect_portal(nav_mesh, overlaps, tile, tile.portals[next[direction]++], target, direction);
            }
        }
    }

    overlaps.release();
}

static void disconnect_external_links(NavMesh &nav_mesh, const NavMeshTile &tile, const NavMeshTile &target) {
    int32_t target_id = (int32_t)target.id;
    for (uint32_t poly_index = 0; poly_index < tile.polys.length; poly_index++) {
        release_links_where(nav_mesh, tile.poly_nodes[poly_index], [&](const NavMeshLink &link) {
            return link.side != 0xff && get_node_by_ref(nav_mesh, link.to_node_ref).tile_id == target_id;
        });
    }
}

static bool connect_off_mesh_connection(NavMesh &nav_mesh, uint32_t id) {
    OffMeshConnection connection = nav_mesh.off_mesh_connections[id];
    Vec3 half_extents = {connection.radius, connection.radius, connection.radius};

    FindNearestPolyResult start = {};
    find_nearest_poly(start, nav_mesh, connection.start, half_extents, DEFAULT_QUERY_FILTER);
    FindNearestPolyResult end = {};
    find_nearest_poly(end, nav_mesh, connection.end, half_extents, DEFAULT_QUERY_FILTER);

    if (!start.success || !end.success) return false;

    uint32_t start_node_index = node::get_node_ref_index(start.node_ref);
    uint32_t end_node_index = node::get_node_ref_index(end.node_ref);

    if (nav_mesh.off_mesh_connection_attachments.length <= id) nav_mesh.off_mesh_connection_attachments.resize(id + 1, {});

    uint32_t off_mesh_node_index = allocate_node(nav_mesh);
    NodeRef off_mesh_node_ref = node::ser_node_ref(NODE_TYPE_OFFMESH, off_mesh_node_index, connection.sequence);
    NavMeshNode &off_mesh_node = nav_mesh.nodes[off_mesh_node_index];
    off_mesh_node.type = NODE_TYPE_OFFMESH;
    off_mesh_node.ref = off_mesh_node_ref;
    off_mesh_node.area = connection.area;
    off_mesh_node.flags = connection.flags;
    off_mesh_node.off_mesh_connection_id = (int32_t)id;

    nav_mesh.off_mesh_connection_attachments[id] = {true, off_mesh_node_ref, start.node_ref, end.node_ref};

    add_link(nav_mesh, start_node_index, off_mesh_node_ref, 0, 0, 0, 0);
    add_link(nav_mesh, off_mesh_node_index, end.node_ref, 1, 0, 0, 0);

    if (connection.direction == OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL) {
        add_link(nav_mesh, end_node_index, off_mesh_node_ref, 1, 0, 0, 0);
        add_link(nav_mesh, off_mesh_node_index, start.node_ref, 0, 0, 0, 0);
    }

    return true;
}

static bool disconnect_off_mesh_connection(NavMesh &nav_mesh, uint32_t id) {
    const OffMeshConnectionAttachment *attachment = attachment_by_id(nav_mesh, id);
    if (!attachment) return false;

    NodeRef off_mesh_node_ref = attachment->off_mesh_node;
    NodeRef start_poly_node = attachment->start_poly_node;
    NodeRef end_poly_node = attachment->end_poly_node;

    auto links_to_off_mesh_node = [&](const NavMeshLink &link) { return link.to_node_ref == off_mesh_node_ref; };

    uint32_t start_node_index = node::get_node_ref_index(start_poly_node);
    if (start_node_index < nav_mesh.nodes.length) release_links_where(nav_mesh, start_node_index, links_to_off_mesh_node);

    uint32_t end_node_index = node::get_node_ref_index(end_poly_node);
    if (end_node_index < nav_mesh.nodes.length) release_links_where(nav_mesh, end_node_index, links_to_off_mesh_node);

    uint32_t off_mesh_node_index = node::get_node_ref_index(off_mesh_node_ref);
    if (off_mesh_node_index < nav_mesh.nodes.length) {
        release_links_where(nav_mesh, off_mesh_node_index, [](const NavMeshLink &) { return true; });
    }

    release_node(nav_mesh, off_mesh_node_index);

    nav_mesh.off_mesh_connection_attachments[id] = {};

    return true;
}

bool reconnect_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id) {
    disconnect_off_mesh_connection(nav_mesh, off_mesh_connection_id);
    return connect_off_mesh_connection(nav_mesh, off_mesh_connection_id);
}

static void update_off_mesh_connections(NavMesh &nav_mesh) {
    for (uint32_t id = 0; id < nav_mesh.off_mesh_connections.length; id++) {
        if (!nav_mesh.off_mesh_connections[id].allocated) continue;
        if (!is_off_mesh_connection_connected(nav_mesh, id)) reconnect_off_mesh_connection(nav_mesh, id);
    }
}

template <typename T>
static Array<T> copy_array(const Array<T> &source) {
    Array<T> copy = {};
    copy.resize(source.length);
    for (uint32_t i = 0; i < source.length; i++) copy[i] = source[i];
    return copy;
}

NavMeshTile build_tile(const NavMeshTileParams &params) {
    NavMeshTile tile = {};
    tile.tile_x = params.tile_x;
    tile.tile_y = params.tile_y;
    tile.tile_layer = params.tile_layer;
    tile.bounds = params.bounds;
    tile.vertices = copy_array(params.vertices);
    tile.polys = copy_array(params.polys);
    for (const NavMeshPoly &poly : tile.polys) ROAM_ASSERT(poly.vertex_count <= MAX_POLY_VERTICES);
    tile.poly_vertices = copy_array(params.poly_vertices);
    tile.poly_neis = copy_array(params.poly_neis);
    tile.detail_meshes = copy_array(params.detail_meshes);
    tile.detail_vertices = copy_array(params.detail_vertices);
    tile.detail_triangles = copy_array(params.detail_triangles);
    tile.bv_tree = bv_tree::build_nav_mesh_bv_tree(params);
    tile.cell_size = params.cell_size;
    tile.cell_height = params.cell_height;
    tile.walkable_height = params.walkable_height;
    tile.walkable_radius = params.walkable_radius;
    tile.walkable_climb = params.walkable_climb;
    return tile;
}

static void connect_with_column(NavMesh &nav_mesh, uint32_t tile_id) {
    const NavMeshTile &tile = nav_mesh.tiles[tile_id];
    const Array<uint32_t> *column = tile_column(nav_mesh, tile.tile_x, tile.tile_y);
    if (!column) return;
    for (uint32_t other_id : *column) {
        if (other_id == tile_id) continue;
        connect_external_links(nav_mesh, nav_mesh.tiles[other_id], tile, -1);
        connect_external_links(nav_mesh, tile, nav_mesh.tiles[other_id], -1);
    }
}

uint32_t add_tile(NavMesh &nav_mesh, NavMeshTile tile) {
    uint64_t position_key = tile_position_key(tile.tile_x, tile.tile_y, tile.tile_layer);

    if (nav_mesh.tile_position_to_tile_id.has(position_key)) remove_tile(nav_mesh, tile.tile_x, tile.tile_y, tile.tile_layer);

    uint32_t *sequence_counter = nav_mesh.tile_position_to_sequence_counter.get(position_key);
    uint32_t sequence = sequence_counter ? (*sequence_counter + 1) % node::MAX_SEQUENCE : 0;
    nav_mesh.tile_position_to_sequence_counter.set(position_key, sequence);

    uint32_t id = index_pool::request(nav_mesh.tile_index_pool);
    if (id == nav_mesh.tiles.length) nav_mesh.tiles.push({});

    tile.allocated = true;
    tile.id = id;
    tile.sequence = sequence;
    tile.poly_nodes.clear();
    tile.poly_nodes.reserve(tile.polys.length);
    tile.portals = {};
    build_tile_portals(tile);
    nav_mesh.tiles[id] = tile;

    nav_mesh.tile_position_to_tile_id.set(position_key, id);

    uint64_t column_key = tile_column_key(tile.tile_x, tile.tile_y);
    Array<uint32_t> *column = nav_mesh.tile_column_to_tile_ids.get(column_key);
    if (!column) column = &nav_mesh.tile_column_to_tile_ids.set(column_key, {});
    column->push(id);

    for (uint32_t i = 0; i < nav_mesh.tiles[id].polys.length; i++) {
        uint32_t node_index = allocate_node(nav_mesh);
        NavMeshNode &node = nav_mesh.nodes[node_index];
        const NavMeshPoly &poly = nav_mesh.tiles[id].polys[i];
        node.ref = node::ser_node_ref(NODE_TYPE_POLY, node_index, sequence);
        node.type = NODE_TYPE_POLY;
        node.area = poly.area;
        node.flags = poly.flags;
        node.tile_id = (int32_t)id;
        node.poly_index = (int32_t)i;
        nav_mesh.tiles[id].poly_nodes.push(node_index);
    }

    connect_internal_links(nav_mesh, nav_mesh.tiles[id]);

    connect_with_column(nav_mesh, id);

    for (int side = 0; side < 8; side++) {
        const Array<uint32_t> *neighbours = neighbour_tile_column(nav_mesh, tile.tile_x, tile.tile_y, side);
        if (!neighbours) continue;
        for (uint32_t neighbour_id : *neighbours) {
            connect_external_links(nav_mesh, nav_mesh.tiles[id], nav_mesh.tiles[neighbour_id], side);
            connect_external_links(nav_mesh, nav_mesh.tiles[neighbour_id], nav_mesh.tiles[id], opposite_tile(side));
        }
    }

    update_off_mesh_connections(nav_mesh);

    return id;
}

bool remove_tile(NavMesh &nav_mesh, int x, int y, int layer) {
    uint64_t position_key = tile_position_key(x, y, layer);
    const uint32_t *found = nav_mesh.tile_position_to_tile_id.get(position_key);
    if (!found) return false;
    uint32_t tile_id = *found;
    if (!tile_by_id(nav_mesh, (int32_t)tile_id)) return false;

    NavMeshTile &tile = nav_mesh.tiles[tile_id];

    const Array<uint32_t> *column = tile_column(nav_mesh, x, y);
    if (column) {
        for (uint32_t other_id : *column) {
            if (other_id == tile_id) continue;
            disconnect_external_links(nav_mesh, nav_mesh.tiles[other_id], tile);
            disconnect_external_links(nav_mesh, tile, nav_mesh.tiles[other_id]);
        }
    }

    for (int side = 0; side < 8; side++) {
        const Array<uint32_t> *neighbours = neighbour_tile_column(nav_mesh, x, y, side);
        if (!neighbours) continue;
        for (uint32_t neighbour_id : *neighbours) {
            disconnect_external_links(nav_mesh, nav_mesh.tiles[neighbour_id], tile);
            disconnect_external_links(nav_mesh, tile, nav_mesh.tiles[neighbour_id]);
        }
    }

    for (uint32_t node_index : tile.poly_nodes) {
        release_links_where(nav_mesh, node_index, [](const NavMeshLink &) { return true; });
    }

    for (uint32_t node_index : tile.poly_nodes) release_node(nav_mesh, node_index);

    release_tile_data(tile);
    tile = {};

    nav_mesh.tile_position_to_tile_id.remove(position_key);

    uint64_t column_key = tile_column_key(x, y);
    Array<uint32_t> *tile_ids = nav_mesh.tile_column_to_tile_ids.get(column_key);
    if (tile_ids) {
        for (uint32_t i = 0; i < tile_ids->length; i++) {
            if ((*tile_ids)[i] != tile_id) continue;
            for (uint32_t j = i; j + 1 < tile_ids->length; j++) (*tile_ids)[j] = (*tile_ids)[j + 1];
            tile_ids->pop();
            break;
        }
        if (tile_ids->length == 0) {
            tile_ids->release();
            nav_mesh.tile_column_to_tile_ids.remove(column_key);
        }
    }

    index_pool::release_index(nav_mesh.tile_index_pool, tile_id);

    update_off_mesh_connections(nav_mesh);

    return true;
}

uint32_t add_off_mesh_connection(NavMesh &nav_mesh, const OffMeshConnectionParams &params) {
    uint32_t id = index_pool::request(nav_mesh.off_mesh_connection_index_pool);

    uint32_t sequence = nav_mesh.off_mesh_connection_sequence_counter;
    nav_mesh.off_mesh_connection_sequence_counter = (nav_mesh.off_mesh_connection_sequence_counter + 1) % node::MAX_SEQUENCE;

    if (id == nav_mesh.off_mesh_connections.length) nav_mesh.off_mesh_connections.push({});

    nav_mesh.off_mesh_connections[id] = {true, id, sequence, params.start, params.end, params.radius, params.direction, params.flags, params.area};

    connect_off_mesh_connection(nav_mesh, id);

    return id;
}

void remove_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id) {
    if (!off_mesh_connection_by_id(nav_mesh, (int32_t)off_mesh_connection_id)) return;

    index_pool::release_index(nav_mesh.off_mesh_connection_index_pool, off_mesh_connection_id);

    disconnect_off_mesh_connection(nav_mesh, off_mesh_connection_id);

    nav_mesh.off_mesh_connections[off_mesh_connection_id] = {};
}

bool is_off_mesh_connection_connected(const NavMesh &nav_mesh, uint32_t off_mesh_connection_id) {
    const OffMeshConnectionAttachment *attachment = attachment_by_id(nav_mesh, off_mesh_connection_id);
    if (!attachment) return false;
    return is_valid_node_ref(nav_mesh, attachment->start_poly_node) && is_valid_node_ref(nav_mesh, attachment->end_poly_node);
}

}

}
