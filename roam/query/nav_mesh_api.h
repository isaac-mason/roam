#pragma once

#include <stdint.h>

#include "roam/math/box3.h"
#include "roam/math/vec2.h"
#include "roam/math/vec3.h"
#include "roam/query/nav_mesh.h"
#include "roam/query/node.h"

namespace roam {

struct QueryFilter {
    bool (*pass_filter)(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh);
    float (*get_cost)(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef previous_ref, NodeRef current_ref,
                      NodeRef next_ref);
    uint32_t include_flags;
    uint32_t exclude_flags;
    void *user_data;
};

struct TileAndPoly {
    bool success;
    const NavMeshTile *tile;
    const NavMeshPoly *poly;
    int poly_index;
};

constexpr uint32_t INLINE_POLY_VERTICES = 32;

struct PolyVertices {
    float inline_values[INLINE_POLY_VERTICES * 3];
    Array<float> overflow = {};
    float *data = nullptr;
};

struct GetPolyHeightResult {
    bool success;
    float height;
};

struct ClosestPointOnDetailEdges {
    Vec3 point;
    float distance_sqr;
};

struct GetClosestPointOnPolyResult {
    bool success;
    bool is_over_poly;
    Vec3 position;
};

struct ClosestPointOnPolyBoundary {
    bool success;
    Vec3 point;
};

struct QueryPolygonsResult {
    uint32_t poly_count;
    uint32_t full_poly_count;
};

struct FindNearestPolyResult {
    bool success;
    NodeRef node_ref;
    Vec3 position;
};

namespace nav_mesh_api {

extern const QueryFilter ANY_QUERY_FILTER;
extern const QueryFilter DEFAULT_QUERY_FILTER;

bool pass_flags(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh);
float distance_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef previous_ref, NodeRef current_ref, NodeRef next_ref);

inline bool passes_flags(const QueryFilter &filter, const NavMeshNode &node) {
    return (node.flags & filter.include_flags) != 0 && (node.flags & filter.exclude_flags) == 0;
}

inline bool passes_filter(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) {
    if (filter.pass_filter == pass_flags) return passes_flags(filter, nav_mesh.nodes[node::get_node_ref_index(node_ref)]);
    return filter.pass_filter(filter, node_ref, nav_mesh);
}

inline float filter_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef previous_ref, NodeRef current_ref,
                         NodeRef next_ref) {
    if (filter.get_cost == distance_cost) return vec3::distance(pa, pb);
    return filter.get_cost(filter, pa, pb, nav_mesh, previous_ref, current_ref, next_ref);
}

QueryFilter create_default_query_filter();

NavMesh create_nav_mesh();
void release_nav_mesh(NavMesh &nav_mesh);

const NavMeshNode &get_node_by_ref(const NavMesh &nav_mesh, NodeRef node_ref);
const NavMeshNode &get_node_by_tile_and_poly(const NavMesh &nav_mesh, const NavMeshTile &tile, int poly_index);
bool is_valid_node_ref(const NavMesh &nav_mesh, NodeRef node_ref);

const NavMeshTile *get_tile_at(const NavMesh &nav_mesh, int x, int y, int layer);
Array<const NavMeshTile *> get_tiles_at(const NavMesh &nav_mesh, int x, int y);
Vec2 world_to_tile_position(const NavMesh &nav_mesh, Vec3 world_position);

TileAndPoly get_tile_and_poly_by_ref(NodeRef ref, const NavMesh &nav_mesh);

uint32_t gather_poly_vertices(const NavMeshTile &tile, const NavMeshPoly &poly, PolyVertices &vertices);
void release(PolyVertices &vertices);

void get_poly_height(GetPolyHeightResult &out, const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position);
ClosestPointOnDetailEdges get_closest_point_on_detail_edges(const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position,
                                                            bool only_boundary);
void get_closest_point_on_poly(GetClosestPointOnPolyResult &out, const NavMesh &nav_mesh, NodeRef node_ref, Vec3 position);
void get_closest_point_on_poly_boundary(ClosestPointOnPolyBoundary &out, const NavMesh &nav_mesh, NodeRef node_ref, Vec3 point);
void find_nearest_poly(FindNearestPolyResult &out, const NavMesh &nav_mesh, Vec3 center, Vec3 half_extents, const QueryFilter &filter);

void query_polygons_in_tile(QueryPolygonsResult &out, const NavMesh &nav_mesh, const NavMeshTile &tile, Box3 bounds, const QueryFilter &filter,
                            NodeRef *polys, uint32_t max_polys);
void query_polygons(QueryPolygonsResult &out, const NavMesh &nav_mesh, Box3 bounds, const QueryFilter &filter, NodeRef *polys, uint32_t max_polys);

NavMeshTile build_tile(const NavMeshTileParams &params);
uint32_t add_tile(NavMesh &nav_mesh, NavMeshTile tile);
bool remove_tile(NavMesh &nav_mesh, int x, int y, int layer);

uint32_t add_off_mesh_connection(NavMesh &nav_mesh, const OffMeshConnectionParams &params);
void remove_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id);
bool reconnect_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id);
bool is_off_mesh_connection_connected(const NavMesh &nav_mesh, uint32_t off_mesh_connection_id);

}

}
