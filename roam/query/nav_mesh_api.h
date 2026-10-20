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

struct FindNearestPolyResult {
    bool success;
    NodeRef node_ref;
    Vec3 position;
};

namespace nav_mesh_api {

extern const QueryFilter ANY_QUERY_FILTER;
extern const QueryFilter DEFAULT_QUERY_FILTER;

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

GetPolyHeightResult get_poly_height(const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position);
ClosestPointOnDetailEdges get_closest_point_on_detail_edges(const NavMeshTile &tile, const NavMeshPoly &poly, int poly_index, Vec3 position,
                                                            bool only_boundary);
GetClosestPointOnPolyResult get_closest_point_on_poly(const NavMesh &nav_mesh, NodeRef node_ref, Vec3 position);
ClosestPointOnPolyBoundary get_closest_point_on_poly_boundary(const NavMesh &nav_mesh, NodeRef node_ref, Vec3 point);
FindNearestPolyResult find_nearest_poly(const NavMesh &nav_mesh, Vec3 center, Vec3 half_extents, const QueryFilter &filter);

void query_polygons_in_tile(Array<NodeRef> &out, const NavMesh &nav_mesh, const NavMeshTile &tile, Box3 bounds, const QueryFilter &filter);
Array<NodeRef> query_polygons(const NavMesh &nav_mesh, Box3 bounds, const QueryFilter &filter);

NavMeshTile build_tile(const NavMeshTileParams &params);
uint32_t add_tile(NavMesh &nav_mesh, NavMeshTile tile);
bool remove_tile(NavMesh &nav_mesh, int x, int y, int layer);

uint32_t add_off_mesh_connection(NavMesh &nav_mesh, const OffMeshConnectionParams &params);
void remove_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id);
bool reconnect_off_mesh_connection(NavMesh &nav_mesh, uint32_t off_mesh_connection_id);
bool is_off_mesh_connection_connected(const NavMesh &nav_mesh, uint32_t off_mesh_connection_id);

}

}
