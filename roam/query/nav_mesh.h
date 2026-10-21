#pragma once

#include <stdint.h>

#include "roam/math/box3.h"
#include "roam/math/vec2.h"
#include "roam/math/vec3.h"
#include "roam/query/node.h"
#include "roam/util/array.h"
#include "roam/util/index_pool.h"
#include "roam/util/map.h"
#include "roam/util/span.h"

namespace roam {

constexpr int32_t NULL_LINK = -1;
constexpr uint32_t MAX_POLY_VERTICES = 0xff;

struct NavMeshPoly {
    uint32_t vertex_base;
    uint32_t vertex_count;
    uint32_t flags;
    uint8_t area;
};

struct NavMeshPolyDetail {
    uint32_t vertices_base;
    uint32_t triangles_base;
    uint32_t vertices_count;
    uint32_t triangles_count;
};

struct NavMeshNode {
    NodeRef ref;
    int32_t first_link;
    uint32_t flags;
    int32_t tile_id;
    int32_t poly_index;
    int32_t off_mesh_connection_id;
    uint8_t area;
    NodeType type;
    bool allocated;
};

static_assert(sizeof(NavMeshNode) == 32);

struct NavMeshLink {
    NodeRef to_node_ref;
    int32_t next;
    uint8_t edge;
    uint8_t side;
    uint8_t bmin;
    uint8_t bmax;
};

static_assert(sizeof(NavMeshLink) == 16);

enum OffMeshConnectionDirection : uint32_t {
    OFF_MESH_CONNECTION_DIRECTION_START_TO_END = 0,
    OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL = 1,
};

struct OffMeshConnectionParams {
    Vec3 start;
    Vec3 end;
    float radius;
    OffMeshConnectionDirection direction;
    uint32_t flags;
    uint8_t area;
};

struct OffMeshConnection {
    bool allocated;
    uint32_t id;
    uint32_t sequence;
    Vec3 start;
    Vec3 end;
    float radius;
    OffMeshConnectionDirection direction;
    uint32_t flags;
    uint8_t area;
};

struct OffMeshConnectionAttachment {
    bool attached;
    NodeRef off_mesh_node;
    NodeRef start_poly_node;
    NodeRef end_poly_node;
};

struct NavMeshBvNode {
    uint16_t bmin[3];
    uint16_t bmax[3];
    int i;
};

struct NavMeshTileBvTree {
    Array<NavMeshBvNode> nodes;
    float quant_factor;
};

struct NavMeshTileParams {
    Box3 bounds;
    Span<const float> vertices;
    Span<const NavMeshPoly> polys;
    Span<const uint32_t> poly_vertices;
    Span<const int> poly_neis;
    Span<const NavMeshPolyDetail> detail_meshes;
    Span<const float> detail_vertices;
    Span<const uint8_t> detail_triangles;
    int tile_x;
    int tile_y;
    int tile_layer;
    float cell_size;
    float cell_height;
    float walkable_height;
    float walkable_radius;
    float walkable_climb;
};

struct NavMeshTilePortal {
    uint32_t poly_index;
    uint8_t edge;
    Vec3 start;
    Vec3 end;
    float position;
    Vec2 bmin;
    Vec2 bmax;
};

struct NavMeshTile {
    bool allocated;
    uint32_t id;
    uint32_t sequence;
    int tile_x;
    int tile_y;
    int tile_layer;
    Box3 bounds;
    Array<float> vertices;
    Array<NavMeshPoly> polys;
    Array<uint32_t> poly_vertices;
    Array<int> poly_neis;
    Array<NavMeshPolyDetail> detail_meshes;
    Array<float> detail_vertices;
    Array<uint8_t> detail_triangles;
    Array<uint32_t> poly_nodes;
    Array<NavMeshTilePortal> portals;
    uint32_t portal_starts[9];
    NavMeshTileBvTree bv_tree;
    float cell_size;
    float cell_height;
    float walkable_height;
    float walkable_radius;
    float walkable_climb;
};

struct NavMesh {
    Vec3 origin;
    float tile_width;
    float tile_height;
    Array<NavMeshNode> nodes;
    Array<NavMeshLink> links;
    Array<OffMeshConnection> off_mesh_connections;
    Array<OffMeshConnectionAttachment> off_mesh_connection_attachments;
    Array<NavMeshTile> tiles;
    Map<uint64_t, uint32_t> tile_position_to_tile_id;
    Map<uint64_t, Array<uint32_t>> tile_column_to_tile_ids;
    Map<uint64_t, uint32_t> tile_position_to_sequence_counter;
    uint32_t off_mesh_connection_sequence_counter;
    IndexPool node_index_pool;
    IndexPool tile_index_pool;
    IndexPool off_mesh_connection_index_pool;
    IndexPool link_index_pool;
};

}
