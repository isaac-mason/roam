#pragma once

#include <math.h>

#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/util/sort.h"

using namespace roam;

inline roam::NavMesh create_nav_mesh_from_triangles(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count) {
    Box3 bounds = box3::empty();
    for (uint32_t i = 0; i < vertex_count; i++) bounds = box3::expand_by_point(bounds, vec3::load(positions, i * 3));

    Array<ExternalPolygon> polygons = {};
    for (uint32_t i = 0; i < index_count; i += 3) polygons.push({{&indices[i], 3}, 1, 0});

    NavMeshTilePolys tile_polys = nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polygons, {positions, vertex_count * 3}, 0, bounds);
    NavMeshTileDetailMesh detail_mesh = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

    NavMeshTileParams params = {};
    params.bounds = bounds;
    params.vertices = tile_polys.vertices;
    params.polys = tile_polys.polys;
    params.poly_vertices = tile_polys.poly_vertices;
    params.poly_neis = tile_polys.poly_neis;
    params.detail_meshes = detail_mesh.detail_meshes;
    params.detail_vertices = detail_mesh.detail_vertices;
    params.detail_triangles = detail_mesh.detail_triangles;
    params.cell_size = 0.2f;
    params.cell_height = 0.2f;
    params.walkable_height = 0.5f;
    params.walkable_radius = 0.5f;
    params.walkable_climb = 0.5f;

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = bounds.min;
    nav_mesh.tile_width = bounds.max.x - bounds.min.x;
    nav_mesh.tile_height = bounds.max.z - bounds.min.z;

    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(detail_mesh);
    polygons.release();
    return nav_mesh;
}

inline NavMesh create_tiled_terrain() {
    const int size = 24;
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    for (int z = 0; z <= size; z++) {
        for (int x = 0; x <= size; x++) {
            float height = 0.4f * sinf((float)x * 0.5f) * cosf((float)z * 0.4f);
            positions.push((float)x);
            positions.push(height);
            positions.push((float)z);
        }
    }
    for (int z = 0; z < size; z++) {
        for (int x = 0; x < size; x++) {
            if (x > 9 && x < 13 && z > 6 && z < 15) continue;
            uint32_t a = (uint32_t)(z * (size + 1) + x);
            uint32_t b = a + 1;
            uint32_t c = a + (uint32_t)(size + 1);
            uint32_t d = c + 1;
            const uint32_t quad[] = {a, c, b, b, c, d};
            for (uint32_t index : quad) indices.push(index);
        }
    }

    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    TiledNavMeshOptions options = {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .tile_size_voxels = 32,
        .tile_size_world = 32 * cell_size,
        .walkable_radius_voxels = 1,
        .walkable_radius_world = 0.2f,
        .walkable_climb_voxels = 3,
        .walkable_climb_world = 0.6f,
        .walkable_height_voxels = 5,
        .walkable_height_world = 1.0f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 4,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = cell_size * 6,
        .detail_sample_max_error = cell_height,
    };
    TiledNavMeshResult result = generate_tiled_nav_mesh::generate_tiled_nav_mesh({positions, indices}, options);
    generate_tiled_nav_mesh::release(result.intermediates);
    positions.release();
    indices.release();

    nav_mesh_api::add_off_mesh_connection(result.nav_mesh, {{2, 0, 2}, {20, 0, 20}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 1});
    return result.nav_mesh;
}

inline uint64_t poly_key(const NavMesh &nav_mesh, NodeRef ref) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, ref);
    if (node.type != NODE_TYPE_POLY) return 0xffffffffffffffffull;
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
    return ((uint64_t)(uint8_t)tile.tile_x << 56) | ((uint64_t)(uint8_t)tile.tile_y << 48) | ((uint64_t)(uint8_t)tile.tile_layer << 40) |
           (uint64_t)(uint32_t)node.poly_index;
}

struct LinkKey {
    uint64_t from;
    uint64_t to;
    uint32_t data;
};

inline Array<LinkKey> link_keys(const NavMesh &nav_mesh) {
    Array<LinkKey> keys = {};
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (!node.allocated || node.type != NODE_TYPE_POLY) continue;
        for (int32_t l = node.first_link; l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
            const NavMeshLink &link = nav_mesh.links[(uint32_t)l];
            uint32_t data = ((uint32_t)link.edge << 24) | ((uint32_t)link.side << 16) | ((uint32_t)link.bmin << 8) | link.bmax;
            keys.push({poly_key(nav_mesh, node.ref), poly_key(nav_mesh, link.to_node_ref), data});
        }
    }
    sort::stable(keys.data, keys.length, [](const LinkKey &a, const LinkKey &b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.to != b.to) return a.to < b.to;
        return a.data < b.data;
    });
    return keys;
}

inline bool same_links(const Array<LinkKey> &a, const Array<LinkKey> &b) {
    if (a.length != b.length) return false;
    for (uint32_t i = 0; i < a.length; i++) {
        if (a[i].from != b[i].from || a[i].to != b[i].to || a[i].data != b[i].data) return false;
    }
    return true;
}

inline const OffMeshConnectionPiece &first_piece(const NavMesh &nav_mesh, uint32_t id) {
    return nav_mesh.off_mesh_connection_pieces[(uint32_t)nav_mesh.off_mesh_connection_attachments[id].first_piece];
}

struct ClusterLinkKey {
    uint32_t from;
    uint32_t to;
    int32_t off_mesh_connection_id;
};

inline void sort_cluster_link_keys(Array<ClusterLinkKey> &keys) {
    sort::stable(keys.data, keys.length, [](const ClusterLinkKey &a, const ClusterLinkKey &b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.to != b.to) return a.to < b.to;
        return a.off_mesh_connection_id < b.off_mesh_connection_id;
    });
}

inline uint32_t node_cluster(const NavMesh &nav_mesh, NodeRef ref) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, ref);
    return nav_mesh.tiles[(uint32_t)node.tile_id].poly_clusters[(uint32_t)node.poly_index];
}

inline uint32_t internal_root(Array<uint32_t> &parents, uint32_t index) {
    while (parents[index] != index) index = parents[index];
    return index;
}

inline bool clusters_are_consistent(const NavMesh &nav_mesh) {
    Array<uint32_t> owner_count = {};
    owner_count.resize(nav_mesh.clusters.length, 0);
    bool ok = true;

    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        if (tile.poly_clusters.length != tile.polys.length) return false;
        for (uint32_t cluster : tile.clusters) {
            bool owned = cluster < nav_mesh.clusters.length && nav_mesh.clusters[cluster].allocated &&
                         nav_mesh.clusters[cluster].tile_id == (int32_t)tile.id;
            if (owned) owner_count[cluster]++;
            else ok = false;
        }

        Array<uint32_t> parents = {};
        parents.resize(tile.polys.length);
        for (uint32_t i = 0; i < tile.polys.length; i++) parents[i] = i;
        for (uint32_t p = 0; p < tile.polys.length; p++) {
            const NavMeshPoly &poly = tile.polys[p];
            for (uint32_t e = 0; e < poly.vertex_count; e++) {
                int nei = tile.poly_neis[poly.vertex_base + e];
                if (nei == 0 || nei & POLY_NEIS_FLAG_EXT_LINK || (uint32_t)(nei - 1) >= tile.polys.length) continue;
                uint32_t a = internal_root(parents, p);
                uint32_t b = internal_root(parents, (uint32_t)(nei - 1));
                if (a != b) parents[b] = a;
            }
        }
        for (uint32_t p = 0; p < tile.polys.length; p++) {
            for (uint32_t q = p + 1; q < tile.polys.length; q++) {
                bool connected = internal_root(parents, p) == internal_root(parents, q);
                if (connected != (tile.poly_clusters[p] == tile.poly_clusters[q])) ok = false;
            }
            bool listed = false;
            for (uint32_t cluster : tile.clusters) listed |= cluster == tile.poly_clusters[p];
            if (!listed) ok = false;
        }
        parents.release();
    }

    for (uint32_t i = 0; i < nav_mesh.clusters.length; i++) {
        if (nav_mesh.clusters[i].allocated != (owner_count[i] == 1)) ok = false;
    }

    Array<ClusterLinkKey> expected = {};
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        for (uint32_t p = 0; p < tile.polys.length; p++) {
            const NavMeshNode &node = nav_mesh.nodes[tile.poly_nodes[p]];
            for (int32_t l = node.first_link; l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
                const NavMeshLink &link = nav_mesh.links[(uint32_t)l];
                if (nav_mesh_api::get_node_by_ref(nav_mesh, link.to_node_ref).type != NODE_TYPE_POLY) continue;
                uint32_t to = node_cluster(nav_mesh, link.to_node_ref);
                if (to == tile.poly_clusters[p]) continue;
                bool seen = false;
                for (const ClusterLinkKey &key : expected) seen |= key.from == tile.poly_clusters[p] && key.to == to && key.off_mesh_connection_id < 0;
                if (!seen) expected.push({tile.poly_clusters[p], to, -1});
            }
        }
    }
    for (uint32_t id = 0; id < nav_mesh.off_mesh_connection_attachments.length; id++) {
        const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[id];
        if (!attachment.attached) continue;
        for (int32_t i = attachment.first_piece; i >= 0; i = nav_mesh.off_mesh_connection_pieces[(uint32_t)i].next) {
            const OffMeshConnectionPiece &piece = nav_mesh.off_mesh_connection_pieces[(uint32_t)i];
            uint32_t start = node_cluster(nav_mesh, piece.start_poly_node);
            uint32_t end = node_cluster(nav_mesh, piece.end_poly_node);
            if (start == end) continue;
            expected.push({start, end, (int32_t)id});
            bool bidirectional = nav_mesh.off_mesh_connections[id].direction == OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL;
            if (bidirectional) expected.push({end, start, (int32_t)id});
        }
    }

    Array<ClusterLinkKey> stored = {};
    for (uint32_t c = 0; c < nav_mesh.clusters.length; c++) {
        if (!nav_mesh.clusters[c].allocated) continue;
        for (int32_t l = nav_mesh.clusters[c].first_link; l != NULL_LINK; l = nav_mesh.cluster_links[(uint32_t)l].next) {
            const NavMeshClusterLink &link = nav_mesh.cluster_links[(uint32_t)l];
            if (link.to_cluster >= nav_mesh.clusters.length || !nav_mesh.clusters[link.to_cluster].allocated) ok = false;
            stored.push({c, link.to_cluster, link.off_mesh_connection_id});
        }
    }

    sort_cluster_link_keys(expected);
    sort_cluster_link_keys(stored);
    if (expected.length != stored.length) ok = false;
    for (uint32_t i = 0; ok && i < expected.length; i++) {
        const ClusterLinkKey &a = expected[i];
        const ClusterLinkKey &b = stored[i];
        if (a.from != b.from || a.to != b.to || a.off_mesh_connection_id != b.off_mesh_connection_id) ok = false;
    }

    expected.release();
    stored.release();
    owner_count.release();
    return ok;
}

inline bool reachable_by_links(const NavMesh &nav_mesh, NodeRef from, NodeRef to) {
    if (!nav_mesh_api::is_valid_node_ref(nav_mesh, from) || !nav_mesh_api::is_valid_node_ref(nav_mesh, to)) return false;
    Array<uint8_t> seen = {};
    seen.resize(nav_mesh.nodes.length, 0);
    Array<uint32_t> queue = {};
    queue.push(node::get_node_ref_index(from));
    seen[queue[0]] = 1;
    bool found = from == to;
    for (uint32_t head = 0; head < queue.length && !found; head++) {
        const NavMeshNode &node = nav_mesh.nodes[queue[head]];
        for (int32_t l = node.first_link; l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
            NodeRef next = nav_mesh.links[(uint32_t)l].to_node_ref;
            uint32_t index = node::get_node_ref_index(next);
            if (seen[index]) continue;
            if (next == to) found = true;
            seen[index] = 1;
            queue.push(index);
        }
    }
    seen.release();
    queue.release();
    return found;
}

inline bool segment_pieces_are_consistent(const NavMesh &nav_mesh) {
    for (const OffMeshConnection &connection : nav_mesh.off_mesh_connections) {
        if (!connection.allocated || connection.type != OFF_MESH_CONNECTION_TYPE_SEGMENT) continue;
        if (connection.id >= nav_mesh.off_mesh_connection_attachments.length) continue;
        const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[connection.id];
        if (!attachment.attached) continue;
        const OffMeshConnectionSegment &segment = connection.segment;
        Vec3 half_extents = {connection.radius, connection.radius, connection.radius};
        float previous_end = 0;
        for (int32_t i = attachment.first_piece; i >= 0; i = nav_mesh.off_mesh_connection_pieces[(uint32_t)i].next) {
            const OffMeshConnectionPiece &piece = nav_mesh.off_mesh_connection_pieces[(uint32_t)i];
            if (piece.t0 < previous_end - 1e-5f || piece.t1 <= piece.t0) return false;
            previous_end = piece.t1;
            float t = (piece.t0 + piece.t1) * 0.5f;
            FindNearestPolyResult start = {};
            nav_mesh_api::find_nearest_poly(start, nav_mesh, vec3::lerp(segment.start_a, segment.start_b, t), half_extents,
                                            nav_mesh_api::DEFAULT_QUERY_FILTER);
            FindNearestPolyResult end = {};
            nav_mesh_api::find_nearest_poly(end, nav_mesh, vec3::lerp(segment.end_a, segment.end_b, t), half_extents,
                                            nav_mesh_api::DEFAULT_QUERY_FILTER);
            if (!start.success || start.node_ref != piece.start_poly_node) return false;
            if (!end.success || end.node_ref != piece.end_poly_node) return false;
        }
    }
    return true;
}
