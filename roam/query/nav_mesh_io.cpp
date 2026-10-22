#include "roam/query/nav_mesh_io.h"

#include <string.h>

#include "roam/query/nav_mesh_api.h"
#include "roam/util/sort.h"

namespace roam {

namespace nav_mesh_io {

static const uint8_t TILE_MAGIC[4] = {'R', 'M', 'T', 'L'};
static const uint8_t NAV_MESH_MAGIC[4] = {'R', 'M', 'N', 'M'};

struct ByteWriter {
    uint8_t *data;
    uint32_t capacity;
    uint64_t size;
};

static void write_bytes(ByteWriter &writer, const void *source, uint32_t count) {
    if (writer.size + count <= writer.capacity) memcpy(writer.data + writer.size, source, count);
    writer.size += count;
}

static void write_u8(ByteWriter &writer, uint8_t value) { write_bytes(writer, &value, 1); }

static void write_u16(ByteWriter &writer, uint16_t value) {
    uint8_t bytes[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    write_bytes(writer, bytes, 2);
}

static void write_u32(ByteWriter &writer, uint32_t value) {
    uint8_t bytes[4] = {(uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    write_bytes(writer, bytes, 4);
}

static void write_i32(ByteWriter &writer, int32_t value) { write_u32(writer, (uint32_t)value); }

static void write_u64(ByteWriter &writer, uint64_t value) {
    write_u32(writer, (uint32_t)value);
    write_u32(writer, (uint32_t)(value >> 32));
}

static void write_f32(ByteWriter &writer, float value) {
    uint32_t bits;
    memcpy(&bits, &value, 4);
    write_u32(writer, bits);
}

static void write_vec3(ByteWriter &writer, Vec3 value) {
    write_f32(writer, value.x);
    write_f32(writer, value.y);
    write_f32(writer, value.z);
}

static void finish_save(SaveResult &out, const ByteWriter &writer) {
    bool fits = writer.size <= writer.capacity && writer.size <= UINT32_MAX;
    out.byte_total = writer.size <= UINT32_MAX ? (uint32_t)writer.size : UINT32_MAX;
    out.byte_count = fits ? out.byte_total : 0;
    out.truncated = !fits;
    out.success = fits;
}

struct ByteReader {
    const uint8_t *data;
    uint32_t size;
    uint32_t offset;
    bool failed;
};

static bool can_read(ByteReader &reader, uint64_t count, uint64_t element_size) {
    if (reader.failed) return false;
    if (count * element_size > (uint64_t)(reader.size - reader.offset)) {
        reader.failed = true;
        return false;
    }
    return true;
}

static void read_bytes(ByteReader &reader, void *destination, uint32_t count) {
    if (!can_read(reader, count, 1)) {
        memset(destination, 0, count);
        return;
    }
    memcpy(destination, reader.data + reader.offset, count);
    reader.offset += count;
}

static uint8_t read_u8(ByteReader &reader) {
    uint8_t value;
    read_bytes(reader, &value, 1);
    return value;
}

static uint16_t read_u16(ByteReader &reader) {
    uint8_t bytes[2];
    read_bytes(reader, bytes, 2);
    return (uint16_t)(bytes[0] | (bytes[1] << 8));
}

static uint32_t read_u32(ByteReader &reader) {
    uint8_t bytes[4];
    read_bytes(reader, bytes, 4);
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static int32_t read_i32(ByteReader &reader) { return (int32_t)read_u32(reader); }

static uint64_t read_u64(ByteReader &reader) {
    uint64_t low = read_u32(reader);
    uint64_t high = read_u32(reader);
    return low | (high << 32);
}

static float read_f32(ByteReader &reader) {
    uint32_t bits = read_u32(reader);
    float value;
    memcpy(&value, &bits, 4);
    return value;
}

static Vec3 read_vec3(ByteReader &reader) {
    float x = read_f32(reader);
    float y = read_f32(reader);
    float z = read_f32(reader);
    return {x, y, z};
}

static LoadStatus read_header(ByteReader &reader, const uint8_t magic[4], uint32_t version) {
    uint8_t found[4];
    read_bytes(reader, found, 4);
    if (reader.failed) return LOAD_STATUS_TRUNCATED;
    if (memcmp(found, magic, 4) != 0) return LOAD_STATUS_BAD_MAGIC;
    uint32_t found_version = read_u32(reader);
    if (reader.failed) return LOAD_STATUS_TRUNCATED;
    if (found_version != version) return LOAD_STATUS_UNSUPPORTED_VERSION;
    return LOAD_STATUS_OK;
}

static void write_tile(ByteWriter &writer, const NavMeshTile &tile) {
    write_bytes(writer, TILE_MAGIC, 4);
    write_u32(writer, NAV_MESH_TILE_FORMAT_VERSION);
    write_i32(writer, tile.tile_x);
    write_i32(writer, tile.tile_y);
    write_i32(writer, tile.tile_layer);
    write_u32(writer, tile.vertices.length / 3);
    write_u32(writer, tile.polys.length);
    write_u32(writer, tile.poly_vertices.length);
    write_u32(writer, tile.detail_meshes.length);
    write_u32(writer, tile.detail_vertices.length / 3);
    write_u32(writer, tile.detail_triangles.length / 4);
    write_u32(writer, tile.bv_tree.nodes.length);
    write_vec3(writer, tile.bounds.min);
    write_vec3(writer, tile.bounds.max);
    write_f32(writer, tile.cell_size);
    write_f32(writer, tile.cell_height);
    write_f32(writer, tile.walkable_height);
    write_f32(writer, tile.walkable_radius);
    write_f32(writer, tile.walkable_climb);
    write_f32(writer, tile.bv_tree.quant_factor);

    for (float value : tile.vertices) write_f32(writer, value);
    for (const NavMeshPoly &poly : tile.polys) {
        write_u32(writer, poly.vertex_base);
        write_u8(writer, (uint8_t)poly.vertex_count);
        write_u8(writer, poly.area);
        write_u16(writer, 0);
        write_u32(writer, poly.flags);
    }
    for (uint32_t value : tile.poly_vertices) write_u32(writer, value);
    for (int value : tile.poly_neis) write_i32(writer, value);
    for (const NavMeshPolyDetail &detail : tile.detail_meshes) {
        write_u32(writer, detail.vertices_base);
        write_u32(writer, detail.triangles_base);
        write_u32(writer, detail.vertices_count);
        write_u32(writer, detail.triangles_count);
    }
    for (float value : tile.detail_vertices) write_f32(writer, value);
    write_bytes(writer, tile.detail_triangles.data, tile.detail_triangles.length);
    for (const NavMeshBvNode &node : tile.bv_tree.nodes) {
        for (int k = 0; k < 3; k++) write_u16(writer, node.bmin[k]);
        for (int k = 0; k < 3; k++) write_u16(writer, node.bmax[k]);
        write_i32(writer, node.i);
    }
}

static bool valid_tile(const NavMeshTile &tile) {
    uint32_t vertex_count = tile.vertices.length / 3;
    uint32_t poly_count = tile.polys.length;
    if (tile.poly_neis.length != tile.poly_vertices.length) return false;
    if (tile.detail_meshes.length != 0 && tile.detail_meshes.length != poly_count) return false;

    for (const NavMeshPoly &poly : tile.polys) {
        if (poly.vertex_count < 3 || poly.vertex_count > MAX_POLY_VERTICES) return false;
        if ((uint64_t)poly.vertex_base + poly.vertex_count > tile.poly_vertices.length) return false;
    }
    for (uint32_t vertex : tile.poly_vertices) {
        if (vertex >= vertex_count) return false;
    }
    for (int nei : tile.poly_neis) {
        if (nei & POLY_NEIS_FLAG_EXT_LINK) {
            if ((nei & ~(POLY_NEIS_FLAG_EXT_LINK | POLY_NEIS_FLAG_EXT_LINK_DIR_MASK)) != 0) return false;
            if ((nei & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK) >= 8) return false;
        } else if (nei < 0 || (uint32_t)nei > poly_count) {
            return false;
        }
    }

    uint32_t detail_vertex_count = tile.detail_vertices.length / 3;
    uint32_t detail_triangle_count = tile.detail_triangles.length / 4;
    for (uint32_t i = 0; i < tile.detail_meshes.length; i++) {
        const NavMeshPolyDetail &detail = tile.detail_meshes[i];
        if ((uint64_t)detail.vertices_base + detail.vertices_count > detail_vertex_count) return false;
        if ((uint64_t)detail.triangles_base + detail.triangles_count > detail_triangle_count) return false;
        uint32_t limit = tile.polys[i].vertex_count + detail.vertices_count;
        for (uint32_t t = 0; t < detail.triangles_count; t++) {
            const uint8_t *triangle = &tile.detail_triangles[(detail.triangles_base + t) * 4];
            for (int k = 0; k < 3; k++) {
                if (triangle[k] >= limit) return false;
            }
        }
    }

    for (const NavMeshBvNode &node : tile.bv_tree.nodes) {
        if (node.i >= 0 && (uint32_t)node.i >= poly_count) return false;
        if (node.i == INT32_MIN) return false;
    }
    return true;
}

static LoadStatus read_tile(ByteReader &reader, NavMeshTile &tile) {
    tile = {};
    LoadStatus status = read_header(reader, TILE_MAGIC, NAV_MESH_TILE_FORMAT_VERSION);
    if (status != LOAD_STATUS_OK) return status;

    tile.tile_x = read_i32(reader);
    tile.tile_y = read_i32(reader);
    tile.tile_layer = read_i32(reader);
    uint32_t vertex_count = read_u32(reader);
    uint32_t poly_count = read_u32(reader);
    uint32_t poly_vertex_count = read_u32(reader);
    uint32_t detail_mesh_count = read_u32(reader);
    uint32_t detail_vertex_count = read_u32(reader);
    uint32_t detail_triangle_count = read_u32(reader);
    uint32_t bv_node_count = read_u32(reader);
    tile.bounds.min = read_vec3(reader);
    tile.bounds.max = read_vec3(reader);
    tile.cell_size = read_f32(reader);
    tile.cell_height = read_f32(reader);
    tile.walkable_height = read_f32(reader);
    tile.walkable_radius = read_f32(reader);
    tile.walkable_climb = read_f32(reader);
    tile.bv_tree.quant_factor = read_f32(reader);

    if (can_read(reader, vertex_count, 12)) {
        tile.vertices.resize(vertex_count * 3);
        for (float &value : tile.vertices) value = read_f32(reader);
    }
    if (can_read(reader, poly_count, 12)) {
        tile.polys.resize(poly_count);
        for (NavMeshPoly &poly : tile.polys) {
            poly.vertex_base = read_u32(reader);
            poly.vertex_count = read_u8(reader);
            poly.area = read_u8(reader);
            read_u16(reader);
            poly.flags = read_u32(reader);
        }
    }
    if (can_read(reader, poly_vertex_count, 8)) {
        tile.poly_vertices.resize(poly_vertex_count);
        for (uint32_t &value : tile.poly_vertices) value = read_u32(reader);
        tile.poly_neis.resize(poly_vertex_count);
        for (int &value : tile.poly_neis) value = read_i32(reader);
    }
    if (can_read(reader, detail_mesh_count, 16)) {
        tile.detail_meshes.resize(detail_mesh_count);
        for (NavMeshPolyDetail &detail : tile.detail_meshes) {
            detail.vertices_base = read_u32(reader);
            detail.triangles_base = read_u32(reader);
            detail.vertices_count = read_u32(reader);
            detail.triangles_count = read_u32(reader);
        }
    }
    if (can_read(reader, detail_vertex_count, 12)) {
        tile.detail_vertices.resize(detail_vertex_count * 3);
        for (float &value : tile.detail_vertices) value = read_f32(reader);
    }
    if (can_read(reader, detail_triangle_count, 4)) {
        tile.detail_triangles.resize(detail_triangle_count * 4);
        read_bytes(reader, tile.detail_triangles.data, detail_triangle_count * 4);
    }
    if (can_read(reader, bv_node_count, 16)) {
        tile.bv_tree.nodes.resize(bv_node_count);
        for (NavMeshBvNode &node : tile.bv_tree.nodes) {
            for (int k = 0; k < 3; k++) node.bmin[k] = read_u16(reader);
            for (int k = 0; k < 3; k++) node.bmax[k] = read_u16(reader);
            node.i = read_i32(reader);
        }
    }

    if (reader.failed) return LOAD_STATUS_TRUNCATED;
    if (!valid_tile(tile)) return LOAD_STATUS_INVALID_DATA;
    return LOAD_STATUS_OK;
}

void save_tile(SaveResult &out, const NavMeshTile &tile, Span<uint8_t> bytes) {
    ByteWriter writer = {bytes.data, bytes.capacity, 0};
    write_tile(writer, tile);
    finish_save(out, writer);
}

LoadStatus load_tile(NavMeshTile &tile, Span<const uint8_t> bytes) {
    ByteReader reader = {bytes.data, bytes.capacity, 0, false};
    LoadStatus status = read_tile(reader, tile);
    if (status != LOAD_STATUS_OK) nav_mesh_api::release(tile);
    return status;
}

static void write_index_pool(ByteWriter &writer, const IndexPool &pool) {
    write_u32(writer, pool.counter);
    write_u32(writer, pool.free.length);
    for (uint32_t index : pool.free) write_u32(writer, index);
}

static void read_index_pool(ByteReader &reader, IndexPool &pool) {
    pool.counter = read_u32(reader);
    uint32_t free_count = read_u32(reader);
    if (!can_read(reader, free_count, 4)) return;
    pool.free.resize(free_count);
    for (uint32_t &index : pool.free) index = read_u32(reader);
}

struct KeyedValue {
    uint64_t key;
    uint32_t value;
};

static void write_sorted_map(ByteWriter &writer, const Map<uint64_t, uint32_t> &map) {
    Array<KeyedValue> entries = {};
    for (uint32_t i = 0; i < map.capacity; i++) {
        if (map.slots[i] == MAP_SLOT_OCCUPIED) entries.push({map.keys[i], map.values[i]});
    }
    sort::stable(entries.data, entries.length, [](const KeyedValue &a, const KeyedValue &b) { return a.key < b.key; });
    write_u32(writer, entries.length);
    for (const KeyedValue &entry : entries) {
        write_u64(writer, entry.key);
        write_u32(writer, entry.value);
    }
    entries.release();
}

static void write_columns(ByteWriter &writer, const Map<uint64_t, Array<uint32_t>> &columns) {
    Array<KeyedValue> entries = {};
    for (uint32_t i = 0; i < columns.capacity; i++) {
        if (columns.slots[i] == MAP_SLOT_OCCUPIED) entries.push({columns.keys[i], i});
    }
    sort::stable(entries.data, entries.length, [](const KeyedValue &a, const KeyedValue &b) { return a.key < b.key; });
    write_u32(writer, entries.length);
    for (const KeyedValue &entry : entries) {
        const Array<uint32_t> &ids = columns.values[entry.value];
        write_u64(writer, entry.key);
        write_u32(writer, ids.length);
        for (uint32_t id : ids) write_u32(writer, id);
    }
    entries.release();
}

static void write_nav_mesh(ByteWriter &writer, const NavMesh &nav_mesh) {
    write_bytes(writer, NAV_MESH_MAGIC, 4);
    write_u32(writer, NAV_MESH_FORMAT_VERSION);
    write_vec3(writer, nav_mesh.origin);
    write_f32(writer, nav_mesh.tile_width);
    write_f32(writer, nav_mesh.tile_height);

    write_u32(writer, nav_mesh.nodes.length);
    for (const NavMeshNode &node : nav_mesh.nodes) {
        write_u8(writer, node.allocated ? 1 : 0);
        if (!node.allocated) continue;
        write_u8(writer, node.type);
        write_u8(writer, node.area);
        write_u8(writer, 0);
        write_u64(writer, node.ref);
        write_i32(writer, node.first_link);
        write_u32(writer, node.flags);
        write_i32(writer, node.tile_id);
        write_i32(writer, node.poly_index);
        write_i32(writer, node.off_mesh_connection_id);
    }

    write_u32(writer, nav_mesh.links.length);
    for (const NavMeshLink &link : nav_mesh.links) {
        write_u64(writer, link.to_node_ref);
        write_i32(writer, link.next);
        write_u8(writer, link.edge);
        write_u8(writer, link.side);
        write_u8(writer, link.bmin);
        write_u8(writer, link.bmax);
    }

    write_u32(writer, nav_mesh.tiles.length);
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        write_u8(writer, tile.allocated ? 1 : 0);
        if (!tile.allocated) continue;
        write_u32(writer, tile.sequence);
        ByteWriter measure = {nullptr, 0, 0};
        write_tile(measure, tile);
        write_u32(writer, measure.size <= UINT32_MAX ? (uint32_t)measure.size : UINT32_MAX);
        write_tile(writer, tile);
        for (uint32_t node_index : tile.poly_nodes) write_u32(writer, node_index);
    }

    write_u32(writer, nav_mesh.off_mesh_connections.length);
    for (const OffMeshConnection &connection : nav_mesh.off_mesh_connections) {
        write_u8(writer, connection.allocated ? 1 : 0);
        if (!connection.allocated) continue;
        write_u32(writer, connection.sequence);
        write_vec3(writer, connection.start);
        write_vec3(writer, connection.end);
        write_f32(writer, connection.radius);
        write_u32(writer, connection.direction);
        write_u32(writer, connection.flags);
        write_u8(writer, connection.area);
    }

    write_u32(writer, nav_mesh.off_mesh_connection_attachments.length);
    for (const OffMeshConnectionAttachment &attachment : nav_mesh.off_mesh_connection_attachments) {
        write_u8(writer, attachment.attached ? 1 : 0);
        if (!attachment.attached) continue;
        write_u64(writer, attachment.off_mesh_node);
        write_u64(writer, attachment.start_poly_node);
        write_u64(writer, attachment.end_poly_node);
    }

    write_columns(writer, nav_mesh.tile_column_to_tile_ids);
    write_sorted_map(writer, nav_mesh.tile_position_to_sequence_counter);
    write_u32(writer, nav_mesh.off_mesh_connection_sequence_counter);
    write_index_pool(writer, nav_mesh.node_index_pool);
    write_index_pool(writer, nav_mesh.tile_index_pool);
    write_index_pool(writer, nav_mesh.off_mesh_connection_index_pool);
    write_index_pool(writer, nav_mesh.link_index_pool);
}

void save_nav_mesh(SaveResult &out, const NavMesh &nav_mesh, Span<uint8_t> bytes) {
    ByteWriter writer = {bytes.data, bytes.capacity, 0};
    write_nav_mesh(writer, nav_mesh);
    finish_save(out, writer);
}

static NavMeshNode released_node() {
    NavMeshNode node = {};
    node.first_link = NULL_LINK;
    node.tile_id = -1;
    node.poly_index = -1;
    node.off_mesh_connection_id = -1;
    return node;
}

static LoadStatus read_nav_mesh(ByteReader &reader, NavMesh &nav_mesh) {
    LoadStatus status = read_header(reader, NAV_MESH_MAGIC, NAV_MESH_FORMAT_VERSION);
    if (status != LOAD_STATUS_OK) return status;

    nav_mesh.origin = read_vec3(reader);
    nav_mesh.tile_width = read_f32(reader);
    nav_mesh.tile_height = read_f32(reader);

    uint32_t node_count = read_u32(reader);
    if (!can_read(reader, node_count, 1)) return LOAD_STATUS_TRUNCATED;
    nav_mesh.nodes.resize(node_count, released_node());
    for (NavMeshNode &node : nav_mesh.nodes) {
        if (read_u8(reader) == 0) continue;
        node.allocated = true;
        node.type = (NodeType)read_u8(reader);
        node.area = read_u8(reader);
        read_u8(reader);
        node.ref = read_u64(reader);
        node.first_link = read_i32(reader);
        node.flags = read_u32(reader);
        node.tile_id = read_i32(reader);
        node.poly_index = read_i32(reader);
        node.off_mesh_connection_id = read_i32(reader);
        if (reader.failed) return LOAD_STATUS_TRUNCATED;
    }

    uint32_t link_count = read_u32(reader);
    if (!can_read(reader, link_count, 16)) return LOAD_STATUS_TRUNCATED;
    nav_mesh.links.resize(link_count);
    for (NavMeshLink &link : nav_mesh.links) {
        link.to_node_ref = read_u64(reader);
        link.next = read_i32(reader);
        link.edge = read_u8(reader);
        link.side = read_u8(reader);
        link.bmin = read_u8(reader);
        link.bmax = read_u8(reader);
    }

    uint32_t tile_count = read_u32(reader);
    if (!can_read(reader, tile_count, 1)) return LOAD_STATUS_TRUNCATED;
    nav_mesh.tiles.resize(tile_count, {});
    for (uint32_t id = 0; id < tile_count; id++) {
        if (read_u8(reader) == 0) continue;
        uint32_t sequence = read_u32(reader);
        uint32_t size = read_u32(reader);
        if (!can_read(reader, size, 1)) return LOAD_STATUS_TRUNCATED;
        ByteReader tile_reader = {reader.data + reader.offset, size, 0, false};
        reader.offset += size;
        NavMeshTile &tile = nav_mesh.tiles[id];
        status = read_tile(tile_reader, tile);
        if (status != LOAD_STATUS_OK) return status;
        if (tile_reader.offset != size) return LOAD_STATUS_INVALID_DATA;
        tile.allocated = true;
        tile.id = id;
        tile.sequence = sequence;
        if (!can_read(reader, tile.polys.length, 4)) return LOAD_STATUS_TRUNCATED;
        tile.poly_nodes.resize(tile.polys.length);
        for (uint32_t &node_index : tile.poly_nodes) node_index = read_u32(reader);
        nav_mesh_api::build_tile_portals(tile);
    }

    uint32_t connection_count = read_u32(reader);
    if (!can_read(reader, connection_count, 1)) return LOAD_STATUS_TRUNCATED;
    nav_mesh.off_mesh_connections.resize(connection_count, {});
    for (uint32_t id = 0; id < connection_count; id++) {
        if (read_u8(reader) == 0) continue;
        OffMeshConnection &connection = nav_mesh.off_mesh_connections[id];
        connection.allocated = true;
        connection.id = id;
        connection.sequence = read_u32(reader);
        connection.start = read_vec3(reader);
        connection.end = read_vec3(reader);
        connection.radius = read_f32(reader);
        connection.direction = (OffMeshConnectionDirection)read_u32(reader);
        connection.flags = read_u32(reader);
        connection.area = read_u8(reader);
        if (reader.failed) return LOAD_STATUS_TRUNCATED;
    }

    uint32_t attachment_count = read_u32(reader);
    if (!can_read(reader, attachment_count, 1)) return LOAD_STATUS_TRUNCATED;
    nav_mesh.off_mesh_connection_attachments.resize(attachment_count, {});
    for (OffMeshConnectionAttachment &attachment : nav_mesh.off_mesh_connection_attachments) {
        if (read_u8(reader) == 0) continue;
        attachment.attached = true;
        attachment.off_mesh_node = read_u64(reader);
        attachment.start_poly_node = read_u64(reader);
        attachment.end_poly_node = read_u64(reader);
        if (reader.failed) return LOAD_STATUS_TRUNCATED;
    }

    uint32_t column_count = read_u32(reader);
    if (!can_read(reader, column_count, 12)) return LOAD_STATUS_TRUNCATED;
    for (uint32_t c = 0; c < column_count; c++) {
        uint64_t key = read_u64(reader);
        uint32_t id_count = read_u32(reader);
        if (!can_read(reader, id_count, 4)) return LOAD_STATUS_TRUNCATED;
        if (id_count == 0 || nav_mesh.tile_column_to_tile_ids.has(key)) return LOAD_STATUS_INVALID_DATA;
        Array<uint32_t> &ids = nav_mesh.tile_column_to_tile_ids.set(key, {});
        ids.resize(id_count);
        for (uint32_t &id : ids) id = read_u32(reader);
    }

    uint32_t counter_count = read_u32(reader);
    if (!can_read(reader, counter_count, 12)) return LOAD_STATUS_TRUNCATED;
    for (uint32_t c = 0; c < counter_count; c++) {
        uint64_t key = read_u64(reader);
        uint32_t counter = read_u32(reader);
        if (nav_mesh.tile_position_to_sequence_counter.has(key)) return LOAD_STATUS_INVALID_DATA;
        nav_mesh.tile_position_to_sequence_counter.set(key, counter);
    }

    nav_mesh.off_mesh_connection_sequence_counter = read_u32(reader);
    read_index_pool(reader, nav_mesh.node_index_pool);
    read_index_pool(reader, nav_mesh.tile_index_pool);
    read_index_pool(reader, nav_mesh.off_mesh_connection_index_pool);
    read_index_pool(reader, nav_mesh.link_index_pool);
    if (reader.failed) return LOAD_STATUS_TRUNCATED;
    if (reader.offset != reader.size) return LOAD_STATUS_INVALID_DATA;
    return LOAD_STATUS_OK;
}

static bool valid_pool(const IndexPool &pool, uint32_t length, const Array<bool> &used) {
    if (pool.counter != length) return false;
    Array<bool> free = {};
    free.resize(length, false);
    bool valid = true;
    for (uint32_t index : pool.free) {
        if (index >= length || free[index] || used[index]) {
            valid = false;
            break;
        }
        free[index] = true;
    }
    for (uint32_t i = 0; valid && i < length; i++) {
        if (!used[i] && !free[i]) valid = false;
    }
    free.release();
    return valid;
}

static bool node_ref_matches(const NavMesh &nav_mesh, NodeRef ref, NodeType type) {
    if (node::get_node_ref_type(ref) != type) return false;
    uint32_t index = node::get_node_ref_index(ref);
    if (index >= nav_mesh.nodes.length) return false;
    const NavMeshNode &node = nav_mesh.nodes[index];
    return node.allocated && node.ref == ref;
}

static bool valid_nav_mesh(const NavMesh &nav_mesh) {
    uint32_t node_count = nav_mesh.nodes.length;
    uint32_t link_count = nav_mesh.links.length;
    uint32_t tile_count = nav_mesh.tiles.length;
    uint32_t connection_count = nav_mesh.off_mesh_connections.length;
    if (node_count > node::MAX_NODE_INDEX + 1) return false;
    if (nav_mesh.off_mesh_connection_attachments.length > connection_count) return false;

    for (uint32_t i = 0; i < node_count; i++) {
        const NavMeshNode &node = nav_mesh.nodes[i];
        if (!node.allocated) continue;
        if (node::get_node_ref_index(node.ref) != i || node::get_node_ref_type(node.ref) != node.type) return false;
        if (node.first_link != NULL_LINK && (node.first_link < 0 || (uint32_t)node.first_link >= link_count)) return false;
        if (node.type == NODE_TYPE_POLY) {
            if (node.tile_id < 0 || (uint32_t)node.tile_id >= tile_count) return false;
            const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
            if (!tile.allocated || node.poly_index < 0 || (uint32_t)node.poly_index >= tile.polys.length) return false;
            if (tile.poly_nodes[(uint32_t)node.poly_index] != i) return false;
            if (node::get_node_ref_sequence(node.ref) != tile.sequence) return false;
        } else if (node.type == NODE_TYPE_OFFMESH) {
            int32_t id = node.off_mesh_connection_id;
            if (id < 0 || (uint32_t)id >= connection_count || (uint32_t)id >= nav_mesh.off_mesh_connection_attachments.length) return false;
            const OffMeshConnection &connection = nav_mesh.off_mesh_connections[(uint32_t)id];
            const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[(uint32_t)id];
            if (!connection.allocated || !attachment.attached || attachment.off_mesh_node != node.ref) return false;
            if (node::get_node_ref_sequence(node.ref) != connection.sequence) return false;
        } else {
            return false;
        }
    }

    for (uint32_t id = 0; id < tile_count; id++) {
        const NavMeshTile &tile = nav_mesh.tiles[id];
        if (!tile.allocated) continue;
        if (tile.sequence > node::MAX_SEQUENCE) return false;
        for (uint32_t p = 0; p < tile.poly_nodes.length; p++) {
            uint32_t node_index = tile.poly_nodes[p];
            if (node_index >= node_count) return false;
            const NavMeshNode &node = nav_mesh.nodes[node_index];
            if (!node.allocated || node.type != NODE_TYPE_POLY || node.tile_id != (int32_t)id || node.poly_index != (int32_t)p) return false;
        }
    }

    for (uint32_t id = 0; id < connection_count; id++) {
        const OffMeshConnection &connection = nav_mesh.off_mesh_connections[id];
        if (!connection.allocated) continue;
        if (connection.direction != OFF_MESH_CONNECTION_DIRECTION_START_TO_END && connection.direction != OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL) {
            return false;
        }
        if (connection.sequence > node::MAX_SEQUENCE) return false;
    }

    for (uint32_t id = 0; id < nav_mesh.off_mesh_connection_attachments.length; id++) {
        const OffMeshConnectionAttachment &attachment = nav_mesh.off_mesh_connection_attachments[id];
        if (!attachment.attached) continue;
        if (!nav_mesh.off_mesh_connections[id].allocated) return false;
        if (!node_ref_matches(nav_mesh, attachment.off_mesh_node, NODE_TYPE_OFFMESH)) return false;
        if (!node_ref_matches(nav_mesh, attachment.start_poly_node, NODE_TYPE_POLY)) return false;
        if (!node_ref_matches(nav_mesh, attachment.end_poly_node, NODE_TYPE_POLY)) return false;
        if (nav_mesh.nodes[node::get_node_ref_index(attachment.off_mesh_node)].off_mesh_connection_id != (int32_t)id) return false;
    }

    Array<bool> link_used = {};
    link_used.resize(link_count, false);
    bool valid = true;
    for (uint32_t i = 0; valid && i < node_count; i++) {
        const NavMeshNode &node = nav_mesh.nodes[i];
        if (!node.allocated && node.first_link != NULL_LINK) valid = false;
        if (!node.allocated) continue;
        for (int32_t l = node.first_link; valid && l != NULL_LINK; l = nav_mesh.links[(uint32_t)l].next) {
            if (l < 0 || (uint32_t)l >= link_count || link_used[(uint32_t)l]) {
                valid = false;
                break;
            }
            link_used[(uint32_t)l] = true;
            const NavMeshLink &link = nav_mesh.links[(uint32_t)l];
            if (link.next != NULL_LINK && (link.next < 0 || (uint32_t)link.next >= link_count)) valid = false;
            NodeType to_type = node::get_node_ref_type(link.to_node_ref);
            if (!node_ref_matches(nav_mesh, link.to_node_ref, to_type)) valid = false;
            if (valid && node.type == NODE_TYPE_POLY && to_type == NODE_TYPE_POLY) {
                const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)node.tile_id];
                if (link.edge >= tile.polys[(uint32_t)node.poly_index].vertex_count) valid = false;
            }
        }
    }
    if (valid) valid = valid_pool(nav_mesh.link_index_pool, link_count, link_used);
    link_used.release();
    if (!valid) return false;

    Array<bool> used = {};
    used.resize(node_count, false);
    for (uint32_t i = 0; i < node_count; i++) used[i] = nav_mesh.nodes[i].allocated;
    valid = valid_pool(nav_mesh.node_index_pool, node_count, used);
    used.resize(tile_count);
    for (uint32_t i = 0; i < tile_count; i++) used[i] = nav_mesh.tiles[i].allocated;
    if (valid) valid = valid_pool(nav_mesh.tile_index_pool, tile_count, used);
    used.resize(connection_count);
    for (uint32_t i = 0; i < connection_count; i++) used[i] = nav_mesh.off_mesh_connections[i].allocated;
    if (valid) valid = valid_pool(nav_mesh.off_mesh_connection_index_pool, connection_count, used);

    Array<bool> listed = {};
    listed.resize(tile_count, false);
    const Map<uint64_t, Array<uint32_t>> &columns = nav_mesh.tile_column_to_tile_ids;
    for (uint32_t s = 0; valid && s < columns.capacity; s++) {
        if (columns.slots[s] != MAP_SLOT_OCCUPIED) continue;
        for (uint32_t id : columns.values[s]) {
            if (id >= tile_count || !nav_mesh.tiles[id].allocated || listed[id]) {
                valid = false;
                break;
            }
            const NavMeshTile &tile = nav_mesh.tiles[id];
            if (nav_mesh_api::get_tile_column_key(tile.tile_x, tile.tile_y) != columns.keys[s]) valid = false;
            listed[id] = true;
        }
    }
    for (uint32_t id = 0; valid && id < tile_count; id++) {
        if (nav_mesh.tiles[id].allocated && !listed[id]) valid = false;
    }
    listed.release();
    used.release();
    return valid;
}

static bool restore_tile_positions(NavMesh &nav_mesh) {
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        uint64_t key = nav_mesh_api::get_tile_position_key(tile.tile_x, tile.tile_y, tile.tile_layer);
        if (nav_mesh.tile_position_to_tile_id.has(key)) return false;
        nav_mesh.tile_position_to_tile_id.set(key, tile.id);
    }
    return true;
}

LoadStatus load_nav_mesh(NavMesh &nav_mesh, Span<const uint8_t> bytes) {
    nav_mesh = nav_mesh_api::create_nav_mesh();
    ByteReader reader = {bytes.data, bytes.capacity, 0, false};
    LoadStatus status = read_nav_mesh(reader, nav_mesh);
    if (status == LOAD_STATUS_OK && !(valid_nav_mesh(nav_mesh) && restore_tile_positions(nav_mesh))) status = LOAD_STATUS_INVALID_DATA;
    if (status != LOAD_STATUS_OK) nav_mesh_api::release(nav_mesh);
    return status;
}

}

}
