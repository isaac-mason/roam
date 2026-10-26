#include <math.h>

#include "roam/blocks/agents/path_corridor.h"
#include "roam/query/find_path.h"
#include "roam/query/find_straight_path.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_io.h"
#include "roam/query/nav_mesh_search.h"
#include "tests/fuzz/fuzz.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

static void push_strip(Array<float> &positions, Array<uint32_t> &indices, float x0, float x1, float z0, float z1, float y, int pieces) {
    uint32_t base = positions.length / 3;
    for (int i = 0; i <= pieces; i++) {
        float x = x0 + (x1 - x0) * (float)i / (float)pieces;
        const float column[] = {x, y, z0, x, y, z1};
        for (float value : column) positions.push(value);
    }
    for (uint32_t i = 0; i < (uint32_t)pieces; i++) {
        uint32_t a = base + i * 2;
        const uint32_t triangles[] = {a, a + 1, a + 3, a, a + 3, a + 2};
        for (uint32_t index : triangles) indices.push(index);
    }
}

static NavMesh create_ledge() {
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    push_strip(positions, indices, 0, 10, 0, 4, 2, 4);
    push_strip(positions, indices, 0, 10, 5, 10, 0, 3);
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions.data, positions.length / 3, indices.data, indices.length);
    positions.release();
    indices.release();
    return nav_mesh;
}

static OffMeshSegmentConnectionParams ledge_link(OffMeshConnectionDirection direction) {
    return {
        .start_a = {1, 2, 4},
        .start_b = {9, 2, 4},
        .end_a = {1, 0, 5},
        .end_b = {9, 0, 5},
        .radius = 0.5f,
        .inset = 0.2f,
        .direction = direction,
        .flags = 1,
        .area = 0,
        .align = false,
    };
}

static NodeRef nearest(const NavMesh &nav_mesh, Vec3 position) {
    FindNearestPolyResult result = {};
    nav_mesh_api::find_nearest_poly(result, nav_mesh, position, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    REQUIRE(result.success);
    return result.node_ref;
}

struct PieceSummary {
    uint32_t count;
    float covered;
    bool ordered;
};

static PieceSummary summarise_pieces(const NavMesh &nav_mesh, uint32_t id) {
    PieceSummary summary = {0, 0, true};
    if (id >= nav_mesh.off_mesh_connection_attachments.length || !nav_mesh.off_mesh_connection_attachments[id].attached) return summary;
    float previous_end = 0;
    for (int32_t i = nav_mesh.off_mesh_connection_attachments[id].first_piece; i >= 0; i = nav_mesh.off_mesh_connection_pieces[(uint32_t)i].next) {
        const OffMeshConnectionPiece &piece = nav_mesh.off_mesh_connection_pieces[(uint32_t)i];
        summary.count++;
        summary.covered += piece.t1 - piece.t0;
        if (piece.t0 < previous_end - 1e-5f) summary.ordered = false;
        previous_end = piece.t1;
    }
    return summary;
}

struct OffMeshCorner {
    bool found;
    StraightPathPoint entry;
    StraightPathPoint exit;
};

static OffMeshCorner find_off_mesh_corner(const StraightPathPoint *points, uint32_t count) {
    for (uint32_t i = 0; i + 1 < count; i++) {
        if (points[i].type == NODE_TYPE_OFFMESH) return {true, points[i], points[i + 1]};
    }
    return {false, {}, {}};
}

TEST_CASE("segment connections split into pieces along the polys under both segments") {
    NavMesh nav_mesh = create_ledge();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));

    REQUIRE(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id));
    PieceSummary summary = summarise_pieces(nav_mesh, id);
    CHECK(summary.count >= 4);
    CHECK(summary.ordered);
    CHECK_NEAR(summary.covered, 1.0f, 0.06f);
    CHECK(segment_pieces_are_consistent(nav_mesh));
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(snapshot_is_valid(nav_mesh));

    nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(snapshot_is_valid(nav_mesh));
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("segment connections only cover where both sides have mesh under them") {
    NavMesh nav_mesh = create_ledge();
    OffMeshSegmentConnectionParams params = ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END);
    params.start_a = {-5, 2, 3.7f};
    params.start_b = {15, 2, 3.7f};
    params.end_a = {-5, 0, 5.3f};
    params.end_b = {15, 0, 5.3f};
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, params);

    REQUIRE(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id));
    PieceSummary summary = summarise_pieces(nav_mesh, id);
    CHECK(summary.covered > 0.45f);
    CHECK(summary.covered < 0.5f);
    CHECK(segment_pieces_are_consistent(nav_mesh));
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("segment connections do not snap stretches that lie off the mesh") {
    NavMesh nav_mesh = create_ledge();
    OffMeshSegmentConnectionParams params = ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END);
    params.start_a = {1, 2, 4.3f};
    params.start_b = {9, 2, 4.3f};
    params.end_a = {1, 0, 4.7f};
    params.end_b = {9, 0, 4.7f};
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, params);

    CHECK_FALSE(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id));
    CHECK(summarise_pieces(nav_mesh, id).count == 0);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("segment connections attach to the poly nearest in height where polys overlap") {
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    push_strip(positions, indices, 0, 10, 0, 4, 0, 2);
    push_strip(positions, indices, 0, 10, 0, 4, 3, 2);
    push_strip(positions, indices, 0, 10, 6, 10, 0, 2);
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions.data, positions.length / 3, indices.data, indices.length);
    positions.release();
    indices.release();
    OffMeshSegmentConnectionParams params = ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END);
    params.start_a = {1, 3, 3.5f};
    params.start_b = {9, 3, 3.5f};
    params.end_a = {1, 0, 6.5f};
    params.end_b = {9, 0, 6.5f};
    params.radius = 0.5f;
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, params);

    REQUIRE(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id));
    for (int32_t i = nav_mesh.off_mesh_connection_attachments[id].first_piece; i >= 0; i = nav_mesh.off_mesh_connection_pieces[(uint32_t)i].next) {
        const OffMeshConnectionPiece &piece = nav_mesh.off_mesh_connection_pieces[(uint32_t)i];
        GetClosestPointOnPolyResult closest = {};
        nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, piece.start_poly_node, {5, 3, 3.5f});
        CHECK_NEAR(closest.position.y, 3.0f, 0.1f);
    }
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("paths cross a segment connection where they meet it and land at the matching point") {
    NavMesh nav_mesh = create_ledge();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    const OffMeshConnectionSegment &segment = nav_mesh.off_mesh_connections[id].segment;

    const float starts[] = {2, 5, 8};
    for (float x : starts) {
        INFO("start x ", x);
        StraightPathPoint points[32];
        FindPathResult path = {};
        find_path::find_path(path, workspace, nav_mesh, {x, 2, 1}, {x, 0, 9}, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
        REQUIRE(path.success);
        OffMeshCorner corner = find_off_mesh_corner(points, path.point_count);
        REQUIRE(corner.found);
        CHECK_NEAR(corner.entry.position.z, 4.0f, 1e-3f);
        CHECK_NEAR(corner.exit.position.z, 5.0f, 1e-3f);
        CHECK_NEAR(corner.entry.position.x, corner.exit.position.x, 1e-3f);
        CHECK(corner.entry.position.x >= segment.start_a.x - 1e-3f);
        CHECK(corner.entry.position.x <= segment.start_b.x + 1e-3f);
        CHECK(fabsf(corner.entry.position.x - x) < 1.3f);
    }

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("one way segment connections are only searchable from the start side") {
    NavMesh nav_mesh = create_ledge();
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    NodeRef upper = nearest(nav_mesh, {5, 2, 1});
    NodeRef lower = nearest(nav_mesh, {5, 0, 9});

    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, upper, lower));
    CHECK_FALSE(nav_mesh_search::is_reachable(workspace, nav_mesh, lower, upper));

    nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);
    nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL));
    CHECK(nav_mesh_search::is_reachable(workspace, nav_mesh, lower, upper));

    StraightPathPoint points[32];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, {3, 0, 9}, {3, 2, 1}, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
    REQUIRE(path.success);
    OffMeshCorner corner = find_off_mesh_corner(points, path.point_count);
    REQUIRE(corner.found);
    CHECK_NEAR(corner.entry.position.z, 5.0f, 1e-3f);
    CHECK_NEAR(corner.exit.position.z, 4.0f, 1e-3f);
    CHECK_NEAR(corner.entry.position.x, corner.exit.position.x, 1e-3f);
    CHECK(clusters_are_consistent(nav_mesh));

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("the inset trims the ends of covered stretches and crossings land on the end poly") {
    NavMesh nav_mesh = create_ledge();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));
    float length = 8;
    const OffMeshConnectionPiece &first = first_piece(nav_mesh, id);
    CHECK_NEAR(first.t0, 0.2f / length, 2e-3f);

    float previous_end = -1;
    float last_end = 0;
    uint32_t gaps = 0;
    for (int32_t i = nav_mesh.off_mesh_connection_attachments[id].first_piece; i >= 0; i = nav_mesh.off_mesh_connection_pieces[(uint32_t)i].next) {
        const OffMeshConnectionPiece &piece = nav_mesh.off_mesh_connection_pieces[(uint32_t)i];
        if (previous_end >= 0 && piece.t0 - previous_end > 1e-5f) gaps++;
        previous_end = piece.t1;
        last_end = piece.t1;
    }
    CHECK(gaps == 0);
    CHECK_NEAR(last_end, 1 - 0.2f / length, 2e-3f);

    OffMeshCrossing crossing = {};
    nav_mesh_api::get_off_mesh_crossing(crossing, nav_mesh, first.start_poly_node, first.off_mesh_node, {-20, 2, 3});
    REQUIRE(crossing.success);
    CHECK(crossing.from_start);
    CHECK_NEAR(crossing.t, first.t0, 1e-5f);
    CHECK(crossing.end_poly_ref == first.end_poly_node);
    CHECK_NEAR(crossing.landing.z, 5.0f, 1e-3f);

    nav_mesh_api::get_off_mesh_crossing(crossing, nav_mesh, first.start_poly_node, first.off_mesh_node, {20, 2, 3});
    CHECK_NEAR(crossing.t, first.t1, 1e-5f);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("the path corridor moves over a segment connection to the matching exit point") {
    NavMesh nav_mesh = create_ledge();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));
    const OffMeshConnectionPiece &piece = first_piece(nav_mesh, id);
    const OffMeshConnection &connection = nav_mesh.off_mesh_connections[id];
    float middle = (piece.t0 + piece.t1) * 0.5f;
    Vec3 entry = vec3::lerp(connection.segment.start_a, connection.segment.start_b, middle);

    PathCorridor corridor = path_corridor::create();
    NodeRef path[] = {piece.start_poly_node, piece.off_mesh_node, piece.end_poly_node};
    path_corridor::set_path(corridor, {entry.x, 0, 9}, path);
    corridor.position = {entry.x, 2, 3.9f};

    MoveOverOffMeshConnectionResult result = {};
    path_corridor::move_over_off_mesh_connection(result, corridor, piece.off_mesh_node, nav_mesh);
    REQUIRE(result.success);
    CHECK_NEAR(result.t, middle, 1e-4f);
    CHECK_NEAR(result.end_position.x, entry.x, 1e-3f);
    CHECK_NEAR(result.end_position.z, 5.0f, 1e-3f);
    CHECK(result.end_node_ref == piece.end_poly_node);
    path_corridor::release(corridor);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("align swaps a reversed end segment") {
    NavMesh nav_mesh = create_ledge();
    OffMeshSegmentConnectionParams params = ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END);
    params.end_a = {9, 0, 5};
    params.end_b = {1, 0, 5};
    params.align = true;
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, params);
    CHECK(nav_mesh.off_mesh_connections[id].segment.end_a.x == 1);
    CHECK(nav_mesh.off_mesh_connections[id].segment.end_b.x == 9);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("segment connections regenerate as tiles under them change") {
    NavMesh nav_mesh = create_tiled_terrain();
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, 0);
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, {
                                                                              .start_a = {2, 0, 3},
                                                                              .start_b = {21, 0, 3},
                                                                              .end_a = {2, 0, 21},
                                                                              .end_b = {21, 0, 21},
                                                                              .radius = 0.8f,
                                                                              .inset = 0.1f,
                                                                              .direction = OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
                                                                              .flags = 1,
                                                                              .area = 0,
                                                                              .align = false,
                                                                          });
    REQUIRE(nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, id));
    PieceSummary before = summarise_pieces(nav_mesh, id);
    CHECK(before.covered > 0.9f);
    NodeRef old_piece = first_piece(nav_mesh, id).off_mesh_node;

    NodeRef start_poly = nearest(nav_mesh, {3, 0, 3});
    const NavMeshTile &tile = nav_mesh.tiles[(uint32_t)nav_mesh_api::get_node_by_ref(nav_mesh, start_poly).tile_id];
    int x = tile.tile_x;
    int y = tile.tile_y;
    int layer = tile.tile_layer;
    SaveResult size = {};
    nav_mesh_io::save_tile(size, tile, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_tile(saved, tile, bytes);

    nav_mesh_api::remove_tile(nav_mesh, x, y, layer);
    CHECK_FALSE(nav_mesh_api::is_valid_node_ref(nav_mesh, old_piece));
    PieceSummary removed = summarise_pieces(nav_mesh, id);
    CHECK(removed.covered < before.covered - 0.1f);
    CHECK(segment_pieces_are_consistent(nav_mesh));
    CHECK(clusters_are_consistent(nav_mesh));
    CHECK(snapshot_is_valid(nav_mesh));

    NavMeshTile restored = {};
    REQUIRE(nav_mesh_io::load_tile(restored, bytes) == LOAD_STATUS_OK);
    nav_mesh_api::add_tile(nav_mesh, restored);
    PieceSummary after = summarise_pieces(nav_mesh, id);
    CHECK_NEAR(after.covered, before.covered, 1e-4f);
    CHECK(after.count == before.count);
    CHECK(segment_pieces_are_consistent(nav_mesh));
    CHECK(clusters_are_consistent(nav_mesh));

    NodeRef unchanged = first_piece(nav_mesh, id).off_mesh_node;
    const NavMeshTile *middle = nav_mesh_api::get_tile_at(nav_mesh, 1, 1, 0);
    REQUIRE(middle != nullptr);
    SaveResult middle_size = {};
    nav_mesh_io::save_tile(middle_size, *middle, {});
    Array<uint8_t> middle_bytes = {};
    middle_bytes.resize(middle_size.byte_total);
    SaveResult middle_saved = {};
    nav_mesh_io::save_tile(middle_saved, *middle, middle_bytes);
    REQUIRE(nav_mesh_api::remove_tile(nav_mesh, 1, 1, 0));
    CHECK(first_piece(nav_mesh, id).off_mesh_node == unchanged);
    NavMeshTile middle_tile = {};
    REQUIRE(nav_mesh_io::load_tile(middle_tile, middle_bytes) == LOAD_STATUS_OK);
    nav_mesh_api::add_tile(nav_mesh, middle_tile);
    CHECK(first_piece(nav_mesh, id).off_mesh_node == unchanged);
    CHECK(nav_mesh_api::is_valid_node_ref(nav_mesh, unchanged));
    middle_bytes.release();

    bytes.release();
    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("saving and loading keeps segment connections and their pieces") {
    NavMesh nav_mesh = create_ledge();
    nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL));
    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{5, 2, 2}, {5, 0, 8}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});

    SaveResult size = {};
    nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
    Array<uint8_t> bytes = {};
    bytes.resize(size.byte_total);
    SaveResult saved = {};
    nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);
    NavMesh loaded = {};
    REQUIRE(nav_mesh_io::load_nav_mesh(loaded, bytes) == LOAD_STATUS_OK);

    REQUIRE(loaded.off_mesh_connections.length == 2);
    CHECK(loaded.off_mesh_connections[0].type == OFF_MESH_CONNECTION_TYPE_SEGMENT);
    CHECK(loaded.off_mesh_connections[1].type == OFF_MESH_CONNECTION_TYPE_POINT);
    CHECK(summarise_pieces(loaded, 0).count == summarise_pieces(nav_mesh, 0).count);
    CHECK(first_piece(loaded, 0).off_mesh_node == first_piece(nav_mesh, 0).off_mesh_node);
    CHECK(clusters_are_consistent(loaded));

    bytes.release();
    nav_mesh_api::release(loaded);
    nav_mesh_api::release(nav_mesh);
}

static float closest_t(Vec3 point, Vec3 a, Vec3 b) {
    float dx = b.x - a.x;
    float dz = b.z - a.z;
    return ((point.x - a.x) * dx + (point.z - a.z) * dz) / (dx * dx + dz * dz);
}

static float path_length(const StraightPathPoint *points, uint32_t count) {
    float length = 0;
    for (uint32_t i = 1; i < count; i++) length += vec3::distance(points[i - 1].position, points[i].position);
    return length;
}

TEST_CASE("paths cross a segment connection on the link with the exit at the same t, never shorter than the best crossing") {
    NavMesh nav_mesh = create_ledge();
    uint32_t id = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_link(OFF_MESH_CONNECTION_DIRECTION_START_TO_END));
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    const OffMeshConnectionSegment &segment = nav_mesh.off_mesh_connections[id].segment;

    struct Case {
        Vec3 start;
        Vec3 end;
    };
    const Case cases[] = {{{2, 2, 2}, {8, 0, 9}}, {{8, 2, 3}, {2, 0, 6}}, {{5, 2, 0.5f}, {9, 0, 9.5f}}, {{1.5f, 2, 3.5f}, {8.5f, 0, 5.5f}}};
    for (const Case &route : cases) {
        INFO("start ", route.start.x, " end ", route.end.x);
        StraightPathPoint points[32];
        FindPathResult path = {};
        find_path::find_path(path, workspace, nav_mesh, route.start, route.end, {0.5f, 0.5f, 0.5f}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
        REQUIRE(path.success);
        OffMeshCorner corner = find_off_mesh_corner(points, path.point_count);
        REQUIRE(corner.found);

        float best = 1e30f;
        for (int k = 0; k <= 4000; k++) {
            float t = (float)k / 4000;
            if (t < 0.2f / 8 || t > 1 - 0.2f / 8) continue;
            Vec3 a = vec3::lerp(segment.start_a, segment.start_b, t);
            Vec3 b = vec3::lerp(segment.end_a, segment.end_b, t);
            float length = vec3::distance(points[0].position, a) + vec3::distance(a, b) + vec3::distance(b, points[path.point_count - 1].position);
            if (length < best) best = length;
        }
        float t = closest_t(corner.entry.position, segment.start_a, segment.start_b);
        CHECK(vec3::distance(corner.entry.position, vec3::lerp(segment.start_a, segment.start_b, t)) < 1e-3f);
        CHECK(vec3::distance(corner.exit.position, vec3::lerp(segment.end_a, segment.end_b, t)) < 1e-3f);
        CHECK(path_length(points, path.point_count) >= best - 1e-3f);
    }

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}
