#include "roam/query/nav_mesh_search.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

static NavMesh create_simple_quad_nav_mesh() {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3};
    return create_nav_mesh_from_triangles(positions, 4, indices, 6);
}

static NavMesh create_long_corridor_nav_mesh() {
    Array<float> positions = {};
    Array<uint32_t> indices = {};
    for (int i = 0; i <= 10; i++) {
        float x = (float)i;
        positions.push(x);
        positions.push(0);
        positions.push(0);
        positions.push(x);
        positions.push(0);
        positions.push(1);
    }
    for (uint32_t i = 0; i < 10; i++) {
        uint32_t base = i * 2;
        const uint32_t triangles[] = {base, base + 2, base + 3, base, base + 3, base + 1};
        for (uint32_t index : triangles) indices.push(index);
    }
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions.data, positions.length / 3, indices.data, indices.length);
    positions.release();
    indices.release();
    return nav_mesh;
}

static const uint32_t MAX_VISITED = 16;

static MoveAlongSurfaceResult move(const NavMesh &nav_mesh, Vec3 start, Vec3 end, Span<NodeRef> visited) {
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    MoveAlongSurfaceResult result;
    nav_mesh_search::move_along_surface(result, workspace, nav_mesh, nav_mesh.nodes[0].ref, start, end, nav_mesh_api::DEFAULT_QUERY_FILTER, visited);
    nav_mesh_search::release(workspace);
    return result;
}

TEST_CASE("move along surface within single polygon") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {0.5f, 0, 0.5f}, {0.8f, 0, 0.8f}, visited);
    CHECK(result.success);
    REQUIRE(result.visited_count == 1);
    CHECK(visited[0] == nav_mesh.nodes[0].ref);
    CHECK(vec3::distance(result.position, {0.8f, 0, 0.8f}) < 0.01f);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface across two adjacent triangles") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.5f, 0, 0.5f}, {0.5f, 0, 1.5f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count == 2);
    CHECK(vec3::distance(result.position, {0.5f, 0, 1.5f}) < 0.1f);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface small movement within polygon") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.0f, 0, 1.0f}, {1.015f, 0, 1.0f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count >= 1);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface across long thin corridor") {
    NavMesh nav_mesh = create_long_corridor_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {0.5f, 0, 0.5f}, {9.5f, 0, 0.5f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count > 1);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface small movement crossing polygon boundary") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.0f, 0, 0.5f}, {1.0f, 0, 1.5f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count == 2);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface very small movement crossing polygon boundary") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.0f, 0, 0.9925f}, {1.0f, 0, 1.0075f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count == 2);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface stops at wall when target is outside") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.0f, 0, 1.0f}, {5.0f, 0, 1.0f}, visited);
    CHECK(result.success);
    CHECK(result.position.x <= 2.0f);
    CHECK(result.position.x > 1.9f);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface snaps to nearest wall when start is outside start polygon") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.0f, 0, 1.5f}, {1.0f, 0, 1.52f}, visited);
    CHECK(result.success);
    CHECK(result.visited_count == 1);
    CHECK(vec3::distance({1.0f, 0, 1.5f}, result.position) > 0.9f);
    nav_mesh_api::release(nav_mesh);
}

TEST_CASE("move along surface stops at wall edge matching off mesh link edge") {
    NavMesh nav_mesh = create_simple_quad_nav_mesh();
    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1.5f, 0, 0.4f}, {0.5f, 0, 1.8f}, 0.1f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 0});

    NodeRef visited[MAX_VISITED];
    MoveAlongSurfaceResult result = move(nav_mesh, {1.5f, 0, 0.5f}, {1.5f, 0, -3.0f}, visited);
    CHECK(result.success);
    REQUIRE(result.visited_count == 1);
    CHECK(visited[0] == nav_mesh.nodes[0].ref);
    CHECK_NEAR(result.position.x, 1.5, 1e-2);
    CHECK_NEAR(result.position.z, 0, 1e-2);
    nav_mesh_api::release(nav_mesh);
}
