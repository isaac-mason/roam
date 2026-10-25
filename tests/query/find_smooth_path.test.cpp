#include "roam/query/find_smooth_path.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

TEST_CASE("find smooth path takes off mesh connection from its start") {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 7, 0, 0, 9, 0, 0, 9, 0, 2, 7, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    NavMesh nav_mesh = create_nav_mesh_from_triangles(positions, 8, indices, 12);

    nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1, 0, 1}, {8, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});

    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    SmoothPathPoint points[256];
    FindSmoothPathResult result;
    find_smooth_path::find_smooth_path(result, workspace, nav_mesh, {1, 0, 1}, {8.5f, 0, 1.5f}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points, {.step_size = 0.5f, .slop = 0.01f});

    REQUIRE(result.success);
    REQUIRE(result.point_count > 1);

    bool took_off_mesh_connection = false;
    for (uint32_t i = 0; i < result.point_count; i++) took_off_mesh_connection |= points[i].type == NODE_TYPE_OFFMESH;
    CHECK(took_off_mesh_connection);

    const SmoothPathPoint &last = points[result.point_count - 1];
    CHECK((last.flags & STRAIGHT_PATH_POINT_END) != 0);
    CHECK_NEAR(last.position.x, 8.5f, 1e-4f);
    CHECK_NEAR(last.position.z, 1.5f, 1e-4f);

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
}
