/* SNIPPET_START: quickstart */
#include <stdio.h>

#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/query/find_path.h"

using namespace roam;

int main() {
    // your level geometry: leave out what shouldn't contribute to walkable surfaces, like foliage
    // or small decorative props. here, a 20 m floor with a 4 m wide, 2 m tall block in the middle,
    // triangles wound counter clockwise from above
    const float positions[] = {
        -10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10,
        -2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2,
        -2, 2, -2, 2, 2, -2, 2, 2, 2, -2, 2, 2,
    };
    const uint32_t indices[] = {
        0, 2, 1, 0, 3, 2,
        8, 10, 9, 8, 11, 10,
        4, 9, 5, 4, 8, 9, 5, 10, 6, 5, 9, 10,
        6, 11, 7, 6, 10, 11, 7, 8, 4, 7, 11, 8,
    };
    const InputTriangleMesh level = {positions, indices};

    // defaults for a human-sized agent in a 1 m scale world: voxels of 0.2 m, for an agent 0.3 m
    // in radius and 1.8 m tall, which can climb 0.4 m and walk up 45 degree slopes. use the debug
    // helpers to see each stage of generation and fine tune these
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    const SoloNavMeshOptions options = generate_solo_nav_mesh::derive_sizes({
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_radius_world = 0.3f,
        .walkable_climb_world = 0.4f,
        .walkable_height_world = 1.8f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 6 * cell_size,
        .detail_sample_max_error = 1 * cell_height,
    });

    // generate the nav mesh. every stage is kept in the intermediates, for debugging: let them go
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(level, options);
    generate_solo_nav_mesh::release(result.intermediates);
    NavMesh &nav_mesh = result.nav_mesh;

    // find a path: the start and end are snapped to the nav mesh within the half extents, and
    // the path's corners are written into the points buffer
    const Vec3 start = {-8, 0, 0};
    const Vec3 end = {8, 0, 0};
    const Vec3 half_extents = {1, 1, 1};
    const QueryFilter &filter = nav_mesh_api::DEFAULT_QUERY_FILTER;

    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    StraightPathPoint points[64];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, start, end, half_extents, filter, points);

    printf("%s, %u points\n", path.success ? "found a path" : "no path", path.point_count);
    for (uint32_t i = 0; i < path.point_count; i++) {
        const Vec3 point = points[i].position;
        printf("  %.2f %.2f %.2f\n", point.x, point.y, point.z);
    }

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
    return 0;
}
/* SNIPPET_END: quickstart */
