#include "scene.h"

namespace roam {

SoloNavMeshOptions scene_options() {
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    return generate_solo_nav_mesh::derive_sizes({
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
}

Scene create_scene() {
    Scene scene = {};
    scene.level = create_level();
    const InputTriangleMesh mesh = {scene.level.positions, scene.level.indices};
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(mesh, scene_options());
    generate_solo_nav_mesh::release(result.intermediates);
    scene.nav_mesh = result.nav_mesh;
    scene.workspace = nav_mesh_search::create_query_workspace();
    scene.filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    return scene;
}

void release(Scene &scene) {
    nav_mesh_search::release(scene.workspace);
    nav_mesh_api::release(scene.nav_mesh);
    scene.level.positions.release();
    scene.level.indices.release();
}

}
