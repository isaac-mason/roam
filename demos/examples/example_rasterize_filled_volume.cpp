#include <math.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/math/scalar.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum FilledVolumeHelper {
    FILLED_VOLUME_HELPER_TRIANGLE_AREA_IDS,
    FILLED_VOLUME_HELPER_HEIGHTFIELD,
    FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_SOLID,
    FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_DISTANCES,
    FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_REGIONS,
    FILLED_VOLUME_HELPER_RAW_CONTOURS,
    FILLED_VOLUME_HELPER_SIMPLIFIED_CONTOURS,
    FILLED_VOLUME_HELPER_POLY_MESH,
    FILLED_VOLUME_HELPER_POLY_MESH_DETAIL,
    FILLED_VOLUME_HELPER_NAV_MESH_BV_TREE,
    FILLED_VOLUME_HELPER_NAV_MESH,
    FILLED_VOLUME_HELPER_NAV_MESH_LINKS,
    FILLED_VOLUME_HELPER_NAV_MESH_ISLANDS,
    FILLED_VOLUME_HELPER_COUNT,
};

static const char *const FILLED_VOLUME_HELPER_NAMES[FILLED_VOLUME_HELPER_COUNT] = {
    "triangle area ids", "heightfield",    "compact heightfield solid", "compact heightfield distances",
    "compact heightfield regions", "raw contours", "simplified contours", "poly mesh",
    "poly mesh detail", "nav mesh bv tree", "nav mesh", "nav mesh links", "nav mesh islands",
};

enum VolumeShape {
    VOLUME_SHAPE_CONVEX,
    VOLUME_SHAPE_CAPSULE,
};

struct Volume {
    VolumeShape shape;
    uint8_t area;
    Vec3 position;
    Vec3 rotation;
    Color color;
};

enum VolumeIndex {
    VOLUME_WALKABLE_DODECAHEDRON,
    VOLUME_NULL_CAPSULE,
    VOLUME_COUNT,
};

static const float GOLDEN_RATIO = 1.6180339887498949f;
static const float INVERSE_GOLDEN_RATIO = 1 / GOLDEN_RATIO;

static const float DODECAHEDRON_VERTICES[] = {
    -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1,
    0, -INVERSE_GOLDEN_RATIO, -GOLDEN_RATIO, 0, -INVERSE_GOLDEN_RATIO, GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO, -GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO, GOLDEN_RATIO,
    -INVERSE_GOLDEN_RATIO, -GOLDEN_RATIO, 0, -INVERSE_GOLDEN_RATIO, GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO, -GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO, GOLDEN_RATIO, 0,
    -GOLDEN_RATIO, 0, -INVERSE_GOLDEN_RATIO, GOLDEN_RATIO, 0, -INVERSE_GOLDEN_RATIO, -GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO, GOLDEN_RATIO, 0, INVERSE_GOLDEN_RATIO,
};

static const uint32_t DODECAHEDRON_INDICES[] = {
    3, 11, 7, 3, 7, 15, 3, 15, 13, 7, 19, 17, 7, 17, 6, 7, 6, 15, 17, 4, 8, 17, 8, 10, 17, 10, 6,
    8, 0, 16, 8, 16, 2, 8, 2, 10, 0, 12, 1, 0, 1, 18, 0, 18, 16, 6, 10, 2, 6, 2, 13, 6, 13, 15,
    2, 16, 18, 2, 18, 3, 2, 3, 13, 18, 1, 9, 18, 9, 11, 18, 11, 3, 4, 14, 12, 4, 12, 0, 4, 0, 8,
    11, 9, 5, 11, 5, 19, 11, 19, 7, 19, 5, 14, 19, 14, 4, 19, 4, 17, 1, 12, 14, 1, 14, 5, 1, 5, 9,
};

static const float CAPSULE_RADIUS = 1;
static const float CAPSULE_HEIGHT = 1;

struct RasterizeFilledVolumeExample {
    NavMeshConfig config;
    bool generated;
    SoloNavMeshResult result;
    double generation_milliseconds;
    Volume volumes[VOLUME_COUNT];
    int selected_volume;
    bool show[FILLED_VOLUME_HELPER_COUNT];
    StaticDrawing drawings[FILLED_VOLUME_HELPER_COUNT];
    float previous_fov_y;
};

static RasterizeFilledVolumeExample filled_volume;

static Vec3 rotate_euler_xyz(Vec3 v, Vec3 rotation) {
    float cx = cosf(rotation.x), sx = sinf(rotation.x);
    float cy = cosf(rotation.y), sy = sinf(rotation.y);
    float cz = cosf(rotation.z), sz = sinf(rotation.z);
    Vec3 a = {v.x * cz - v.y * sz, v.x * sz + v.y * cz, v.z};
    Vec3 b = {a.x * cy + a.z * sy, a.y, -a.x * sy + a.z * cy};
    return {b.x, b.y * cx - b.z * sx, b.y * sx + b.z * cx};
}

static Vec3 volume_to_world(const Volume &volume, Vec3 local) { return vec3::add(rotate_euler_xyz(local, volume.rotation), volume.position); }

static Array<float> dodecahedron_world_vertices(const Volume &volume) {
    Array<float> vertices = {};
    uint32_t vertex_count = sizeof(DODECAHEDRON_VERTICES) / sizeof(DODECAHEDRON_VERTICES[0]) / 3;
    for (uint32_t i = 0; i < vertex_count; i++) {
        Vec3 world = volume_to_world(volume, vec3::normalize(vec3::load(DODECAHEDRON_VERTICES, i * 3)));
        vertices.push(world.x);
        vertices.push(world.y);
        vertices.push(world.z);
    }
    return vertices;
}

static void rasterize_volume(Heightfield &heightfield, const Volume &volume) {
    switch (volume.shape) {
        case VOLUME_SHAPE_CAPSULE: {
            float half_height = CAPSULE_HEIGHT / 2;
            Vec3 start = volume_to_world(volume, {0, half_height, 0});
            Vec3 end = volume_to_world(volume, {0, -half_height, 0});
            heightfield::rasterize_capsule(heightfield, start, end, CAPSULE_RADIUS, volume.area, 1);
            break;
        }
        case VOLUME_SHAPE_CONVEX: {
            Array<float> vertices = dodecahedron_world_vertices(volume);
            heightfield::rasterize_convex(heightfield, {vertices, DODECAHEDRON_INDICES}, volume.area, 1);
            vertices.release();
            break;
        }
    }
}

static SoloNavMeshResult generate_nav_mesh(const InputTriangleMesh &input, const Volume *volumes, uint32_t volume_count,
                                           const SoloNavMeshOptions &options) {
    SoloNavMeshResult result = {};
    SoloNavMeshIntermediates &intermediates = result.intermediates;
    BuildContext &context = intermediates.build_context;
    intermediates.input = input;

    intermediates.tri_area_ids.resize(input.indices.capacity / 3, 0);
    input_triangle_mesh::mark_walkable_triangles(intermediates.tri_area_ids, input, options.walkable_slope_angle_degrees);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(input);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, options.cell_size);
    intermediates.heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, options.cell_size, options.cell_height);
    Heightfield &solid = intermediates.heightfield;

    heightfield::rasterize_triangles(context, solid, input, intermediates.tri_area_ids, options.walkable_climb_voxels);

    for (uint32_t i = 0; i < volume_count; i++) rasterize_volume(solid, volumes[i]);

    heightfield::filter_low_hanging_walkable_obstacles(solid, options.walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, options.walkable_height_voxels);

    intermediates.compact_heightfield = compact_heightfield::build(context, solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    CompactHeightfield &compact = intermediates.compact_heightfield;

    compact_heightfield::erode_walkable_area(compact, options.walkable_radius_voxels);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    intermediates.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    intermediates.poly_mesh = poly_mesh::build_poly_mesh(context, intermediates.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = intermediates.poly_mesh;

    for (int poly_index = 0; poly_index < polys.poly_count; poly_index++) {
        if (polys.areas[(uint32_t)poly_index] == WALKABLE_AREA) polys.areas[(uint32_t)poly_index] = 0;
        if (polys.areas[(uint32_t)poly_index] == 0) polys.flags[(uint32_t)poly_index] = 1;
    }

    intermediates.poly_mesh_detail =
        poly_mesh_detail::build_poly_mesh_detail(context, polys, compact, options.detail_sample_distance, options.detail_sample_max_error);

    NavMesh &nav_mesh = result.nav_mesh;
    nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.tile_width = polys.bounds.max.x - polys.bounds.min.x;
    nav_mesh.tile_height = polys.bounds.max.z - polys.bounds.min.z;
    nav_mesh.origin = polys.bounds.min;

    NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
    NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, intermediates.poly_mesh_detail);

    NavMeshTileParams tile_params = {
        .bounds = polys.bounds,
        .vertices = tile_polys.vertices,
        .polys = tile_polys.polys,
        .poly_vertices = tile_polys.poly_vertices,
        .poly_neis = tile_polys.poly_neis,
        .detail_meshes = tile_detail_mesh.detail_meshes,
        .detail_vertices = tile_detail_mesh.detail_vertices,
        .detail_triangles = tile_detail_mesh.detail_triangles,
        .tile_x = 0,
        .tile_y = 0,
        .tile_layer = 0,
        .cell_size = options.cell_size,
        .cell_height = options.cell_height,
        .walkable_height = options.walkable_height_world,
        .walkable_radius = options.walkable_radius_world,
        .walkable_climb = options.walkable_climb_world,
    };

    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(tile_params));

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail_mesh);
    return result;
}

static DebugPrimitives create_helper(int helper) {
    const SoloNavMeshIntermediates &intermediates = filled_volume.result.intermediates;
    const NavMesh &nav_mesh = filled_volume.result.nav_mesh;
    switch (helper) {
        case FILLED_VOLUME_HELPER_TRIANGLE_AREA_IDS:
            return debug::create_triangle_area_ids_helper(intermediates.input, intermediates.tri_area_ids);
        case FILLED_VOLUME_HELPER_HEIGHTFIELD: return debug::create_heightfield_helper(intermediates.heightfield);
        case FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_SOLID: return debug::create_compact_heightfield_solid_helper(intermediates.compact_heightfield);
        case FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_DISTANCES:
            return debug::create_compact_heightfield_distances_helper(intermediates.compact_heightfield);
        case FILLED_VOLUME_HELPER_COMPACT_HEIGHTFIELD_REGIONS:
            return debug::create_compact_heightfield_regions_helper(intermediates.compact_heightfield);
        case FILLED_VOLUME_HELPER_RAW_CONTOURS: return debug::create_raw_contours_helper(intermediates.contour_set);
        case FILLED_VOLUME_HELPER_SIMPLIFIED_CONTOURS: return debug::create_simplified_contours_helper(intermediates.contour_set);
        case FILLED_VOLUME_HELPER_POLY_MESH: return debug::create_poly_mesh_helper(intermediates.poly_mesh);
        case FILLED_VOLUME_HELPER_POLY_MESH_DETAIL: return debug::create_poly_mesh_detail_helper(intermediates.poly_mesh_detail);
        case FILLED_VOLUME_HELPER_NAV_MESH_BV_TREE: return debug::create_nav_mesh_bv_tree_helper(nav_mesh);
        case FILLED_VOLUME_HELPER_NAV_MESH: {
            DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
        case FILLED_VOLUME_HELPER_NAV_MESH_LINKS: return debug::create_nav_mesh_links_helper(nav_mesh);
        case FILLED_VOLUME_HELPER_NAV_MESH_ISLANDS: {
            DebugPrimitives primitives = debug::create_nav_mesh_islands_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
    }
    return {};
}

static void update_drawing(int helper) {
    renderer::release(filled_volume.drawings[helper]);
    if (!filled_volume.generated || !filled_volume.show[helper]) return;
    DebugPrimitives primitives = create_helper(helper);
    filled_volume.drawings[helper] = renderer::create_static(primitives);
    debug::release(primitives);
}

static void release_result() {
    if (!filled_volume.generated) return;
    generate_solo_nav_mesh::release(filled_volume.result.intermediates);
    nav_mesh_api::release(filled_volume.result.nav_mesh);
    filled_volume.generated = false;
}

static void generate(App &app) {
    RasterizeFilledVolumeExample &state = filled_volume;
    release_result();

    SoloNavMeshOptions options = nav_mesh_config::solo_options(state.config);

    Volume volumes[VOLUME_COUNT] = {state.volumes[VOLUME_NULL_CAPSULE], state.volumes[VOLUME_WALKABLE_DODECAHEDRON]};

    const Model &level = app.level;
    double start = app::now_milliseconds();
    state.result = generate_nav_mesh({level.walkable_positions, level.walkable_indices}, volumes,
                                     VOLUME_COUNT, options);
    state.generation_milliseconds = app::now_milliseconds() - start;
    state.generated = true;
    for (int i = 0; i < FILLED_VOLUME_HELPER_COUNT; i++) update_drawing(i);
}

static void draw_capsule(DrawList &list, const Volume &volume) {
    const int cap_segments = 4;
    const int radial_segments = 8;
    const int vertical_segments = cap_segments * 2 + 1;
    float half_height = CAPSULE_HEIGHT / 2;
    auto at = [&](int row, int column) {
        float profile_y;
        float profile_radius;
        if (row <= cap_segments) {
            float angle = (float)row / cap_segments * scalar::PI / 2;
            profile_y = -half_height - CAPSULE_RADIUS * cosf(angle);
            profile_radius = CAPSULE_RADIUS * sinf(angle);
        } else if (row <= cap_segments + 1) {
            profile_y = half_height;
            profile_radius = CAPSULE_RADIUS;
        } else {
            float angle = (float)(row - cap_segments - 1) / cap_segments * scalar::PI / 2;
            profile_y = half_height + CAPSULE_RADIUS * sinf(angle);
            profile_radius = CAPSULE_RADIUS * cosf(angle);
        }
        float theta = (float)column / radial_segments * scalar::PI * 2;
        return volume_to_world(volume, {-profile_radius * cosf(theta), profile_y, profile_radius * sinf(theta)});
    };
    for (int row = 1; row <= vertical_segments; row++) {
        for (int column = 0; column < radial_segments; column++) {
            renderer::triangle(list, at(row - 1, column), at(row - 1, column + 1), at(row, column), volume.color);
            renderer::triangle(list, at(row - 1, column + 1), at(row, column + 1), at(row, column), volume.color);
        }
    }
}

static void draw_dodecahedron(DrawList &list, const Volume &volume) {
    Array<float> vertices = dodecahedron_world_vertices(volume);
    uint32_t index_count = sizeof(DODECAHEDRON_INDICES) / sizeof(DODECAHEDRON_INDICES[0]);
    for (uint32_t i = 0; i < index_count; i += 3) {
        renderer::triangle(list, vec3::load(vertices.data, DODECAHEDRON_INDICES[i] * 3), vec3::load(vertices.data, DODECAHEDRON_INDICES[i + 1] * 3),
                           vec3::load(vertices.data, DODECAHEDRON_INDICES[i + 2] * 3), volume.color);
    }
    vertices.release();
}

static void draw_dragger(DrawList &list, Vec3 position) {
    renderer::line(list, position, {position.x + 1.5f, position.y, position.z}, {1, 0.2f, 0.2f, 1});
    renderer::line(list, position, {position.x, position.y + 1.5f, position.z}, {0.6f, 0.9f, 0.4f, 1});
    renderer::line(list, position, {position.x, position.y, position.z + 1.5f}, {0.1f, 0.2f, 0.9f, 1});
}

static void init(App &app) {
    RasterizeFilledVolumeExample &state = filled_volume;
    state = {};
    state.config = nav_mesh_config::create();
    state.show[FILLED_VOLUME_HELPER_NAV_MESH] = true;
    state.volumes[VOLUME_WALKABLE_DODECAHEDRON] = {VOLUME_SHAPE_CONVEX, WALKABLE_AREA, {-2, 0, -7}, {0, scalar::PI / 4, 0}, {0, 1, 0, 0.75f}};
    state.volumes[VOLUME_NULL_CAPSULE] = {VOLUME_SHAPE_CAPSULE, NULL_AREA, {2, 0, -7}, {0, 0, scalar::PI / 4}, {1, 0, 0, 0.75f}};

    app::load_level(app, "nav-test.glb");
    filled_volume.previous_fov_y = app.camera.fov_y;
    app.camera.fov_y = 75.0f * scalar::PI / 180.0f;
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});
    generate(app);
}

static void update(App &app, float) {
    RasterizeFilledVolumeExample &state = filled_volume;
    if (app.click.clicked && app.click.hit && !app.click.secondary) state.volumes[state.selected_volume].position = app.click.point;
    for (int i = 0; i < FILLED_VOLUME_HELPER_COUNT; i++) {
        if (state.show[i]) app::draw_static(app, state.drawings[i]);
    }
    DrawList &list = app::frame(app);
    draw_dodecahedron(list, state.volumes[VOLUME_WALKABLE_DODECAHEDRON]);
    draw_capsule(list, state.volumes[VOLUME_NULL_CAPSULE]);
    for (const Volume &volume : state.volumes) draw_dragger(list, volume.position);
}

static void ui(App &app) {
    RasterizeFilledVolumeExample &state = filled_volume;
    ImGui::TextWrapped("click the level to move the selected volume, then generate the nav mesh");
    ImGui::Text("generated in %.2f ms", state.generation_milliseconds);
    if (state.generated) ImGui::Text("%d polys", state.result.intermediates.poly_mesh.poly_count);
    if (ImGui::Button("generate nav mesh")) generate(app);
    if (ImGui::CollapsingHeader("Volumes", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::RadioButton("walkable dodecahedron", &state.selected_volume, VOLUME_WALKABLE_DODECAHEDRON);
        ImGui::DragFloat3("##dodecahedron", &state.volumes[VOLUME_WALKABLE_DODECAHEDRON].position.x, 0.05f);
        ImGui::RadioButton("null capsule", &state.selected_volume, VOLUME_NULL_CAPSULE);
        ImGui::DragFloat3("##capsule", &state.volumes[VOLUME_NULL_CAPSULE].position.x, 0.05f);
    }
    nav_mesh_config::ui(state.config, false);
    if (ImGui::CollapsingHeader("Debug Helpers", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("show mesh", &app.show_level);
        for (int i = 0; i < FILLED_VOLUME_HELPER_COUNT; i++) {
            if (ImGui::Checkbox(FILLED_VOLUME_HELPER_NAMES[i], &state.show[i])) update_drawing(i);
        }
    }
}

static void shutdown(App &app) {
    for (StaticDrawing &drawing : filled_volume.drawings) renderer::release(drawing);
    release_result();
    app.show_level = true;
    app.camera.fov_y = filled_volume.previous_fov_y;
}

const Example EXAMPLE_RASTERIZE_FILLED_VOLUME = {"rasterize filled volume", init, update, ui, shutdown};
