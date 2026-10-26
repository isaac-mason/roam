#include <math.h>

#include "app/examples.h"
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

enum NavMeshAreaType : uint8_t {
    NAV_MESH_AREA_TYPE_A = 1,
    NAV_MESH_AREA_TYPE_B = 2,
    NAV_MESH_AREA_TYPE_C = 3,
    NAV_MESH_AREA_TYPE_D = 4,
    NAV_MESH_AREA_TYPE_E = 5,
};

static const Box3 WATER_BOX = {{5, -1, -8}, {11, 1, -3}};

static const Vec3 GRASS_CYLINDER_CENTER = {9, 1.5f, 3.5f};
static const float GRASS_CYLINDER_RADIUS = 0.5f;
static const float GRASS_CYLINDER_HEIGHT = 2;

static const float ROAD_VERTS[] = {
    -2, 0, 5, 2, 0, 5, 3, 0, 3, 0, 0, 2, -3, 0, 3,
};
static const float ROAD_MIN_Y = -0.5f;
static const float ROAD_MAX_Y = 1.5f;

static const Vec3 ICE_RINK_CENTER = {-3, 0, -3};
static const Vec3 ICE_RINK_HALF_EXTENTS = {1, 1, 4};
static const float ICE_RINK_ROTATION = 40.0f * scalar::PI / 180.0f;

struct MarkCompactHeightfieldAreasExample {
    bool generated;
    SoloNavMeshResult result;
    bool show_nav_mesh;
    bool show_heightfield;
    bool show_compact_heightfield;
    bool show_area_markers;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing heightfield_drawing;
    StaticDrawing compact_heightfield_drawing;
    StaticDrawing area_markers_drawing;
    float previous_fov_y;
};

static MarkCompactHeightfieldAreasExample mark_areas;

static SoloNavMeshResult generate_nav_mesh(const InputTriangleMesh &input, const SoloNavMeshOptions &options) {
    SoloNavMeshResult result = {};
    SoloNavMeshIntermediates &intermediates = result.intermediates;
    BuildContext &context = intermediates.build_context;
    intermediates.input = input;

    intermediates.tri_area_ids.resize(input.indices.length / 3, 0);
    input_triangle_mesh::mark_walkable_triangles(intermediates.tri_area_ids, input, options.walkable_slope_angle_degrees);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(input);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, options.cell_size);
    intermediates.heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, options.cell_size, options.cell_height);
    Heightfield &solid = intermediates.heightfield;

    heightfield::rasterize_triangles(context, solid, input, intermediates.tri_area_ids, options.walkable_climb_voxels);

    heightfield::filter_low_hanging_walkable_obstacles(solid, options.walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, options.walkable_height_voxels);

    intermediates.compact_heightfield = compact_heightfield::build(context, solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    CompactHeightfield &compact = intermediates.compact_heightfield;

    compact_heightfield::mark_box_area(compact, WATER_BOX, NAV_MESH_AREA_TYPE_B);
    compact_heightfield::mark_cylinder_area(compact, GRASS_CYLINDER_CENTER, GRASS_CYLINDER_RADIUS, GRASS_CYLINDER_HEIGHT, NAV_MESH_AREA_TYPE_C);
    compact_heightfield::mark_convex_poly_area(compact, ROAD_VERTS, ROAD_MIN_Y, ROAD_MAX_Y, NAV_MESH_AREA_TYPE_D);
    compact_heightfield::mark_rotated_box_area(compact, ICE_RINK_CENTER, ICE_RINK_HALF_EXTENTS, ICE_RINK_ROTATION, NAV_MESH_AREA_TYPE_E);

    compact_heightfield::erode_walkable_area(compact, options.walkable_radius_voxels);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    intermediates.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    intermediates.poly_mesh = poly_mesh::build_poly_mesh(context, intermediates.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = intermediates.poly_mesh;

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

static StaticDrawing lifted_drawing(DebugPrimitives primitives, float height) {
    app::lift(primitives, height);
    StaticDrawing drawing = renderer::create_static(primitives);
    debug::release(primitives);
    return drawing;
}

static void rotated_box(DrawList &list, Vec3 center, Vec3 half, float rotation, Color color) {
    float c = cosf(rotation);
    float s = sinf(rotation);
    Vec3 corners[8];
    for (int i = 0; i < 8; i++) {
        float x = (i & 1) ? half.x : -half.x;
        float y = (i & 2) ? half.y : -half.y;
        float z = (i & 4) ? half.z : -half.z;
        corners[i] = {center.x + x * c + z * s, center.y + y, center.z - x * s + z * c};
    }
    const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
    for (const int *face : faces) {
        renderer::triangle(list, corners[face[0]], corners[face[1]], corners[face[2]], color);
        renderer::triangle(list, corners[face[0]], corners[face[2]], corners[face[3]], color);
    }
}

static StaticDrawing create_area_markers_drawing() {
    DrawList list = {};

    Vec3 water_center = vec3::scale(vec3::add(WATER_BOX.min, WATER_BOX.max), 0.5f);
    Vec3 water_half = vec3::scale(vec3::subtract(WATER_BOX.max, WATER_BOX.min), 0.5f);
    renderer::box(list, water_center, water_half, {0x15 / 255.0f, 0x65 / 255.0f, 0xc0 / 255.0f, 0.35f});

    renderer::cylinder(list, GRASS_CYLINDER_CENTER, GRASS_CYLINDER_RADIUS, GRASS_CYLINDER_HEIGHT, {0x4c / 255.0f, 0xaf / 255.0f, 0x50 / 255.0f, 0.35f});

    uint32_t road_vertex_count = sizeof(ROAD_VERTS) / sizeof(ROAD_VERTS[0]) / 3;
    for (uint32_t i = 0; i < road_vertex_count; i++) {
        Vec3 a = vec3::load(ROAD_VERTS, i * 3);
        Vec3 b = vec3::load(ROAD_VERTS, ((i + 1) % road_vertex_count) * 3);
        renderer::line(list, {a.x, a.y + 0.05f, a.z}, {b.x, b.y + 0.05f, b.z}, {1, 0.65f, 0, 1});
    }

    rotated_box(list, ICE_RINK_CENTER, ICE_RINK_HALF_EXTENTS, ICE_RINK_ROTATION, {1, 0, 0x99 / 255.0f, 0.35f});

    StaticDrawing drawing = renderer::create_static(list);
    renderer::release(list);
    return drawing;
}

static void init(App &app) {
    MarkCompactHeightfieldAreasExample &state = mark_areas;
    state = {};
    state.show_nav_mesh = true;
    state.show_area_markers = true;

    app::load_level(app, "nav-test.glb");
    mark_areas.previous_fov_y = app.camera.fov_y;
    app.camera.fov_y = 75.0f * scalar::PI / 180.0f;
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    const float cell_size = 0.2f;
    const float cell_height = 0.15f;
    const float walkable_radius_world = 0.1f;
    const float walkable_climb_world = 0.5f;
    const float walkable_height_world = 0.1f;
    const float detail_sample_distance_voxels = 6;
    const float detail_sample_max_error_voxels = 1;

    SoloNavMeshOptions options = {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_radius_voxels = (int)ceilf(walkable_radius_world / cell_size),
        .walkable_radius_world = walkable_radius_world,
        .walkable_climb_voxels = (int)ceilf(walkable_climb_world / cell_height),
        .walkable_climb_world = walkable_climb_world,
        .walkable_height_voxels = (int)ceilf(walkable_height_world / cell_height),
        .walkable_height_world = walkable_height_world,
        .walkable_slope_angle_degrees = 45,
        .border_size = 4,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 5,
        .detail_sample_distance = detail_sample_distance_voxels < 0.9f ? 0 : cell_size * detail_sample_distance_voxels,
        .detail_sample_max_error = cell_height * detail_sample_max_error_voxels,
    };

    const Model &level = app.level;
    state.result = generate_nav_mesh({level.walkable_positions, level.walkable_indices}, options);
    state.generated = true;

    state.nav_mesh_drawing = lifted_drawing(debug::create_nav_mesh_helper(state.result.nav_mesh), 0.1f);
    state.heightfield_drawing = lifted_drawing(debug::create_heightfield_helper(state.result.intermediates.heightfield), 0.05f);
    state.compact_heightfield_drawing =
        lifted_drawing(debug::create_compact_heightfield_solid_helper(state.result.intermediates.compact_heightfield), 0.1f);
    state.area_markers_drawing = create_area_markers_drawing();
}

static void update(App &app, float) {
    MarkCompactHeightfieldAreasExample &state = mark_areas;
    if (state.show_nav_mesh) app::draw_static(app, state.nav_mesh_drawing);
    if (state.show_heightfield) app::draw_static(app, state.heightfield_drawing);
    if (state.show_compact_heightfield) app::draw_static(app, state.compact_heightfield_drawing);
    if (state.show_area_markers) app::draw_static(app, state.area_markers_drawing);
}

static void ui(App &) {
    MarkCompactHeightfieldAreasExample &state = mark_areas;
    ImGui::TextWrapped("areas marked with box, cylinder, convex poly and rotated box volumes");
    if (state.generated) ImGui::Text("%d polys", state.result.intermediates.poly_mesh.poly_count);
    if (ImGui::CollapsingHeader("Debug Views", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("nav mesh", &state.show_nav_mesh);
        ImGui::Checkbox("heightfield", &state.show_heightfield);
        ImGui::Checkbox("compact heightfield", &state.show_compact_heightfield);
        ImGui::Checkbox("area markers", &state.show_area_markers);
    }
}

static void shutdown(App &app) {
    MarkCompactHeightfieldAreasExample &state = mark_areas;
    renderer::release(state.nav_mesh_drawing);
    renderer::release(state.heightfield_drawing);
    renderer::release(state.compact_heightfield_drawing);
    renderer::release(state.area_markers_drawing);
    if (state.generated) {
        generate_solo_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
    app.camera.fov_y = mark_areas.previous_fov_y;
}

const Example EXAMPLE_MARK_COMPACT_HEIGHTFIELD_AREAS = {"mark compact heightfield areas", init, update, ui, shutdown};
