#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/contour_set.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/generate/poly_mesh.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum AreaCostsAreaType : uint8_t {
    AREA_COSTS_GROUND = 1,
    AREA_COSTS_RED = 2,
    AREA_COSTS_GREEN = 3,
};

static const Box3 RED_ZONE_BOUNDS = {{-8, -2, -2}, {2, 2, 12}};
static const Box3 GREEN_ENTRY_BOUNDS = {{2, -2, -12}, {14, 2, -2}};
static const Box3 GREEN_LANE_BOUNDS = {{8, -2, -2}, {14, 2, 12}};
static const Box3 GREEN_EXIT_BOUNDS = {{-12, -2, 12}, {14, 2, 20}};

static const Color RED_COLOR = {1, 0.231f, 0.188f, 1};
static const Color GREEN_COLOR = {0, 1, 0.416f, 1};
static const Color FLOOR_COLOR = {0.114f, 0.306f, 0.847f, 1};

static const float FLOOR_SIZE = 40;

struct AreaCostConfig {
    float ground;
    float red;
    float green;
};

enum AreaCostsMoving {
    AREA_COSTS_MOVING_NONE,
    AREA_COSTS_MOVING_START,
    AREA_COSTS_MOVING_END,
};

struct AreaCostsExample {
    bool generated;
    NavMesh nav_mesh;
    StaticDrawing overlay_drawing;
    StaticDrawing path_drawing;
    AreaCostConfig costs;
    QueryFilter filter;
    Vec3 start;
    Vec3 end;
    AreaCostsMoving moving;
    bool success;
    uint32_t point_count;
    QueryWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
};

static AreaCostsExample area_costs_example;

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static Model create_floor() {
    Model model = {};
    model.bounds = box3::empty();
    const float half = FLOOR_SIZE * 0.5f;
    const Vec3 corners[4] = {{-half, 0, -half}, {half, 0, -half}, {-half, 0, half}, {half, 0, half}};
    for (Vec3 corner : corners) {
        model.positions.push(corner.x);
        model.positions.push(corner.y);
        model.positions.push(corner.z);
        model.colors.push(FLOOR_COLOR.r);
        model.colors.push(FLOOR_COLOR.g);
        model.colors.push(FLOOR_COLOR.b);
        model.walkable_positions.push(corner.x);
        model.walkable_positions.push(corner.y);
        model.walkable_positions.push(corner.z);
        model.bounds = box3::expand_by_point(model.bounds, corner);
    }
    const uint32_t indices[6] = {0, 2, 1, 2, 3, 1};
    for (uint32_t index : indices) {
        model.indices.push(index);
        model.walkable_indices.push(index);
    }
    model.loaded = true;
    return model;
}

static NavMesh generate_flat_nav_mesh(const InputTriangleMesh &mesh) {
    const float cell_size = 0.5f;
    const float cell_height = 0.2f;
    const float walkable_radius_world = 0.3f;
    const float walkable_climb_world = 0.4f;
    const float walkable_height_world = 0.5f;
    const float walkable_slope_angle_degrees = 45;
    const int border_size = 4;
    const int min_region_area = 8;
    const int merge_region_area = 20;
    const float max_simplification_error = 1.3f;
    const int max_edge_length = 12;
    const int max_vertices_per_poly = 6;
    const float detail_sample_distance = 0.6f;
    const float detail_sample_max_error = 0.2f;

    const int walkable_climb_voxels = (int)ceilf(walkable_climb_world / cell_height);
    const int walkable_height_voxels = (int)ceilf(walkable_height_world / cell_height);
    const int walkable_radius_voxels = (int)ceilf(walkable_radius_world / cell_size);

    BuildContext context = {};

    Array<uint8_t> tri_area_ids = {};
    tri_area_ids.resize(mesh.indices.capacity / 3, 0);
    input_triangle_mesh::mark_walkable_triangles(tri_area_ids, mesh, walkable_slope_angle_degrees);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(mesh);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, cell_size);
    Heightfield solid = heightfield::create(grid_size.width, grid_size.height, bounds, cell_size, cell_height);

    heightfield::rasterize_triangles(context, solid, mesh, tri_area_ids, walkable_climb_voxels);

    heightfield::filter_low_hanging_walkable_obstacles(solid, walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, walkable_height_voxels, walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, walkable_height_voxels);

    CompactHeightfield compact = compact_heightfield::build(context, solid, walkable_height_voxels, walkable_climb_voxels);

    compact_heightfield::mark_box_area(compact, RED_ZONE_BOUNDS, AREA_COSTS_RED);
    compact_heightfield::mark_box_area(compact, GREEN_ENTRY_BOUNDS, AREA_COSTS_GREEN);
    compact_heightfield::mark_box_area(compact, GREEN_LANE_BOUNDS, AREA_COSTS_GREEN);
    compact_heightfield::mark_box_area(compact, GREEN_EXIT_BOUNDS, AREA_COSTS_GREEN);

    compact_heightfield::erode_walkable_area(compact, walkable_radius_voxels);

    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, border_size, min_region_area, merge_region_area);

    ContourSet contours = contour_set::build_contours(context, compact, max_simplification_error, max_edge_length, CONTOUR_TESS_WALL_EDGES);

    PolyMesh polys = poly_mesh::build_poly_mesh(context, contours, max_vertices_per_poly);

    for (uint32_t poly_index = 0; poly_index < (uint32_t)polys.poly_count; poly_index++) {
        if (polys.areas[poly_index] == WALKABLE_AREA) {
            polys.areas[poly_index] = AREA_COSTS_GROUND;
            polys.flags[poly_index] = 0x01;
        } else if (polys.areas[poly_index] == AREA_COSTS_RED) {
            polys.flags[poly_index] = 0x02;
        } else if (polys.areas[poly_index] == AREA_COSTS_GREEN) {
            polys.flags[poly_index] = 0x04;
        }
    }

    PolyMeshDetail detail = poly_mesh_detail::build_poly_mesh_detail(context, polys, compact, detail_sample_distance, detail_sample_max_error);

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.tile_width = polys.bounds.max.x - polys.bounds.min.x;
    nav_mesh.tile_height = polys.bounds.max.z - polys.bounds.min.z;
    nav_mesh.origin = polys.bounds.min;

    NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
    NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, detail);

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
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_height = walkable_height_world,
        .walkable_radius = walkable_radius_world,
        .walkable_climb = walkable_climb_world,
    };

    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(tile_params));

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail_mesh);
    poly_mesh_detail::release(detail);
    poly_mesh::release(polys);
    contour_set::release(contours);
    compact_heightfield::release(compact);
    heightfield::release(solid);
    tri_area_ids.release();
    build_context::release(context);
    return nav_mesh;
}

static bool pass_all(const QueryFilter &, NodeRef, const NavMesh &) { return true; }

static float area_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef, NodeRef current_ref, NodeRef) {
    const AreaCostConfig &costs = *(const AreaCostConfig *)filter.user_data;
    float base = vec3::distance(pa, pb);
    uint8_t area = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref).area;
    float multiplier = area == AREA_COSTS_RED ? costs.red : area == AREA_COSTS_GREEN ? costs.green : costs.ground;
    return base * multiplier;
}

static void overlay(DrawList &list, Box3 bounds, Color color) {
    const float y = 0.01f;
    Vec3 a = {bounds.min.x, y, bounds.min.z};
    Vec3 b = {bounds.max.x, y, bounds.min.z};
    Vec3 c = {bounds.max.x, y, bounds.max.z};
    Vec3 d = {bounds.min.x, y, bounds.max.z};
    renderer::triangle(list, a, c, b, color);
    renderer::triangle(list, a, d, c, color);
}

static void update_path() {
    AreaCostsExample &state = area_costs_example;
    renderer::release(state.path_drawing);

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, GREEN_COLOR);

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, state.nav_mesh, state.start, state.end, {0.6f, 1, 0.6f}, state.filter, state.points);
    state.success = path.success;
    state.point_count = path.point_count;

    for (uint32_t i = 0; i < path.point_count; i++) {
        Vec3 position = state.points[i].position;
        renderer::point(list, position, 0.2f, {1, 1, 1, 1});
        if (i > 0) renderer::line(list, state.points[i - 1].position, position, {1, 1, 1, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    AreaCostsExample &state = area_costs_example;
    state = {};
    state.workspace = nav_mesh_search::create_query_workspace();
    state.costs = {1.0f, 3.0f, 0.45f};
    state.filter = {pass_all, area_cost, 0xffffffffu, 0, &state.costs};
    state.start = {-6, 0, -10};
    state.end = {-6, 0, 18};
    app.want_hover = true;

    app::set_level(app, create_floor());
    app.camera.yaw = atan2f(-18, 22);
    app.camera.pitch = asinf(16 / sqrtf(18 * 18 + 16 * 16 + 22 * 22));

    const Model &level = app.level;
    state.nav_mesh = generate_flat_nav_mesh({level.walkable_positions, level.walkable_indices});
    state.generated = true;

    DrawList overlays = {};
    overlay(overlays, RED_ZONE_BOUNDS, RED_COLOR);
    overlay(overlays, GREEN_ENTRY_BOUNDS, GREEN_COLOR);
    overlay(overlays, GREEN_LANE_BOUNDS, GREEN_COLOR);
    overlay(overlays, GREEN_EXIT_BOUNDS, GREEN_COLOR);
    state.overlay_drawing = renderer::create_static(overlays);
    renderer::release(overlays);

    update_path();
}

static void update(App &app, float) {
    AreaCostsExample &state = area_costs_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary) {
            state.moving = state.moving == AREA_COSTS_MOVING_END ? AREA_COSTS_MOVING_NONE : AREA_COSTS_MOVING_END;
            state.end = app.click.point;
        } else {
            state.moving = state.moving == AREA_COSTS_MOVING_START ? AREA_COSTS_MOVING_NONE : AREA_COSTS_MOVING_START;
            state.start = app.click.point;
        }
        update_path();
    } else if (state.moving != AREA_COSTS_MOVING_NONE && app.hover.hit) {
        if (state.moving == AREA_COSTS_MOVING_START) {
            state.start = app.hover.point;
        } else {
            state.end = app.hover.point;
        }
        update_path();
    }
    app::draw_static(app, state.overlay_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    AreaCostsExample &state = area_costs_example;
    ImGui::TextWrapped("click to move the start, right click to move the end, click again to place it");
    if (ImGui::CollapsingHeader("Cost multipliers", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed = false;
        changed |= ImGui::SliderFloat("ground", &state.costs.ground, 0.1f, 5, "%.2f");
        changed |= ImGui::SliderFloat("red", &state.costs.red, 0.1f, 5, "%.2f");
        changed |= ImGui::SliderFloat("green", &state.costs.green, 0.1f, 5, "%.2f");
        if (changed) update_path();
    }
    ImGui::Text("%s, %u points", state.success ? "found" : "failed", state.point_count);
}

static void shutdown(App &) {
    AreaCostsExample &state = area_costs_example;
    renderer::release(state.overlay_drawing);
    renderer::release(state.path_drawing);
    nav_mesh_search::release(state.workspace);
    if (state.generated) nav_mesh_api::release(state.nav_mesh);
    state.generated = false;
}

const Example EXAMPLE_AREA_COSTS = {"area costs", init, update, ui, shutdown};
