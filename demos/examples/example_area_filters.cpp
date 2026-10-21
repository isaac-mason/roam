#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/debug/debug.h"
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

enum AreaFiltersAreaType : uint8_t {
    AREA_FILTERS_GROUND = 1,
    AREA_FILTERS_WATER = 2,
};

enum AreaFiltersFilter : int {
    AREA_FILTERS_FILTER_ALL = 0,
    AREA_FILTERS_FILTER_GROUND = 1,
    AREA_FILTERS_FILTER_WATER = 2,
};

static const Box3 WATER_BOUNDS = {{-100, -1, -100}, {100, 1, 100}};

struct AreaFiltersNavMesh {
    NavMesh nav_mesh;
    BuildContext build_context;
    Array<uint8_t> tri_area_ids;
    Heightfield heightfield;
    CompactHeightfield compact_heightfield;
    ContourSet contour_set;
    PolyMesh poly_mesh;
    PolyMeshDetail poly_mesh_detail;
};

struct AreaFiltersExample {
    bool generated;
    AreaFiltersNavMesh result;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing heightfield_drawing;
    StaticDrawing compact_heightfield_drawing;
    StaticDrawing path_drawing;
    bool show_nav_mesh;
    bool show_heightfield;
    bool show_compact_heightfield;
    int filter;
    Vec3 start;
    Vec3 end;
    bool success;
    bool partial;
    uint32_t point_count;
    SearchWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static AreaFiltersExample area_filters_example;

static bool pass_area(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) {
    return nav_mesh_api::get_node_by_ref(nav_mesh, node_ref).area == (uint8_t)(uintptr_t)filter.user_data;
}

static QueryFilter area_only_query_filter(AreaFiltersAreaType area) {
    QueryFilter filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    filter.pass_filter = pass_area;
    filter.user_data = (void *)(uintptr_t)area;
    return filter;
}

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.533f, 0.533f, 0.533f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static AreaFiltersNavMesh generate_nav_mesh(const float *positions, const uint32_t *indices, uint32_t index_count) {
    const float cell_size = 0.2f;
    const float cell_height = 0.15f;
    const float walkable_radius_world = 0.1f;
    const int walkable_radius_voxels = (int)ceilf(walkable_radius_world / cell_size);
    const float walkable_climb_world = 0.5f;
    const int walkable_climb_voxels = (int)ceilf(walkable_climb_world / cell_height);
    const float walkable_height_world = 0.1f;
    const int walkable_height_voxels = (int)ceilf(walkable_height_world / cell_height);
    const float walkable_slope_angle_degrees = 45;
    const int border_size = 4;
    const int min_region_area = 8;
    const int merge_region_area = 20;
    const float max_simplification_error = 1.3f;
    const int max_edge_length = 12;
    const int max_vertices_per_poly = 5;
    const float detail_sample_distance = cell_size * 6;
    const float detail_sample_max_error = cell_height * 1;

    AreaFiltersNavMesh result = {};
    BuildContext &context = result.build_context;

    result.tri_area_ids.resize(index_count / 3, 0);
    input_triangle_mesh::mark_walkable_triangles(positions, indices, index_count, result.tri_area_ids.data, walkable_slope_angle_degrees);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(positions, indices, index_count);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, cell_size);
    result.heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, cell_size, cell_height);
    Heightfield &solid = result.heightfield;

    heightfield::rasterize_triangles(context, solid, positions, indices, index_count, result.tri_area_ids.data, walkable_climb_voxels);

    heightfield::filter_low_hanging_walkable_obstacles(solid, walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, walkable_height_voxels, walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, walkable_height_voxels);

    result.compact_heightfield = compact_heightfield::build(context, solid, walkable_height_voxels, walkable_climb_voxels);
    CompactHeightfield &compact = result.compact_heightfield;

    compact_heightfield::mark_box_area(compact, WATER_BOUNDS, AREA_FILTERS_WATER);

    compact_heightfield::erode_walkable_area(compact, walkable_radius_voxels);

    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, border_size, min_region_area, merge_region_area);

    result.contour_set = contour_set::build_contours(context, compact, max_simplification_error, max_edge_length, CONTOUR_TESS_WALL_EDGES);

    result.poly_mesh = poly_mesh::build_poly_mesh(context, result.contour_set, max_vertices_per_poly);
    PolyMesh &polys = result.poly_mesh;

    for (uint32_t poly_index = 0; poly_index < (uint32_t)polys.poly_count; poly_index++) {
        if (polys.areas[poly_index] == WALKABLE_AREA) {
            polys.areas[poly_index] = AREA_FILTERS_GROUND;
            polys.flags[poly_index] = 0x01;
        } else if (polys.areas[poly_index] == AREA_FILTERS_WATER) {
            polys.areas[poly_index] = AREA_FILTERS_WATER;
            polys.flags[poly_index] = 0x02;
        }
    }

    result.poly_mesh_detail = poly_mesh_detail::build_poly_mesh_detail(context, polys, compact, detail_sample_distance, detail_sample_max_error);

    NavMesh &nav_mesh = result.nav_mesh;
    nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.tile_width = polys.bounds.max.x - polys.bounds.min.x;
    nav_mesh.tile_height = polys.bounds.max.z - polys.bounds.min.z;
    nav_mesh.origin = polys.bounds.min;

    NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
    NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, result.poly_mesh_detail);

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
    return result;
}

static void release(AreaFiltersNavMesh &result) {
    nav_mesh_api::release(result.nav_mesh);
    build_context::release(result.build_context);
    result.tri_area_ids.release();
    heightfield::release(result.heightfield);
    compact_heightfield::release(result.compact_heightfield);
    contour_set::release(result.contour_set);
    poly_mesh::release(result.poly_mesh);
    poly_mesh_detail::release(result.poly_mesh_detail);
}

static StaticDrawing lifted_static(DebugPrimitives primitives, float height) {
    app::lift(primitives, height);
    StaticDrawing drawing = renderer::create_static(primitives);
    debug::release(primitives);
    return drawing;
}

static void update_path() {
    AreaFiltersExample &state = area_filters_example;
    renderer::release(state.path_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    DrawList list = {};
    flag(list, state.start, {0.129f, 0.588f, 0.953f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    QueryFilter filter = state.filter == AREA_FILTERS_FILTER_ALL      ? nav_mesh_api::DEFAULT_QUERY_FILTER
                         : state.filter == AREA_FILTERS_FILTER_GROUND ? area_only_query_filter(AREA_FILTERS_GROUND)
                                                                      : area_only_query_filter(AREA_FILTERS_WATER);

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, nav_mesh, state.start, state.end, {1, 1, 1}, filter, state.points,
                         {.node_path = state.node_path});
    state.success = path.success;
    state.partial = (path.straight_path_flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) != 0;
    state.point_count = path.point_count;

    DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace);
    renderer::add_primitives(list, nodes);
    debug::release(nodes);

    for (uint32_t n = 0; n < path.node_path_count; n++) {
        NodeRef node_ref = state.node_path[n];
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    for (uint32_t i = 0; i < path.point_count; i++) {
        Vec3 position = state.points[i].position;
        renderer::point(list, position, 0.2f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, state.points[i - 1].position, position, {1, 1, 0, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    AreaFiltersExample &state = area_filters_example;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.filter = AREA_FILTERS_FILTER_GROUND;
    state.start = {-8, 1.5f, -2.3f};
    state.end = {8, 1, -0.5f};

    app::load_level(app, "bridges.glb", {.skip_unwalkable_nodes = true});

    const Model &level = app.level;
    state.result = generate_nav_mesh(level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length);
    state.generated = true;

    state.nav_mesh_drawing = lifted_static(debug::create_nav_mesh_helper(state.result.nav_mesh), 0.1f);
    state.heightfield_drawing = lifted_static(debug::create_heightfield_helper(state.result.heightfield), 0.05f);
    state.compact_heightfield_drawing = lifted_static(debug::create_compact_heightfield_solid_helper(state.result.compact_heightfield), 0.1f);

    update_path();
}

static void update(App &app, float) {
    AreaFiltersExample &state = area_filters_example;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.end = app.click.point;
        } else {
            state.start = app.click.point;
        }
        update_path();
    }
    if (state.show_nav_mesh) app::draw_static(app, state.nav_mesh_drawing);
    if (state.show_heightfield) app::draw_static(app, state.heightfield_drawing);
    if (state.show_compact_heightfield) app::draw_static(app, state.compact_heightfield_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    AreaFiltersExample &state = area_filters_example;
    ImGui::TextWrapped("click to set the start, right click or shift click to set the end");
    if (ImGui::CollapsingHeader("Debug Views", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("nav mesh", &state.show_nav_mesh);
        ImGui::Checkbox("heightfield", &state.show_heightfield);
        ImGui::Checkbox("compact heightfield", &state.show_compact_heightfield);
    }
    if (ImGui::CollapsingHeader("Query Filter", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char *filters[] = {"all", "ground", "water"};
        if (ImGui::Combo("area", &state.filter, filters, 3)) update_path();
    }
    ImGui::Text("%s, %u points%s", state.success ? "found" : "failed", state.point_count, state.partial ? ", partial" : "");
}

static void shutdown(App &) {
    AreaFiltersExample &state = area_filters_example;
    renderer::release(state.nav_mesh_drawing);
    nav_mesh_search::release(state.workspace);
    renderer::release(state.heightfield_drawing);
    renderer::release(state.compact_heightfield_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) release(state.result);
    state.generated = false;
}

const Example EXAMPLE_AREA_FILTERS = {"area filters", init, update, ui, shutdown};
