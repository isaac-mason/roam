#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/geometry/merge_positions_and_indices.h"
#include "roam/debug/debug.h"
#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum NavMeshAreaType : uint8_t {
    AREA_GROUND = 1,
    AREA_DOOR = 2,
};

static const uint8_t DOORS_AREA_START = 10;
static const int MAX_DOORS = 8;

struct Door {
    Box3 box;
    int door_id;
    int32_t node;
};

struct Key {
    int key_id;
    int32_t node;
};

struct DoorsAndKeysExample {
    bool generated;
    SoloNavMeshResult result;
    MergedMesh level_mesh;
    Door doors[MAX_DOORS];
    int door_count;
    Key keys[MAX_DOORS];
    int key_count;
    uint32_t held_keys;
    QueryFilter filter;
    bool show_nav_mesh;
    bool show_heightfield;
    bool show_compact_heightfield;
    bool key_1;
    bool key_2;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing heightfield_drawing;
    StaticDrawing compact_heightfield_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    bool path_success;
    bool path_partial;
    QueryWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static DoorsAndKeysExample doors_and_keys_example;

static bool pass_doors_filter(const QueryFilter &filter, NodeRef node_ref, const NavMesh &nav_mesh) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);
    if (node.area == AREA_GROUND) return true;
    if (node.area == AREA_DOOR) {
        uint32_t held_keys = *(const uint32_t *)filter.user_data;
        return node.flags < 32 && (held_keys & (1u << node.flags)) != 0;
    }
    return false;
}

static SoloNavMeshResult generate_nav_mesh(const InputTriangleMesh &input, const SoloNavMeshOptions &options, const Door *doors, int door_count) {
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

    for (int i = 0; i < door_count; i++) {
        compact_heightfield::mark_box_area(compact, doors[i].box, (uint8_t)(DOORS_AREA_START + doors[i].door_id));
    }

    compact_heightfield::erode_walkable_area(compact, options.walkable_radius_voxels);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    intermediates.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    intermediates.poly_mesh = poly_mesh::build_poly_mesh(context, intermediates.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = intermediates.poly_mesh;
    for (uint32_t poly_index = 0; poly_index < (uint32_t)polys.poly_count; poly_index++) {
        if (polys.areas[poly_index] == WALKABLE_AREA) {
            polys.areas[poly_index] = AREA_GROUND;
            polys.flags[poly_index] = 1;
        } else {
            polys.flags[poly_index] = (uint32_t)(polys.areas[poly_index] - DOORS_AREA_START);
            polys.areas[poly_index] = AREA_DOOR;
        }
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

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::cylinder(list, position, 0.03f, 1.2f, {0.8f, 0.8f, 0.8f, 1});
    Vec3 top = {position.x, position.y + 1.2f, position.z};
    renderer::triangle(list, top, {top.x + 0.5f, top.y - 0.15f, top.z}, {top.x, top.y - 0.3f, top.z}, color);
}

static void update_path() {
    DoorsAndKeysExample &state = doors_and_keys_example;
    renderer::release(state.path_drawing);
    const NavMesh &nav_mesh = state.result.nav_mesh;

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, nav_mesh, state.start, state.end, {1, 1, 1}, state.filter, state.points,
                         {.node_path = state.node_path});
    state.path_success = path.success;
    state.path_partial = (path.straight_path_flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) != 0;

    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace);
    renderer::add_primitives(list, nodes);
    debug::release(nodes);

    for (uint32_t n = 0; n < path.node_path_count; n++) {
        NodeRef node_ref = state.node_path[n];
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(nav_mesh, node_ref);
        app::lift(poly, 0.2f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    for (uint32_t i = 0; i < path.point_count; i++) {
        renderer::point(list, state.points[i].position, 0.2f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, state.points[i - 1].position, state.points[i].position, {1, 1, 0, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void update_keys(App &app) {
    DoorsAndKeysExample &state = doors_and_keys_example;
    state.held_keys = 0;
    if (state.key_1) state.held_keys |= 1u << 1;
    if (state.key_2) state.held_keys |= 1u << 2;

    for (int i = 0; i < state.door_count; i++) app.level.nodes[(uint32_t)state.doors[i].node].hidden = (state.held_keys & (1u << state.doors[i].door_id)) != 0;
    for (int i = 0; i < state.key_count; i++) app.level.nodes[(uint32_t)state.keys[i].node].hidden = (state.held_keys & (1u << state.keys[i].key_id)) != 0;
    renderer::set_level(app.renderer, app.level);

    update_path();
}

static void init(App &app) {
    DoorsAndKeysExample &state = doors_and_keys_example;
    state = {};
    state.workspace = nav_mesh_search::create_query_workspace();
    state.start = {-20.6f, 0.2f, -6.8f};
    state.end = {-5.4f, 0.3f, -2.9f};
    state.filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    state.filter.pass_filter = pass_doors_filter;
    state.filter.user_data = &state.held_keys;

    app::load_level(app, "doors-and-keys.glb");
    camera::look_at(app.camera, {-20, 20, -10}, {-15, 1, 0});
    Model &level = app.level;

    int32_t level_node = model::find_node(level, "merged level");
    Array<uint32_t> level_indices = {};
    for (uint32_t i = 0; i < level.nodes.length; i++) {
        const ModelNode &node = level.nodes[i];
        float id = 0;
        if (model::extras_number(node, "door", id) && state.door_count < MAX_DOORS) {
            Box3 box = node.bounds;
            box.min.y -= 0.5f;
            state.doors[state.door_count++] = {box, (int)id, (int32_t)i};
        } else if (model::extras_number(node, "key", id) && state.key_count < MAX_DOORS) {
            state.keys[state.key_count++] = {(int)id, (int32_t)i};
        }
        if (level_node >= 0 && model::is_descendant(level, (int32_t)i, level_node)) {
            for (uint32_t index = node.index_start; index < node.index_start + node.index_count; index++) level_indices.push(level.indices[index]);
        }
    }

    InputTriangleMesh view = {level.positions, level_indices};
    state.level_mesh = merge_positions_and_indices::merge_positions_and_indices({&view, 1});
    level_indices.release();

    const float cell_size = 0.5f;
    const float cell_height = 0.5f;
    const float walkable_radius_world = 0.3f;
    const float walkable_climb_world = 0.5f;
    const float walkable_height_world = 2;
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
        .detail_sample_distance = cell_size * 6,
        .detail_sample_max_error = cell_height * 4,
    };
    state.result = generate_nav_mesh({state.level_mesh.positions, state.level_mesh.indices}, options,
                                     state.doors, state.door_count);
    state.generated = true;

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.result.nav_mesh);
    app::lift(nav_mesh, 0.1f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    DebugPrimitives heightfield = debug::create_heightfield_helper(state.result.intermediates.heightfield);
    app::lift(heightfield, 0.05f);
    state.heightfield_drawing = renderer::create_static(heightfield);
    debug::release(heightfield);

    DebugPrimitives compact = debug::create_compact_heightfield_solid_helper(state.result.intermediates.compact_heightfield);
    app::lift(compact, 0.1f);
    state.compact_heightfield_drawing = renderer::create_static(compact);
    debug::release(compact);

    update_keys(app);
}

static void update(App &app, float) {
    DoorsAndKeysExample &state = doors_and_keys_example;
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

static void ui(App &app) {
    DoorsAndKeysExample &state = doors_and_keys_example;
    ImGui::TextWrapped("click to set the start, right click or shift click to set the end. Pick up keys to open their doors");
    ImGui::Text("%s%s", state.path_success ? "path found" : "no path", state.path_partial ? " (partial)" : "");
    if (ImGui::CollapsingHeader("Debug Views", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("nav mesh", &state.show_nav_mesh);
        ImGui::Checkbox("heightfield", &state.show_heightfield);
        ImGui::Checkbox("compact heightfield", &state.show_compact_heightfield);
    }
    if (ImGui::CollapsingHeader("Keys", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed = ImGui::Checkbox("key 1", &state.key_1);
        changed |= ImGui::Checkbox("key 2", &state.key_2);
        if (changed) update_keys(app);
    }
}

static void shutdown(App &) {
    DoorsAndKeysExample &state = doors_and_keys_example;
    renderer::release(state.nav_mesh_drawing);
    nav_mesh_search::release(state.workspace);
    renderer::release(state.heightfield_drawing);
    renderer::release(state.compact_heightfield_drawing);
    renderer::release(state.path_drawing);
    merge_positions_and_indices::release(state.level_mesh);
    if (state.generated) {
        generate_solo_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_DOORS_AND_KEYS = {"doors and keys", init, update, ui, shutdown};
