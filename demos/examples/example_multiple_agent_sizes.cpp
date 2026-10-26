#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/debug/debug.h"
#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum AreaId : uint8_t {
    AREA_WALKABLE = 1,
    AREA_WALKABLE_NARROW = 2,
};

static const Color AGENT_COLORS[] = {
    {0, 0, 1, 1}, {0, 1, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {1, 0, 1, 1},
    {0, 1, 1, 1}, {1, 0.65f, 0, 1}, {0.5f, 0, 0.5f, 1}, {1, 0.75f, 0.8f, 1}, {0.56f, 0.93f, 0.56f, 1},
};

static const bool AGENT_IS_SMALL[] = {true, true, true, true, true, true, false, false, false, false};

static const float CELL_SIZE = 0.05f;
static const float CELL_HEIGHT = 0.2f;
static const float SMALL_AGENT_RADIUS = 0.15f;
static const float LARGE_AGENT_RADIUS = 0.45f;
static const double SCATTER_TIMEOUT_MILLISECONDS = 5000;

static const OffMeshConnectionParams OFF_MESH_CONNECTIONS[] = {
    {{0.39257542778564014f, 3.9164539337158204f, 2.7241512942770267f}, {1.2915380743929097f, 2.8616158587143867f, 3.398593875470379f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
    {{3.491345350637368f, 3.169861227710937f, 2.8419154179454473f}, {4.0038066734125435f, 0.466454005241394f, 1.686211347289651f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
    {{4.612475330561077f, 0.466454005241394f, 2.7619018768157435f}, {6.696740007427642f, 0.5132029874438654f, 2.5838885990777243f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 0xffffff, 0},
    {{3.8221359252929688f, 0.47645399570465086f, -4.391971844600165f}, {5.91173484469572f, 0.6573111525835266f, -4.671632275169128f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 0xffffff, 0},
    {{8.354324172733968f, 0.5340897451517822f, -3.2333049546492223f}, {8.461111697936666f, 0.8365034207348984f, -1.0863215738579806f}, 0.5f,
     OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 0xffffff, 0},
};

enum DebugHelper {
    HELPER_TRIANGLE_AREA_IDS,
    HELPER_HEIGHTFIELD,
    HELPER_COMPACT_HEIGHTFIELD_SOLID,
    HELPER_COMPACT_HEIGHTFIELD_DISTANCES,
    HELPER_COMPACT_HEIGHTFIELD_REGIONS,
    HELPER_RAW_CONTOURS,
    HELPER_SIMPLIFIED_CONTOURS,
    HELPER_POLY_MESH,
    HELPER_POLY_MESH_DETAIL,
    HELPER_NAV_MESH_BV_TREE,
    HELPER_NAV_MESH,
    HELPER_OFF_MESH_CONNECTIONS,
    HELPER_NAV_MESH_ISLANDS,
    HELPER_COUNT,
};

static const char *HELPER_NAMES[HELPER_COUNT] = {
    "triangle area ids", "heightfield", "compact heightfield solid", "compact heightfield distances", "compact heightfield regions",
    "raw contours",      "simplified contours", "poly mesh",       "poly mesh detail",              "nav mesh bv tree",
    "nav mesh",          "off-mesh connections", "nav mesh islands",
};

struct MultipleAgentSizesExample {
    bool generated;
    SoloNavMeshResult result;
    bool show_helpers[HELPER_COUNT];
    StaticDrawing helper_drawings[HELPER_COUNT];
    Crowd crowd;
    uint32_t random_state;
    QueryFilter small_agent_filter;
    QueryFilter large_agent_filter;
    bool show_velocity_vectors;
    bool show_path_line;
    double last_interaction_time;
    double last_scatter_time;
};

static MultipleAgentSizesExample multiple_agent_sizes_example;

static float next_random(void *user_data) {
    uint32_t &state = *(uint32_t *)user_data;
    state = state * 1664525u + 1013904223u;
    return (float)((double)state / 4294967296.0);
}

static bool pass_all(const QueryFilter &, NodeRef, const NavMesh &) { return true; }

static bool pass_not_narrow(const QueryFilter &, NodeRef node_ref, const NavMesh &nav_mesh) {
    return nav_mesh_api::get_node_by_ref(nav_mesh, node_ref).area != AREA_WALKABLE_NARROW;
}

static SoloNavMeshResult generate_nav_mesh(const InputTriangleMesh &input, const SoloNavMeshOptions &options,
                                           Span<const WalkableAreaThreshold> thresholds) {
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

    compact_heightfield::erode_and_mark_walkable_areas(compact, options.walkable_radius_voxels, thresholds);
    compact_heightfield::median_filter_walkable_area(compact);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions_monotone(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    intermediates.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    intermediates.poly_mesh = poly_mesh::build_poly_mesh(context, intermediates.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = intermediates.poly_mesh;
    for (uint32_t poly_index = 0; poly_index < (uint32_t)polys.poly_count; poly_index++) {
        if (polys.areas[poly_index] == WALKABLE_AREA) polys.areas[poly_index] = AREA_WALKABLE;
        if (polys.areas[poly_index] != 0) polys.flags[poly_index] = 1;
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
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    const SoloNavMeshIntermediates &intermediates = state.result.intermediates;
    const NavMesh &nav_mesh = state.result.nav_mesh;
    switch (helper) {
        case HELPER_TRIANGLE_AREA_IDS:
            return debug::create_triangle_area_ids_helper(intermediates.input, intermediates.tri_area_ids);
        case HELPER_HEIGHTFIELD: return debug::create_heightfield_helper(intermediates.heightfield);
        case HELPER_COMPACT_HEIGHTFIELD_SOLID: return debug::create_compact_heightfield_solid_helper(intermediates.compact_heightfield);
        case HELPER_COMPACT_HEIGHTFIELD_DISTANCES: return debug::create_compact_heightfield_distances_helper(intermediates.compact_heightfield);
        case HELPER_COMPACT_HEIGHTFIELD_REGIONS: return debug::create_compact_heightfield_regions_helper(intermediates.compact_heightfield);
        case HELPER_RAW_CONTOURS: return debug::create_raw_contours_helper(intermediates.contour_set);
        case HELPER_SIMPLIFIED_CONTOURS: return debug::create_simplified_contours_helper(intermediates.contour_set);
        case HELPER_POLY_MESH: return debug::create_poly_mesh_helper(intermediates.poly_mesh);
        case HELPER_POLY_MESH_DETAIL: return debug::create_poly_mesh_detail_helper(intermediates.poly_mesh_detail);
        case HELPER_NAV_MESH_BV_TREE: return debug::create_nav_mesh_bv_tree_helper(nav_mesh);
        case HELPER_NAV_MESH: {
            DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
        case HELPER_NAV_MESH_ISLANDS: {
            DebugPrimitives primitives = debug::create_nav_mesh_islands_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
        default: return debug::create_nav_mesh_off_mesh_connections_helper(nav_mesh);
    }
}

static void update_helpers() {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    for (int helper = 0; helper < HELPER_COUNT; helper++) {
        renderer::release(state.helper_drawings[helper]);
        if (!state.show_helpers[helper]) continue;
        DebugPrimitives primitives = create_helper(helper);
        state.helper_drawings[helper] = renderer::create_static(primitives);
        debug::release(primitives);
    }
}

static void scatter() {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, state.result.nav_mesh, nav_mesh_api::DEFAULT_QUERY_FILTER, next_random, &state.random_state);
        if (point.success) crowd::request_move_target(state.crowd, id, point.node_ref, point.position);
    }
}

static void init(App &app) {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    state = {};
    state.random_state = 42;
    state.show_velocity_vectors = true;
    state.show_helpers[HELPER_NAV_MESH] = true;
    state.show_helpers[HELPER_OFF_MESH_CONNECTIONS] = true;

    state.small_agent_filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    state.small_agent_filter.pass_filter = pass_all;
    state.large_agent_filter = nav_mesh_api::DEFAULT_QUERY_FILTER;
    state.large_agent_filter.pass_filter = pass_not_narrow;

    app::load_level(app, "nav-test.glb");
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    const float walkable_climb_world = 0.5f;
    const float walkable_height_world = 1;
    SoloNavMeshOptions options = {
        .cell_size = CELL_SIZE,
        .cell_height = CELL_HEIGHT,
        .walkable_radius_voxels = (int)ceilf(SMALL_AGENT_RADIUS / CELL_SIZE),
        .walkable_radius_world = SMALL_AGENT_RADIUS,
        .walkable_climb_voxels = (int)ceilf(walkable_climb_world / CELL_HEIGHT),
        .walkable_climb_world = walkable_climb_world,
        .walkable_height_voxels = (int)ceilf(walkable_height_world / CELL_HEIGHT),
        .walkable_height_world = walkable_height_world,
        .walkable_slope_angle_degrees = 45,
        .border_size = 4,
        .min_region_area = 4,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 5,
        .detail_sample_distance = CELL_SIZE * 6,
        .detail_sample_max_error = CELL_HEIGHT * 1,
    };
    const WalkableAreaThreshold thresholds[] = {{AREA_WALKABLE_NARROW, (int)ceilf(LARGE_AGENT_RADIUS / CELL_SIZE)}};

    const Model &level = app.level;
    state.result = generate_nav_mesh({level.walkable_positions, level.walkable_indices}, options, thresholds);
    state.generated = true;
    NavMesh &nav_mesh = state.result.nav_mesh;

    for (const OffMeshConnectionParams &connection : OFF_MESH_CONNECTIONS) nav_mesh_api::add_off_mesh_connection(nav_mesh, connection);

    update_helpers();

    state.crowd = crowd::create(1);
    for (int i = 0; i < 10; i++) {
        FindRandomPointResult point = {};
        nav_mesh_search::find_random_point(point, nav_mesh, state.large_agent_filter, next_random, &state.random_state);
        bool small = AGENT_IS_SMALL[i];
        AgentParams params = {
            .radius = small ? SMALL_AGENT_RADIUS : LARGE_AGENT_RADIUS,
            .height = 0.6f,
            .max_acceleration = 15.0f,
            .max_speed = 3.5f,
            .collision_query_range = 2,
            .separation_weight = 0.5f,
            .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_OPTIMIZE_TOPO |
                            CROWD_UPDATE_OPTIMIZE_VIS,
            .query_filter = small ? state.small_agent_filter : state.large_agent_filter,
            .auto_traverse_off_mesh_connections = true,
        };
        crowd::add_agent(state.crowd, nav_mesh, point.position, params);
    }

    scatter();
    state.last_interaction_time = app::now_milliseconds();
    state.last_scatter_time = state.last_interaction_time;
}

static void velocity_arrow(DrawList &list, Vec3 origin, Vec3 velocity, Color color) {
    if (vec3::length(velocity) <= 0.01f) return;
    renderer::arrow(list, origin, vec3::scale_and_add(origin, velocity, 0.5f), color);
}

static void draw_agent(DrawList &list, const Agent &agent, Color color) {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    renderer::cylinder(list, agent.position, agent.radius, agent.height + agent.radius * 2, color);
    renderer::point(list, vec3::add(agent.target_position, {0, 0.1f, 0}), 0.1f, {1, 1, 0, 1});

    if (state.show_path_line) {
        StraightPathPoint corners[3];
        FindCornersResult found = {};
        path_corridor::find_corners(found, agent.corridor, state.result.nav_mesh, corners);
        if (found.success && found.corner_count > 1) {
            const Vec3 lift = {0, 0.2f, 0};
            Vec3 previous = vec3::add(agent.position, lift);
            for (uint32_t i = 0; i < found.corner_count; i++) {
                Vec3 point = vec3::add(corners[i].position, lift);
                renderer::line(list, previous, point, color);
                previous = point;
            }
        }
    }

    if (state.show_velocity_vectors) {
        velocity_arrow(list, vec3::add(agent.position, {0, 0.5f, 0}), agent.velocity, {0, 1, 0, 1});
        velocity_arrow(list, vec3::add(agent.position, {0, 0.6f, 0}), agent.desired_velocity, {1, 0, 0, 1});
    }
}

static void update(App &app, float delta_time) {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    const NavMesh &nav_mesh = state.result.nav_mesh;
    double now = app::now_milliseconds();

    if (app.click.clicked && !app.click.secondary) {
        state.last_interaction_time = now;
        if (app.click.hit) {
            FindNearestPolyResult nearest = {};
            nav_mesh_api::find_nearest_poly(nearest, nav_mesh, app.click.point, {1, 1, 1}, nav_mesh_api::ANY_QUERY_FILTER);
            if (nearest.success) {
                for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
                    if (crowd::is_agent(state.crowd, id)) crowd::request_move_target(state.crowd, id, nearest.node_ref, nearest.position);
                }
            }
        }
    }

    if (now - state.last_interaction_time > SCATTER_TIMEOUT_MILLISECONDS && now - state.last_scatter_time > SCATTER_TIMEOUT_MILLISECONDS) {
        scatter();
        state.last_scatter_time = now;
    }

    crowd::update(state.crowd, nav_mesh, fminf(delta_time, 0.1f));

    for (int helper = 0; helper < HELPER_COUNT; helper++) {
        if (state.show_helpers[helper]) app::draw_static(app, state.helper_drawings[helper]);
    }

    DrawList &list = app::frame(app);
    uint32_t color_count = sizeof(AGENT_COLORS) / sizeof(AGENT_COLORS[0]);
    uint32_t index = 0;
    for (uint32_t id = 0; id < state.crowd.agents.length; id++) {
        if (!crowd::is_agent(state.crowd, id)) continue;
        draw_agent(list, state.crowd.agents[id], AGENT_COLORS[index++ % color_count]);
    }
}

static void ui(App &app) {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    ImGui::TextWrapped("click to send every agent to a point. Large agents avoid the narrow areas that only small agents fit through");
    if (ImGui::CollapsingHeader("Agent Display", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("show velocity vectors", &state.show_velocity_vectors);
        ImGui::Checkbox("show path line", &state.show_path_line);
    }
    if (ImGui::CollapsingHeader("Debug Helpers")) {
        ImGui::Checkbox("show mesh", &app.show_level);
        bool changed = false;
        for (int helper = 0; helper < HELPER_COUNT; helper++) changed |= ImGui::Checkbox(HELPER_NAMES[helper], &state.show_helpers[helper]);
        if (changed) update_helpers();
    }
}

static void shutdown(App &app) {
    MultipleAgentSizesExample &state = multiple_agent_sizes_example;
    app.show_level = true;
    crowd::release(state.crowd);
    for (StaticDrawing &drawing : state.helper_drawings) renderer::release(drawing);
    if (state.generated) {
        generate_solo_nav_mesh::release(state.result.intermediates);
        nav_mesh_api::release(state.result.nav_mesh);
    }
    state.generated = false;
}

const Example EXAMPLE_MULTIPLE_AGENT_SIZES = {"multiple agent sizes", init, update, ui, shutdown};
