#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bench/bench.h"
#include "roam/blocks/agents/crowd.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

struct NavTestInput {
    Array<float> positions;
    Array<uint32_t> indices;
};

static bool parse_array(const char *json, const char *key, Array<double> &out) {
    const char *start = strstr(json, key);
    if (!start) return false;
    start = strchr(start, '[');
    if (!start) return false;
    const char *cursor = start + 1;
    while (*cursor && *cursor != ']') {
        char *end = nullptr;
        double value = strtod(cursor, &end);
        if (end == cursor) {
            cursor++;
            continue;
        }
        out.push(value);
        cursor = end;
    }
    return true;
}

static NavTestInput load_input(const char *path) {
    NavTestInput input = {};
    FILE *file = fopen(path, "rb");
    if (!file) return input;
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    Array<char> text = {};
    text.resize((uint32_t)size + 1);
    size_t read = fread(text.data, 1, (size_t)size, file);
    text[(uint32_t)read] = 0;
    fclose(file);

    Array<double> values = {};
    if (parse_array(text.data, "\"positions\"", values)) {
        for (double value : values) input.positions.push((float)value);
    }
    values.clear();
    if (parse_array(text.data, "\"indices\"", values)) {
        for (double value : values) input.indices.push((uint32_t)value);
    }
    values.release();
    text.release();
    return input;
}

static SoloNavMeshOptions solo_options() {
    const float cell_size = 0.15f;
    const float cell_height = 0.15f;
    const float walkable_radius = 0.1f;
    const float walkable_climb = 0.5f;
    const float walkable_height = 0.25f;
    return {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_radius_voxels = (int)ceilf(walkable_radius / cell_size),
        .walkable_radius_world = walkable_radius,
        .walkable_climb_voxels = (int)ceilf(walkable_climb / cell_height),
        .walkable_climb_world = walkable_climb,
        .walkable_height_voxels = (int)ceilf(walkable_height / cell_height),
        .walkable_height_world = walkable_height,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 5,
        .detail_sample_distance = cell_size * 6,
        .detail_sample_max_error = cell_height * 1,
    };
}

static TiledNavMeshOptions tiled_options() {
    SoloNavMeshOptions solo = solo_options();
    return {
        .cell_size = solo.cell_size,
        .cell_height = solo.cell_height,
        .tile_size_voxels = 32,
        .tile_size_world = 32 * solo.cell_size,
        .walkable_radius_voxels = solo.walkable_radius_voxels,
        .walkable_radius_world = solo.walkable_radius_world,
        .walkable_climb_voxels = solo.walkable_climb_voxels,
        .walkable_climb_world = solo.walkable_climb_world,
        .walkable_height_voxels = solo.walkable_height_voxels,
        .walkable_height_world = solo.walkable_height_world,
        .walkable_slope_angle_degrees = solo.walkable_slope_angle_degrees,
        .border_size = solo.walkable_radius_voxels + 3,
        .min_region_area = solo.min_region_area,
        .merge_region_area = solo.merge_region_area,
        .max_simplification_error = solo.max_simplification_error,
        .max_edge_length = solo.max_edge_length,
        .max_vertices_per_poly = solo.max_vertices_per_poly,
        .detail_sample_distance = solo.detail_sample_distance,
        .detail_sample_max_error = solo.detail_sample_max_error,
    };
}

enum Stage {
    STAGE_MARK,
    STAGE_RASTERIZE,
    STAGE_FILTER,
    STAGE_COMPACT,
    STAGE_ERODE,
    STAGE_DISTANCE_FIELD,
    STAGE_REGIONS,
    STAGE_CONTOURS,
    STAGE_POLY_MESH,
    STAGE_DETAIL,
    STAGE_COUNT,
};

struct Pipeline {
    int built;
    BuildContext context;
    Array<uint8_t> tri_area_ids;
    Heightfield heightfield;
    CompactHeightfield compact_heightfield;
    ContourSet contour_set;
    PolyMesh poly_mesh;
    PolyMeshDetail poly_mesh_detail;
};

struct BenchState {
    NavTestInput input;
    SoloNavMeshOptions options;
    Pipeline pipeline;
    int stage;
    NavMesh nav_mesh;
    Array<Vec3> centers;
    Array<NodeRef> center_refs;
    Crowd crowd;
};

static BenchState state;

static void advance(Pipeline &pipeline, int stage) {
    const NavTestInput &input = state.input;
    const SoloNavMeshOptions &options = state.options;
    switch (stage) {
        case STAGE_MARK:
            pipeline.tri_area_ids.resize(input.indices.length / 3, 0);
            input_triangle_mesh::mark_walkable_triangles(input.positions.data, input.indices.data, input.indices.length, pipeline.tri_area_ids.data,
                                                         options.walkable_slope_angle_degrees);
            break;
        case STAGE_RASTERIZE: {
            Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(input.positions.data, input.indices.data, input.indices.length);
            GridSize grid = heightfield::calculate_grid_size(bounds, options.cell_size);
            pipeline.heightfield = heightfield::create(grid.width, grid.height, bounds, options.cell_size, options.cell_height);
            heightfield::rasterize_triangles(pipeline.context, pipeline.heightfield, input.positions.data, input.indices.data, input.indices.length,
                                             pipeline.tri_area_ids.data, options.walkable_climb_voxels);
            break;
        }
        case STAGE_FILTER:
            heightfield::filter_low_hanging_walkable_obstacles(pipeline.heightfield, options.walkable_climb_voxels);
            heightfield::filter_ledge_spans(pipeline.heightfield, options.walkable_height_voxels, options.walkable_climb_voxels);
            heightfield::filter_walkable_low_height_spans(pipeline.heightfield, options.walkable_height_voxels);
            break;
        case STAGE_COMPACT:
            pipeline.compact_heightfield =
                compact_heightfield::build(pipeline.context, options.walkable_height_voxels, options.walkable_climb_voxels, pipeline.heightfield);
            break;
        case STAGE_ERODE: compact_heightfield::erode_walkable_area(options.walkable_radius_voxels, pipeline.compact_heightfield); break;
        case STAGE_DISTANCE_FIELD: compact_heightfield_regions::build_distance_field(pipeline.compact_heightfield); break;
        case STAGE_REGIONS:
            compact_heightfield_regions::build_regions(pipeline.context, pipeline.compact_heightfield, options.border_size, options.min_region_area,
                                                       options.merge_region_area);
            break;
        case STAGE_CONTOURS:
            pipeline.contour_set = contour_set::build_contours(pipeline.context, pipeline.compact_heightfield, options.max_simplification_error,
                                                               options.max_edge_length, CONTOUR_TESS_WALL_EDGES);
            break;
        case STAGE_POLY_MESH: pipeline.poly_mesh = poly_mesh::build_poly_mesh(pipeline.context, pipeline.contour_set, options.max_vertices_per_poly); break;
        case STAGE_DETAIL:
            pipeline.poly_mesh_detail = poly_mesh_detail::build_poly_mesh_detail(pipeline.context, pipeline.poly_mesh, pipeline.compact_heightfield,
                                                                                 options.detail_sample_distance, options.detail_sample_max_error);
            break;
    }
    pipeline.built = stage + 1;
}

static void release(Pipeline &pipeline) {
    if (pipeline.built > STAGE_DETAIL) poly_mesh_detail::release(pipeline.poly_mesh_detail);
    if (pipeline.built > STAGE_POLY_MESH) poly_mesh::release(pipeline.poly_mesh);
    if (pipeline.built > STAGE_CONTOURS) contour_set::release(pipeline.contour_set);
    if (pipeline.built > STAGE_COMPACT) compact_heightfield::release(pipeline.compact_heightfield);
    if (pipeline.built > STAGE_RASTERIZE) heightfield::release(pipeline.heightfield);
    pipeline.tri_area_ids.release();
    build_context::release(pipeline.context);
    pipeline = {};
}

static void prepare_stage() {
    state.pipeline = {};
    state.pipeline.tri_area_ids.resize(state.input.indices.length / 3, 0);
    state.pipeline.tri_area_ids.clear();
    for (int stage = 0; stage < state.stage; stage++) advance(state.pipeline, stage);
}

static void run_stage() { advance(state.pipeline, state.stage); }

static void finish_stage() { release(state.pipeline); }

static void run_solo() {
    SoloNavMeshResult result =
        generate_solo_nav_mesh::generate_solo_nav_mesh({state.input.positions.data, state.input.indices.data, state.input.indices.length}, state.options);
    generate_solo_nav_mesh::release(result.intermediates);
    nav_mesh_api::release_nav_mesh(result.nav_mesh);
}

static void run_tiled() {
    TiledNavMeshResult result =
        generate_tiled_nav_mesh::generate_tiled_nav_mesh({state.input.positions.data, state.input.indices.data, state.input.indices.length}, tiled_options());
    generate_tiled_nav_mesh::release(result.intermediates);
    nav_mesh_api::release_nav_mesh(result.nav_mesh);
}

static const uint32_t QUERY_COUNT = 100;

static uint32_t pair_end(uint32_t i) { return (i * 37 + 11) % state.centers.length; }

static const uint32_t MAX_PATH = 256;

static SearchWorkspace query_workspace = {};

static void run_find_path() {
    StraightPathPoint points[MAX_PATH];
    for (uint32_t i = 0; i < QUERY_COUNT; i++) {
        uint32_t a = i % state.centers.length;
        FindPathResult path;
        find_path::find_path(path, query_workspace, state.nav_mesh, state.centers[a], state.centers[pair_end(i)], {1, 1, 1},
                             nav_mesh_api::DEFAULT_QUERY_FILTER, points);
    }
}

static void run_raycast() {
    NodeRef path[MAX_PATH];
    for (uint32_t i = 0; i < QUERY_COUNT; i++) {
        uint32_t a = i % state.centers.length;
        RaycastResult ray;
        nav_mesh_search::raycast(ray, state.nav_mesh, state.center_refs[a], state.centers[a], state.centers[pair_end(i)], nav_mesh_api::DEFAULT_QUERY_FILTER,
                                 path);
    }
}

static void run_path_ends() {
    for (uint32_t i = 0; i < QUERY_COUNT; i++) {
        FindNearestPolyResult a;
        nav_mesh_api::find_nearest_poly(a, state.nav_mesh, state.centers[i % state.centers.length], {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        FindNearestPolyResult b;
        nav_mesh_api::find_nearest_poly(b, state.nav_mesh, state.centers[pair_end(i)], {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    }
}

static void run_node_paths() {
    NodeRef path[MAX_PATH];
    for (uint32_t i = 0; i < QUERY_COUNT; i++) {
        uint32_t a = i % state.centers.length;
        uint32_t b = pair_end(i);
        FindNodePathResult result;
        nav_mesh_search::find_node_path(result, query_workspace, state.nav_mesh, state.center_refs[a], state.center_refs[b], state.centers[a],
                                        state.centers[b], nav_mesh_api::DEFAULT_QUERY_FILTER, path);
    }
}

static void run_find_nearest_poly() {
    for (uint32_t i = 0; i < 2 * QUERY_COUNT; i++) {
        Vec3 point = state.centers[i % state.centers.length];
        point.x += 0.37f;
        point.y += 0.5f;
        point.z -= 0.21f;
        FindNearestPolyResult nearest;
        nav_mesh_api::find_nearest_poly(nearest, state.nav_mesh, point, {1, 2, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
    }
}

static const uint32_t CROWD_AGENTS = 100;
static const uint32_t CROWD_FRAMES = 60;

static void prepare_crowd() {
    state.crowd = crowd::create(0.6f);
    AgentParams params = {
        .radius = 0.3f,
        .height = 0.6f,
        .max_acceleration = 15,
        .max_speed = 3.5f,
        .collision_query_range = 2,
        .separation_weight = 0.5f,
        .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_OPTIMIZE_TOPO |
                        CROWD_UPDATE_OPTIMIZE_VIS,
    };
    for (uint32_t i = 0; i < CROWD_AGENTS; i++) {
        uint32_t id = crowd::add_agent(state.crowd, state.nav_mesh, state.centers[i % state.centers.length], params);
        uint32_t target = (i * 7 + 3) % state.centers.length;
        crowd::request_move_target(state.crowd, id, state.center_refs[target], state.centers[target]);
    }
}

static void prepare_capped_crowd() {
    prepare_crowd();
    state.crowd.max_neighbours = 6;
}

static void run_crowd() {
    for (uint32_t frame = 0; frame < CROWD_FRAMES; frame++) crowd::update(state.crowd, state.nav_mesh, 1.0f / 60.0f);
}

static void finish_crowd() { crowd::release(state.crowd); }

static void build_query_points() {
    SoloNavMeshResult result =
        generate_solo_nav_mesh::generate_solo_nav_mesh({state.input.positions.data, state.input.indices.data, state.input.indices.length}, state.options);
    generate_solo_nav_mesh::release(result.intermediates);
    state.nav_mesh = result.nav_mesh;

    const NavMeshTile &tile = state.nav_mesh.tiles[0];
    for (const NavMeshPoly &poly : tile.polys) {
        Vec3 center = {0, 0, 0};
        for (uint32_t i = 0; i < poly.vertex_count; i++) center = vec3::add(center, vec3::load(tile.vertices.data, tile.poly_vertices[poly.vertex_base + i] * 3));
        center = vec3::scale(center, 1.0f / (float)poly.vertex_count);
        FindNearestPolyResult nearest = {};
        nav_mesh_api::find_nearest_poly(nearest, state.nav_mesh, center, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
        if (!nearest.success) continue;
        state.centers.push(nearest.position);
        state.center_refs.push(nearest.node_ref);
    }
    query_workspace = nav_mesh_search::create_search_workspace();
}

int main(int argc, char **argv) {
    const char *fixture = "../navcat/bench/fixtures/nav-test.json";
    const char *filter = nullptr;
    double min_total_milliseconds = 500;
    const char *dump_points = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) {
            fixture = argv[++i];
        } else if (strcmp(argv[i], "--dump-points") == 0 && i + 1 < argc) {
            dump_points = argv[++i];
        } else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc) {
            min_total_milliseconds = atof(argv[++i]);
        } else if (strcmp(argv[i], "--cpu") == 0) {
            bench::measure_cpu_time(true);
        } else if (strcmp(argv[i], "--samples") == 0 && i + 1 < argc) {
            bench::use_fixed_samples(atoi(argv[++i]));
        } else {
            filter = argv[i];
        }
    }

    state.input = load_input(fixture);
    if (state.input.indices.length == 0) {
        fprintf(stderr, "roam_bench: could not load %s (pass --fixture <navcat>/bench/fixtures/nav-test.json)\n", fixture);
        return 1;
    }
    state.options = solo_options();
    build_query_points();

    if (dump_points) {
        FILE *file = fopen(dump_points, "w");
        if (!file) return 1;
        fprintf(file, "[");
        for (uint32_t i = 0; i < state.centers.length; i++) {
            const Vec3 &p = state.centers[i];
            fprintf(file, "%s%.9g,%.9g,%.9g", i ? "," : "", p.x, p.y, p.z);
        }
        fprintf(file, "]\n");
        fclose(file);
        return 0;
    }

    static const char *const STAGE_NAMES[STAGE_COUNT] = {
        "mark walkable triangles", "rasterize triangles", "filter heightfield", "build compact heightfield", "erode walkable area",
        "build distance field",    "build regions",       "build contours",     "build poly mesh",          "build poly mesh detail",
    };

    BenchCase cases[STAGE_COUNT + 9] = {};
    uint32_t case_count = 0;
    for (int stage = 0; stage < STAGE_COUNT; stage++) cases[case_count++] = {STAGE_NAMES[stage], prepare_stage, run_stage, finish_stage};
    cases[case_count++] = {"generate solo nav mesh", nullptr, run_solo, nullptr};
    cases[case_count++] = {"generate tiled nav mesh", nullptr, run_tiled, nullptr};
    cases[case_count++] = {"find path (100 pairs)", nullptr, run_find_path, nullptr};
    cases[case_count++] = {"path ends (200 nearest)", nullptr, run_path_ends, nullptr};
    cases[case_count++] = {"node paths (100 pairs)", nullptr, run_node_paths, nullptr};
    cases[case_count++] = {"raycast (100 rays)", nullptr, run_raycast, nullptr};
    cases[case_count++] = {"find nearest poly (200 points)", nullptr, run_find_nearest_poly, nullptr};
    cases[case_count++] = {"crowd (100 agents, 60 updates)", prepare_crowd, run_crowd, finish_crowd};
    cases[case_count++] = {"crowd (100 agents, 60 updates, 6 neighbours)", prepare_capped_crowd, run_crowd, finish_crowd};

    printf("%-34s %12s %12s %8s\n", "bench", "median ms", "min ms", "samples");
    for (uint32_t i = 0; i < case_count; i++) {
        if (filter && !strstr(cases[i].name, filter)) continue;
        state.stage = i < STAGE_COUNT ? (int)i : 0;
        BenchResult result = bench::run(cases[i], min_total_milliseconds);
        printf("%-34s %12.4f %12.4f %8u\n", cases[i].name, result.median_milliseconds, result.min_milliseconds, result.samples);
        fflush(stdout);
    }

    nav_mesh_search::release(query_workspace);
    nav_mesh_api::release_nav_mesh(state.nav_mesh);
    state.centers.release();
    state.center_refs.release();
    state.input.positions.release();
    state.input.indices.release();
    return 0;
}
