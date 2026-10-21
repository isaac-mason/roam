#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <vector>

#include "ChunkyTriMesh.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "Recast.h"

#if defined(__APPLE__)
#include <mach/mach_time.h>
#else
#include <time.h>
#endif

static bool use_cpu_time = false;

static double cpu_milliseconds() {
    timespec time;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time);
    return (double)time.tv_sec * 1000.0 + (double)time.tv_nsec / 1e6;
}

static double now_milliseconds() {
    if (use_cpu_time) return cpu_milliseconds();
#if defined(__APPLE__)
    static mach_timebase_info_data_t timebase = {0, 0};
    if (timebase.denom == 0) mach_timebase_info(&timebase);
    return (double)mach_absolute_time() * timebase.numer / timebase.denom / 1e6;
#else
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (double)time.tv_sec * 1000.0 + (double)time.tv_nsec / 1e6;
#endif
}

struct BenchCase {
    const char *name;
    void (*prepare)();
    void (*run)();
    void (*finish)();
};

static double sample(const BenchCase &bench_case) {
    if (bench_case.prepare) bench_case.prepare();
    double start = now_milliseconds();
    bench_case.run();
    double elapsed = now_milliseconds() - start;
    if (bench_case.finish) bench_case.finish();
    return elapsed;
}

static int fixed_samples = -1;

static void run_case(const BenchCase &bench_case, double min_total_milliseconds) {
    std::vector<double> times;
    if (fixed_samples >= 0) {
        for (int i = 0; i < fixed_samples; i++) times.push_back(sample(bench_case));
        if (times.empty()) return;
        std::stable_sort(times.begin(), times.end());
        printf("%-34s %12.4f %12.4f %8u\n", bench_case.name, times[times.size() / 2], times[0], (unsigned)times.size());
        return;
    }
    for (int i = 0; i < 3; i++) sample(bench_case);
    double total = 0;
    while (times.size() < 2000 && (times.size() < 10 || total < min_total_milliseconds)) {
        double elapsed = sample(bench_case);
        times.push_back(elapsed);
        total += elapsed;
    }
    std::stable_sort(times.begin(), times.end());
    printf("%-34s %12.4f %12.4f %8u\n", bench_case.name, times[times.size() / 2], times[0], (unsigned)times.size());
    fflush(stdout);
}

static bool parse_array(const char *json, const char *key, std::vector<double> &out) {
    const char *start = key ? strstr(json, key) : json;
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
        out.push_back(value);
        cursor = end;
    }
    return true;
}

static std::vector<char> read_file(const char *path) {
    std::vector<char> text;
    FILE *file = fopen(path, "rb");
    if (!file) return text;
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    text.resize((size_t)size + 1);
    size_t read = fread(text.data(), 1, (size_t)size, file);
    text[read] = 0;
    fclose(file);
    return text;
}

static const float CELL_SIZE = 0.15f;
static const float CELL_HEIGHT = 0.15f;
static const float WALKABLE_RADIUS = 0.1f;
static const float WALKABLE_CLIMB = 0.5f;
static const float WALKABLE_HEIGHT = 0.25f;
static const int TILE_SIZE = 32;

struct State {
    std::vector<float> vertices;
    std::vector<int> triangles;
    rcConfig config;
    rcContext context{false};
    float bounds_min[3];
    float bounds_max[3];

    int stage;
    std::vector<unsigned char> areas;
    rcHeightfield *heightfield;
    rcCompactHeightfield *compact_heightfield;
    rcContourSet *contour_set;
    rcPolyMesh *poly_mesh;
    rcPolyMeshDetail *poly_mesh_detail;
    int built;

    rcChunkyTriMesh chunky;
    dtNavMesh *nav_mesh;
    dtNavMeshQuery *query;
    dtQueryFilter filter;
    std::vector<float> centers;
    std::vector<dtPolyRef> center_refs;
    dtCrowd *crowd;
};

static State state;

static rcConfig solo_config() {
    rcConfig config;
    memset(&config, 0, sizeof(config));
    config.cs = CELL_SIZE;
    config.ch = CELL_HEIGHT;
    config.walkableSlopeAngle = 45;
    config.walkableHeight = (int)ceilf(WALKABLE_HEIGHT / CELL_HEIGHT);
    config.walkableClimb = (int)ceilf(WALKABLE_CLIMB / CELL_HEIGHT);
    config.walkableRadius = (int)ceilf(WALKABLE_RADIUS / CELL_SIZE);
    config.maxEdgeLen = 12;
    config.maxSimplificationError = 1.3f;
    config.minRegionArea = 8;
    config.mergeRegionArea = 20;
    config.maxVertsPerPoly = 5;
    config.detailSampleDist = CELL_SIZE * 6;
    config.detailSampleMaxError = CELL_HEIGHT * 1;
    rcVcopy(config.bmin, state.bounds_min);
    rcVcopy(config.bmax, state.bounds_max);
    rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);
    return config;
}

static void release_pipeline() {
    rcFreeHeightField(state.heightfield);
    rcFreeCompactHeightfield(state.compact_heightfield);
    rcFreeContourSet(state.contour_set);
    rcFreePolyMesh(state.poly_mesh);
    rcFreePolyMeshDetail(state.poly_mesh_detail);
    state.heightfield = nullptr;
    state.compact_heightfield = nullptr;
    state.contour_set = nullptr;
    state.poly_mesh = nullptr;
    state.poly_mesh_detail = nullptr;
    state.built = 0;
}

static void advance(int stage) {
    const rcConfig &c = state.config;
    rcContext *ctx = &state.context;
    int vertex_count = (int)state.vertices.size() / 3;
    int triangle_count = (int)state.triangles.size() / 3;
    switch (stage) {
        case 0:
            state.areas.assign((size_t)triangle_count, 0);
            rcMarkWalkableTriangles(ctx, c.walkableSlopeAngle, state.vertices.data(), vertex_count, state.triangles.data(), triangle_count,
                                    state.areas.data());
            break;
        case 1:
            state.heightfield = rcAllocHeightfield();
            rcCreateHeightfield(ctx, *state.heightfield, c.width, c.height, c.bmin, c.bmax, c.cs, c.ch);
            rcRasterizeTriangles(ctx, state.vertices.data(), vertex_count, state.triangles.data(), state.areas.data(), triangle_count,
                                 *state.heightfield, c.walkableClimb);
            break;
        case 2:
            rcFilterLowHangingWalkableObstacles(ctx, c.walkableClimb, *state.heightfield);
            rcFilterLedgeSpans(ctx, c.walkableHeight, c.walkableClimb, *state.heightfield);
            rcFilterWalkableLowHeightSpans(ctx, c.walkableHeight, *state.heightfield);
            break;
        case 3:
            state.compact_heightfield = rcAllocCompactHeightfield();
            rcBuildCompactHeightfield(ctx, c.walkableHeight, c.walkableClimb, *state.heightfield, *state.compact_heightfield);
            break;
        case 4: rcErodeWalkableArea(ctx, c.walkableRadius, *state.compact_heightfield); break;
        case 5: rcBuildDistanceField(ctx, *state.compact_heightfield); break;
        case 6: rcBuildRegions(ctx, *state.compact_heightfield, c.borderSize, c.minRegionArea, c.mergeRegionArea); break;
        case 7:
            state.contour_set = rcAllocContourSet();
            rcBuildContours(ctx, *state.compact_heightfield, c.maxSimplificationError, c.maxEdgeLen, *state.contour_set);
            break;
        case 8:
            state.poly_mesh = rcAllocPolyMesh();
            rcBuildPolyMesh(ctx, *state.contour_set, c.maxVertsPerPoly, *state.poly_mesh);
            break;
        case 9:
            state.poly_mesh_detail = rcAllocPolyMeshDetail();
            rcBuildPolyMeshDetail(ctx, *state.poly_mesh, *state.compact_heightfield, c.detailSampleDist, c.detailSampleMaxError,
                                  *state.poly_mesh_detail);
            break;
    }
    state.built = stage + 1;
}

static void prepare_stage() {
    release_pipeline();
    for (int stage = 0; stage < state.stage; stage++) advance(stage);
}

static void run_stage() { advance(state.stage); }

static void finish_stage() { release_pipeline(); }

static unsigned char *create_tile_data(rcPolyMesh &mesh, rcPolyMeshDetail &detail, int tile_x, int tile_y, int *size) {
    for (int i = 0; i < mesh.npolys; ++i) {
        if (mesh.areas[i] == RC_WALKABLE_AREA) mesh.areas[i] = 0;
        mesh.flags[i] = mesh.areas[i] == 0 ? 1 : 0;
    }

    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts = mesh.verts;
    params.vertCount = mesh.nverts;
    params.polys = mesh.polys;
    params.polyAreas = mesh.areas;
    params.polyFlags = mesh.flags;
    params.polyCount = mesh.npolys;
    params.nvp = mesh.nvp;
    params.detailMeshes = detail.meshes;
    params.detailVerts = detail.verts;
    params.detailVertsCount = detail.nverts;
    params.detailTris = detail.tris;
    params.detailTriCount = detail.ntris;
    params.walkableHeight = WALKABLE_HEIGHT;
    params.walkableRadius = WALKABLE_RADIUS;
    params.walkableClimb = WALKABLE_CLIMB;
    params.tileX = tile_x;
    params.tileY = tile_y;
    params.tileLayer = 0;
    rcVcopy(params.bmin, mesh.bmin);
    rcVcopy(params.bmax, mesh.bmax);
    params.cs = state.config.cs;
    params.ch = state.config.ch;
    params.buildBvTree = true;

    unsigned char *data = nullptr;
    if (!dtCreateNavMeshData(&params, &data, size)) return nullptr;
    return data;
}

static dtNavMesh *generate_solo() {
    release_pipeline();
    for (int stage = 0; stage < 10; stage++) advance(stage);

    int size = 0;
    unsigned char *data = create_tile_data(*state.poly_mesh, *state.poly_mesh_detail, 0, 0, &size);
    release_pipeline();
    if (!data) return nullptr;

    dtNavMesh *nav_mesh = dtAllocNavMesh();
    if (dtStatusFailed(nav_mesh->init(data, size, DT_TILE_FREE_DATA))) {
        dtFree(data);
        dtFreeNavMesh(nav_mesh);
        return nullptr;
    }
    return nav_mesh;
}

static void run_solo() { dtFreeNavMesh(generate_solo()); }

static unsigned int next_pow2(unsigned int v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}

static unsigned int ilog2(unsigned int v) {
    unsigned int r = 0;
    while (v >>= 1) r++;
    return r;
}

static void run_tiled() {
    rcContext *ctx = &state.context;
    rcConfig base = solo_config();
    int grid_width = base.width;
    int grid_height = base.height;
    int tiles_x = (grid_width + TILE_SIZE - 1) / TILE_SIZE;
    int tiles_y = (grid_height + TILE_SIZE - 1) / TILE_SIZE;
    float tile_world = TILE_SIZE * CELL_SIZE;

    rcChunkyTriMesh chunky;
    rcCreateChunkyTriMesh(state.vertices.data(), state.triangles.data(), (int)state.triangles.size() / 3, 256, &chunky);

    unsigned int tile_bits = std::min(ilog2(next_pow2((unsigned int)(tiles_x * tiles_y))), 14u);
    unsigned int poly_bits = 22 - tile_bits;
    dtNavMeshParams nav_params;
    memset(&nav_params, 0, sizeof(nav_params));
    rcVcopy(nav_params.orig, state.bounds_min);
    nav_params.tileWidth = tile_world;
    nav_params.tileHeight = tile_world;
    nav_params.maxTiles = 1 << tile_bits;
    nav_params.maxPolys = 1 << poly_bits;
    dtNavMesh *nav_mesh = dtAllocNavMesh();
    nav_mesh->init(&nav_params);

    const float *vertices = state.vertices.data();
    int vertex_count = (int)state.vertices.size() / 3;
    std::vector<unsigned char> areas((size_t)chunky.maxTrisPerChunk);
    std::vector<int> chunk_ids(4096);

    for (int tx = 0; tx < tiles_x; tx++) {
        for (int ty = 0; ty < tiles_y; ty++) {
            rcConfig c = base;
            c.tileSize = TILE_SIZE;
            c.borderSize = c.walkableRadius + 3;
            c.width = c.tileSize + c.borderSize * 2;
            c.height = c.tileSize + c.borderSize * 2;
            c.bmin[0] = state.bounds_min[0] + tx * tile_world - c.borderSize * c.cs;
            c.bmin[1] = state.bounds_min[1];
            c.bmin[2] = state.bounds_min[2] + ty * tile_world - c.borderSize * c.cs;
            c.bmax[0] = state.bounds_min[0] + (tx + 1) * tile_world + c.borderSize * c.cs;
            c.bmax[1] = state.bounds_max[1];
            c.bmax[2] = state.bounds_min[2] + (ty + 1) * tile_world + c.borderSize * c.cs;

            rcHeightfield *solid = rcAllocHeightfield();
            rcCreateHeightfield(ctx, *solid, c.width, c.height, c.bmin, c.bmax, c.cs, c.ch);

            float tbmin[2] = {c.bmin[0], c.bmin[2]};
            float tbmax[2] = {c.bmax[0], c.bmax[2]};
            int chunk_count = rcGetChunksOverlappingRect(&chunky, tbmin, tbmax, chunk_ids.data(), (int)chunk_ids.size());
            for (int i = 0; i < chunk_count; ++i) {
                const rcChunkyTriMeshNode &node = chunky.nodes[chunk_ids[i]];
                const int *tris = &chunky.tris[node.i * 3];
                memset(areas.data(), 0, areas.size());
                rcMarkWalkableTriangles(ctx, c.walkableSlopeAngle, vertices, vertex_count, tris, node.n, areas.data());
                rcRasterizeTriangles(ctx, vertices, vertex_count, tris, areas.data(), node.n, *solid, c.walkableClimb);
            }

            rcFilterLowHangingWalkableObstacles(ctx, c.walkableClimb, *solid);
            rcFilterLedgeSpans(ctx, c.walkableHeight, c.walkableClimb, *solid);
            rcFilterWalkableLowHeightSpans(ctx, c.walkableHeight, *solid);

            rcCompactHeightfield *chf = rcAllocCompactHeightfield();
            rcBuildCompactHeightfield(ctx, c.walkableHeight, c.walkableClimb, *solid, *chf);
            rcFreeHeightField(solid);
            rcErodeWalkableArea(ctx, c.walkableRadius, *chf);
            rcBuildDistanceField(ctx, *chf);
            rcBuildRegions(ctx, *chf, c.borderSize, c.minRegionArea, c.mergeRegionArea);

            rcContourSet *cset = rcAllocContourSet();
            rcBuildContours(ctx, *chf, c.maxSimplificationError, c.maxEdgeLen, *cset);
            rcPolyMesh *pmesh = rcAllocPolyMesh();
            rcBuildPolyMesh(ctx, *cset, c.maxVertsPerPoly, *pmesh);
            rcPolyMeshDetail *dmesh = rcAllocPolyMeshDetail();
            rcBuildPolyMeshDetail(ctx, *pmesh, *chf, c.detailSampleDist, c.detailSampleMaxError, *dmesh);

            if (pmesh->nverts > 0) {
                int size = 0;
                unsigned char *data = create_tile_data(*pmesh, *dmesh, tx, ty, &size);
                if (data && dtStatusFailed(nav_mesh->addTile(data, size, DT_TILE_FREE_DATA, 0, nullptr))) dtFree(data);
            }

            rcFreeCompactHeightfield(chf);
            rcFreeContourSet(cset);
            rcFreePolyMesh(pmesh);
            rcFreePolyMeshDetail(dmesh);
        }
    }

    dtFreeNavMesh(nav_mesh);
}

static const int QUERY_COUNT = 100;
static const int MAX_PATH = 256;

static int center_count() { return (int)state.centers.size() / 3; }
static const float *center(int i) { return &state.centers[(size_t)(i % center_count()) * 3]; }
static int pair_end(int i) { return (i * 37 + 11) % center_count(); }

static void run_find_path() {
    const float half_extents[3] = {1, 1, 1};
    dtPolyRef path[MAX_PATH];
    float straight[MAX_PATH * 3];
    unsigned char straight_flags[MAX_PATH];
    dtPolyRef straight_refs[MAX_PATH];
    for (int i = 0; i < QUERY_COUNT; i++) {
        const float *start = center(i);
        const float *end = center(pair_end(i));
        dtPolyRef start_ref = 0;
        dtPolyRef end_ref = 0;
        float start_point[3];
        float end_point[3];
        state.query->findNearestPoly(start, half_extents, &state.filter, &start_ref, start_point);
        state.query->findNearestPoly(end, half_extents, &state.filter, &end_ref, end_point);
        int path_count = 0;
        state.query->findPath(start_ref, end_ref, start_point, end_point, &state.filter, path, &path_count, MAX_PATH);
        int straight_count = 0;
        if (path_count > 0) {
            state.query->findStraightPath(start_point, end_point, path, path_count, straight, straight_flags, straight_refs, &straight_count,
                                          MAX_PATH);
        }
    }
}

static void run_path_ends() {
    const float half_extents[3] = {1, 1, 1};
    for (int i = 0; i < QUERY_COUNT; i++) {
        dtPolyRef ref = 0;
        float point[3];
        state.query->findNearestPoly(center(i), half_extents, &state.filter, &ref, point);
        state.query->findNearestPoly(center(pair_end(i)), half_extents, &state.filter, &ref, point);
    }
}

static void run_node_paths() {
    dtPolyRef path[MAX_PATH];
    for (int i = 0; i < QUERY_COUNT; i++) {
        int a = i % center_count();
        int b = pair_end(i);
        int path_count = 0;
        state.query->findPath(state.center_refs[(size_t)a], state.center_refs[(size_t)b], center(a), center(b), &state.filter, path, &path_count, MAX_PATH);
    }
}

static void run_raycast() {
    dtPolyRef path[MAX_PATH];
    for (int i = 0; i < QUERY_COUNT; i++) {
        int a = i % center_count();
        float t = 0;
        float normal[3];
        int path_count = 0;
        state.query->raycast(state.center_refs[(size_t)a], center(a), center(pair_end(i)), &state.filter, &t, normal, path, &path_count, MAX_PATH);
    }
}

static void run_find_nearest_poly() {
    const float half_extents[3] = {1, 2, 1};
    for (int i = 0; i < 2 * QUERY_COUNT; i++) {
        const float *c = center(i);
        float point[3] = {c[0] + 0.37f, c[1] + 0.5f, c[2] - 0.21f};
        dtPolyRef ref = 0;
        float nearest[3];
        state.query->findNearestPoly(point, half_extents, &state.filter, &ref, nearest);
    }
}

static void prepare_crowd() {
    state.crowd = dtAllocCrowd();
    state.crowd->init(100, 0.6f, state.nav_mesh);

    dtObstacleAvoidanceParams avoidance;
    avoidance.velBias = 0.4f;
    avoidance.weightDesVel = 2.0f;
    avoidance.weightCurVel = 0.75f;
    avoidance.weightSide = 0.75f;
    avoidance.weightToi = 2.5f;
    avoidance.horizTime = 2.5f;
    avoidance.gridSize = 33;
    avoidance.adaptiveDivs = 7;
    avoidance.adaptiveRings = 2;
    avoidance.adaptiveDepth = 5;
    state.crowd->setObstacleAvoidanceParams(0, &avoidance);

    dtCrowdAgentParams params;
    memset(&params, 0, sizeof(params));
    params.radius = 0.3f;
    params.height = 0.6f;
    params.maxAcceleration = 15;
    params.maxSpeed = 3.5f;
    params.collisionQueryRange = 2;
    params.pathOptimizationRange = 0.3f * 30;
    params.separationWeight = 0.5f;
    params.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_SEPARATION | DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_OPTIMIZE_TOPO | DT_CROWD_OPTIMIZE_VIS;
    params.obstacleAvoidanceType = 0;
    params.queryFilterType = 0;

    for (int i = 0; i < 100; i++) {
        int agent = state.crowd->addAgent(center(i), &params);
        int target = (i * 7 + 3) % center_count();
        if (agent >= 0) state.crowd->requestMoveTarget(agent, state.center_refs[(size_t)target], center(target));
    }
}

static void run_crowd() {
    for (int frame = 0; frame < 60; frame++) state.crowd->update(1.0f / 60.0f, nullptr);
}

static void finish_crowd() {
    dtFreeCrowd(state.crowd);
    state.crowd = nullptr;
}

int main(int argc, char **argv) {
    const char *fixture = nullptr;
    const char *points = nullptr;
    const char *filter = nullptr;
    double min_total_milliseconds = 500;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--fixture") == 0 && i + 1 < argc) {
            fixture = argv[++i];
        } else if (strcmp(argv[i], "--points") == 0 && i + 1 < argc) {
            points = argv[++i];
        } else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc) {
            min_total_milliseconds = atof(argv[++i]);
        } else if (strcmp(argv[i], "--cpu") == 0) {
            use_cpu_time = true;
        } else if (strcmp(argv[i], "--samples") == 0 && i + 1 < argc) {
            fixed_samples = atoi(argv[++i]);
        } else {
            filter = argv[i];
        }
    }
    if (!fixture || !points) {
        fprintf(stderr, "usage: recast_bench --fixture <fixture.json> --points <points.json> [--time ms] [filter]\n");
        return 1;
    }

    std::vector<char> fixture_text = read_file(fixture);
    std::vector<char> points_text = read_file(points);
    std::vector<double> values;
    if (fixture_text.empty() || !parse_array(fixture_text.data(), "\"positions\"", values)) return 1;
    for (double value : values) state.vertices.push_back((float)value);
    values.clear();
    parse_array(fixture_text.data(), "\"indices\"", values);
    for (double value : values) state.triangles.push_back((int)value);
    values.clear();
    if (points_text.empty() || !parse_array(points_text.data(), nullptr, values)) return 1;
    for (double value : values) state.centers.push_back((float)value);

    rcCalcBounds(state.vertices.data(), (int)state.vertices.size() / 3, state.bounds_min, state.bounds_max);
    state.config = solo_config();

    state.nav_mesh = generate_solo();
    if (!state.nav_mesh) {
        fprintf(stderr, "recast_bench: solo nav mesh generation failed\n");
        return 1;
    }
    int poly_count = 0;
    const dtNavMesh *const_nav_mesh = state.nav_mesh;
    for (int i = 0; i < state.nav_mesh->getMaxTiles(); i++) {
        const dtMeshTile *tile = const_nav_mesh->getTile(i);
        if (tile && tile->header) poly_count += tile->header->polyCount;
    }
    fprintf(stderr, "recast_bench: solo nav mesh has %d polys\n", poly_count);

    state.query = dtAllocNavMeshQuery();
    state.query->init(state.nav_mesh, 2048);
    const float half_extents[3] = {1, 1, 1};
    for (int i = 0; i < center_count(); i++) {
        dtPolyRef ref = 0;
        float nearest[3];
        state.query->findNearestPoly(center(i), half_extents, &state.filter, &ref, nearest);
        state.center_refs.push_back(ref);
    }

    static const char *const STAGE_NAMES[10] = {
        "mark walkable triangles", "rasterize triangles", "filter heightfield", "build compact heightfield", "erode walkable area",
        "build distance field",    "build regions",       "build contours",     "build poly mesh",          "build poly mesh detail",
    };

    std::vector<BenchCase> cases;
    for (int stage = 0; stage < 10; stage++) cases.push_back({STAGE_NAMES[stage], prepare_stage, run_stage, finish_stage});
    cases.push_back({"generate solo nav mesh", nullptr, run_solo, nullptr});
    cases.push_back({"generate tiled nav mesh", nullptr, run_tiled, nullptr});
    cases.push_back({"find path (100 pairs)", nullptr, run_find_path, nullptr});
    cases.push_back({"path ends (200 nearest)", nullptr, run_path_ends, nullptr});
    cases.push_back({"node paths (100 pairs)", nullptr, run_node_paths, nullptr});
    cases.push_back({"raycast (100 rays)", nullptr, run_raycast, nullptr});
    cases.push_back({"find nearest poly (200 points)", nullptr, run_find_nearest_poly, nullptr});
    cases.push_back({"crowd (100 agents, 60 updates)", prepare_crowd, run_crowd, finish_crowd});

    printf("%-34s %12s %12s %8s\n", "bench", "median ms", "min ms", "samples");
    for (size_t i = 0; i < cases.size(); i++) {
        if (filter && !strstr(cases[i].name, filter)) continue;
        state.stage = i < 10 ? (int)i : 0;
        run_case(cases[i], min_total_milliseconds);
    }

    dtFreeNavMeshQuery(state.query);
    dtFreeNavMesh(state.nav_mesh);
    return 0;
}
