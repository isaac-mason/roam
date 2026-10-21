#include <float.h>
#include <math.h>

#include "app/examples.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/math/scalar.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

static const uint8_t SIMPLEX_PERMUTATION[256] = {
    151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,   225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240,
    21,  10,  23,  190, 6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203, 117, 35,  11,  32,  57,  177, 33,  88,
    237, 149, 56,  87,  174, 20,  125, 136, 171, 168, 68,  175, 74,  165, 71,  134, 139, 48,  27,  166, 77,  146, 158, 231, 83,  111,
    229, 122, 60,  211, 133, 230, 220, 105, 92,  41,  55,  46,  245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,
    209, 76,  132, 187, 208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,  164, 100, 109, 198, 173, 186, 3,   64,  52,  217,
    226, 250, 124, 123, 5,   202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,  58,  17,  182, 189, 28,  42,
    223, 183, 170, 213, 119, 248, 152, 2,   44,  154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253, 19,  98,
    108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228, 251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241,
    81,  51,  145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157, 184, 84,  204, 176, 115, 121, 50,  45,  127, 4,
    150, 254, 138, 236, 205, 93,  222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180,
};

static const int SIMPLEX_GRADIENTS[12][2] = {
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}, {1, 0}, {-1, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0, 1}, {0, -1},
};

struct Simplex2D {
    int perm[512];
    int gradient[512];
};

static Simplex2D create_simplex_2d(int seed) {
    if (seed < 256) seed |= seed << 8;
    Simplex2D simplex = {};
    for (int i = 0; i < 256; i++) {
        int value = (i & 1) ? SIMPLEX_PERMUTATION[i] ^ (seed & 255) : SIMPLEX_PERMUTATION[i] ^ ((seed >> 8) & 255);
        simplex.perm[i] = simplex.perm[i + 256] = value;
        simplex.gradient[i] = simplex.gradient[i + 256] = value % 12;
    }
    return simplex;
}

static double simplex_2d(const Simplex2D &simplex, double x_in, double y_in) {
    const double f2 = 0.5 * (sqrt(3.0) - 1);
    const double g2 = (3 - sqrt(3.0)) / 6;
    double s = (x_in + y_in) * f2;
    int i = (int)floor(x_in + s);
    int j = (int)floor(y_in + s);
    double t = (i + j) * g2;
    double x0 = x_in - i + t;
    double y0 = y_in - j + t;
    int i1 = x0 > y0 ? 1 : 0;
    int j1 = x0 > y0 ? 0 : 1;
    double x1 = x0 - i1 + g2;
    double y1 = y0 - j1 + g2;
    double x2 = x0 - 1 + 2 * g2;
    double y2 = y0 - 1 + 2 * g2;
    i &= 255;
    j &= 255;
    const int *gi0 = SIMPLEX_GRADIENTS[simplex.gradient[i + simplex.perm[j]]];
    const int *gi1 = SIMPLEX_GRADIENTS[simplex.gradient[i + i1 + simplex.perm[j + j1]]];
    const int *gi2 = SIMPLEX_GRADIENTS[simplex.gradient[i + 1 + simplex.perm[j + 1]]];
    auto corner = [](double tc, const int *gradient, double x, double y) {
        if (tc < 0) return 0.0;
        tc *= tc;
        return tc * tc * (gradient[0] * x + gradient[1] * y);
    };
    double n0 = corner(0.5 - x0 * x0 - y0 * y0, gi0, x0, y0);
    double n1 = corner(0.5 - x1 * x1 - y1 * y1, gi1, x1, y1);
    double n2 = corner(0.5 - x2 * x2 - y2 * y2, gi2, x2, y2);
    return 70 * (n0 + n1 + n2);
}

static float next_mulberry32(uint32_t &state) {
    state += 1831565813u;
    uint32_t t = state;
    t = (t ^ (t >> 15)) * (t | 1);
    t ^= t + (t ^ (t >> 7)) * (t | 61);
    return (float)((double)(t ^ (t >> 14)) / 4294967296.0);
}

static const float TERRAIN_SIZE = 50;
static const int TERRAIN_SEGMENTS = 128;
static const float MAX_HEIGHT = 2;
static const int HOUSE_COUNT = 30;
static const float GRID_SIZE = 2;

struct House {
    Vec3 position;
    float scale;
};

struct CustomNavMeshGenerationExample {
    Array<float> terrain_positions;
    Array<uint32_t> terrain_indices;
    House houses[HOUSE_COUNT];
    Array<float> walkable_points;
    Array<uint32_t> indices;
    NavMesh nav_mesh;
    bool generated;
    double generation_milliseconds;
    bool show_nav_mesh;
    bool show_walkable_points;
    bool show_triangulation;
    StaticDrawing nav_mesh_drawing;
    StaticDrawing walkable_points_drawing;
    StaticDrawing triangulation_drawing;
    StaticDrawing path_drawing;
    Vec3 start;
    Vec3 end;
    bool success;
    bool partial;
    uint32_t point_count;
    float previous_fov_y;
    SearchWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
};

static CustomNavMeshGenerationExample custom_generation;

static bool vertical_triangle_hit(Vec3 a, Vec3 b, Vec3 c, float x, float z, float &out_y) {
    float d = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
    if (fabsf(d) < 1e-12f) return false;
    float u = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / d;
    float v = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / d;
    float w = 1 - u - v;
    const float epsilon = -1e-5f;
    if (u < epsilon || v < epsilon || w < epsilon) return false;
    out_y = u * a.y + v * b.y + w * c.y;
    return true;
}

static bool terrain_height(float x, float z, float &out_y) {
    const CustomNavMeshGenerationExample &state = custom_generation;
    float segment = TERRAIN_SIZE / TERRAIN_SEGMENTS;
    int cell_x = (int)floorf((x + TERRAIN_SIZE / 2) / segment);
    int cell_z = (int)floorf((z + TERRAIN_SIZE / 2) / segment);
    bool found = false;
    float best = -FLT_MAX;
    for (int iz = cell_z - 1; iz <= cell_z + 1; iz++) {
        for (int ix = cell_x - 1; ix <= cell_x + 1; ix++) {
            if (ix < 0 || iz < 0 || ix >= TERRAIN_SEGMENTS || iz >= TERRAIN_SEGMENTS) continue;
            uint32_t base = (uint32_t)(iz * TERRAIN_SEGMENTS + ix) * 6;
            for (uint32_t k = base; k < base + 6; k += 3) {
                Vec3 a = vec3::load(state.terrain_positions.data, state.terrain_indices[k] * 3);
                Vec3 b = vec3::load(state.terrain_positions.data, state.terrain_indices[k + 1] * 3);
                Vec3 c = vec3::load(state.terrain_positions.data, state.terrain_indices[k + 2] * 3);
                float y;
                if (vertical_triangle_hit(a, b, c, x, z, y) && y > best) {
                    best = y;
                    found = true;
                }
            }
        }
    }
    out_y = best;
    return found;
}

static bool house_top(const House &house, float x, float z, float &out_y) {
    float half = house.scale / 2;
    if (x < house.position.x - half || x > house.position.x + half || z < house.position.z - half || z > house.position.z + half) return false;
    out_y = house.position.y + house.scale;
    return true;
}

static void generate_scene() {
    CustomNavMeshGenerationExample &state = custom_generation;
    Simplex2D simplex = create_simplex_2d(42);

    int row = TERRAIN_SEGMENTS + 1;
    float segment = TERRAIN_SIZE / TERRAIN_SEGMENTS;
    for (int iz = 0; iz < row; iz++) {
        for (int ix = 0; ix < row; ix++) {
            double x = ix * segment - TERRAIN_SIZE / 2;
            double z = iz * segment - TERRAIN_SIZE / 2;
            double y = 0;
            y += simplex_2d(simplex, x * 0.05, z * 0.05) * MAX_HEIGHT * 0.5;
            y += simplex_2d(simplex, x * 0.1, z * 0.1) * MAX_HEIGHT * 0.25;
            y += simplex_2d(simplex, x * 0.2, z * 0.2) * MAX_HEIGHT * 0.125;
            y += simplex_2d(simplex, x * 0.4, z * 0.4) * MAX_HEIGHT * 0.0625;
            y += simplex_2d(simplex, x * 0.8, z * 0.8) * MAX_HEIGHT * 0.03125;
            state.terrain_positions.push((float)x);
            state.terrain_positions.push((float)y);
            state.terrain_positions.push((float)z);
        }
    }
    for (int iz = 0; iz < TERRAIN_SEGMENTS; iz++) {
        for (int ix = 0; ix < TERRAIN_SEGMENTS; ix++) {
            uint32_t a = (uint32_t)(ix + row * iz);
            uint32_t b = (uint32_t)(ix + row * (iz + 1));
            uint32_t c = (uint32_t)(ix + 1 + row * (iz + 1));
            uint32_t d = (uint32_t)(ix + 1 + row * iz);
            const uint32_t triangles[6] = {a, b, d, b, c, d};
            for (uint32_t index : triangles) state.terrain_indices.push(index);
        }
    }

    uint32_t random_state = 42;
    for (House &house : state.houses) {
        house.position.x = (next_mulberry32(random_state) - 0.5f) * TERRAIN_SIZE * 0.9f;
        house.position.z = (next_mulberry32(random_state) - 0.5f) * TERRAIN_SIZE * 0.9f;
        float height = 0;
        if (!terrain_height(house.position.x, house.position.z, height)) height = 0;
        house.position.y = height - 0.2f;
        house.scale = 1 + next_mulberry32(random_state) * 2;
    }
}

static Model create_level_model() {
    const CustomNavMeshGenerationExample &state = custom_generation;
    Model level = {};
    level.bounds = box3::empty();

    auto push_vertex = [&](Vec3 p, Vec3 color) {
        level.positions.push(p.x);
        level.positions.push(p.y);
        level.positions.push(p.z);
        level.colors.push(color.x);
        level.colors.push(color.y);
        level.colors.push(color.z);
        level.bounds = box3::expand_by_point(level.bounds, p);
    };

    const Vec3 terrain_color = {0x22 / 255.0f, 0x88 / 255.0f, 0x22 / 255.0f};
    for (uint32_t i = 0; i < state.terrain_positions.length / 3; i++) push_vertex(vec3::load(state.terrain_positions.data, i * 3), terrain_color);
    for (uint32_t index : state.terrain_indices) level.indices.push(index);

    const Vec3 house_color = {0x88 / 255.0f, 0x88 / 255.0f, 0x88 / 255.0f};
    const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
    for (const House &house : state.houses) {
        uint32_t base = level.positions.length / 3;
        float half = house.scale / 2;
        for (int i = 0; i < 8; i++) {
            push_vertex({house.position.x + ((i & 1) ? half : -half), house.position.y + ((i & 2) ? house.scale : 0),
                         house.position.z + ((i & 4) ? half : -half)},
                        house_color);
        }
        for (const int *face : faces) {
            const int corners[6] = {face[0], face[1], face[2], face[0], face[2], face[3]};
            for (int corner : corners) level.indices.push(base + (uint32_t)corner);
        }
    }

    level.loaded = true;
    return level;
}

static void generate_nav_mesh() {
    CustomNavMeshGenerationExample &state = custom_generation;
    double start = app::now_milliseconds();

    const float bounds_min_x = -TERRAIN_SIZE / 2;
    const float bounds_min_z = -TERRAIN_SIZE / 2;
    const int nx = (int)ceilf(TERRAIN_SIZE / GRID_SIZE) + 1;
    const int nz = (int)ceilf(TERRAIN_SIZE / GRID_SIZE) + 1;

    Array<int32_t> vertex_index_grid = {};
    vertex_index_grid.resize((uint32_t)(nx * nz), -1);

    for (int ix = 0; ix < nx; ix++) {
        for (int iz = 0; iz < nz; iz++) {
            float x = bounds_min_x + ix * GRID_SIZE;
            float z = bounds_min_z + iz * GRID_SIZE;

            float walkable_y;
            if (!terrain_height(x, z, walkable_y)) continue;
            bool blocked = false;
            for (const House &house : state.houses) {
                float top;
                if (house_top(house, x, z, top) && top >= walkable_y) blocked = true;
            }
            if (blocked) continue;

            vertex_index_grid[(uint32_t)(ix * nz + iz)] = (int32_t)(state.walkable_points.length / 3);
            state.walkable_points.push(x);
            state.walkable_points.push(walkable_y);
            state.walkable_points.push(z);
        }
    }

    for (int ix = 0; ix < nx - 1; ix++) {
        for (int iz = 0; iz < nz - 1; iz++) {
            int32_t a = vertex_index_grid[(uint32_t)(ix * nz + iz)];
            int32_t b = vertex_index_grid[(uint32_t)((ix + 1) * nz + iz)];
            int32_t c = vertex_index_grid[(uint32_t)(ix * nz + iz + 1)];
            int32_t d = vertex_index_grid[(uint32_t)((ix + 1) * nz + iz + 1)];
            if (a < 0 || b < 0 || c < 0 || d < 0) continue;

            Vec3 pa = vec3::load(state.walkable_points.data, (uint32_t)a * 3);
            Vec3 pb = vec3::load(state.walkable_points.data, (uint32_t)b * 3);
            Vec3 pc = vec3::load(state.walkable_points.data, (uint32_t)d * 3);
            Vec3 normal = vec3::cross(vec3::subtract(pb, pa), vec3::subtract(pc, pa));

            const int32_t flipped[6] = {a, d, b, a, c, d};
            const int32_t unflipped[6] = {a, b, d, a, d, c};
            for (int32_t index : normal.y < 0 ? flipped : unflipped) state.indices.push((uint32_t)index);
        }
    }

    vertex_index_grid.release();
    state.generation_milliseconds = app::now_milliseconds() - start;

    Box3 bounds = box3::empty();
    for (uint32_t i = 0; i < state.walkable_points.length / 3; i++) bounds = box3::expand_by_point(bounds, vec3::load(state.walkable_points.data, i * 3));

    Array<ExternalPolygon> polys = {};
    for (uint32_t i = 0; i + 2 < state.indices.length; i += 3) polys.push({&state.indices[i], 3, 1, 0});

    NavMeshTilePolys tile_polys = nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polys.data, polys.length, state.walkable_points.data,
                                                                                 state.walkable_points.length / 3, 0, bounds);
    NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

    NavMeshTileParams tile_params = {
        .bounds = bounds,
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
        .cell_size = 0.2f,
        .cell_height = 0.2f,
        .walkable_height = 0.5f,
        .walkable_radius = 0.5f,
        .walkable_climb = 0.5f,
    };

    state.nav_mesh = nav_mesh_api::create_nav_mesh();
    state.nav_mesh.tile_width = bounds.max.x - bounds.min.x;
    state.nav_mesh.tile_height = bounds.max.z - bounds.min.z;
    state.nav_mesh.origin = bounds.min;
    nav_mesh_api::add_tile(state.nav_mesh, nav_mesh_api::build_tile(tile_params));
    state.generated = true;

    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail_mesh);
    polys.release();
}

static void create_drawings() {
    CustomNavMeshGenerationExample &state = custom_generation;

    DebugPrimitives nav_mesh = debug::create_nav_mesh_helper(state.nav_mesh);
    app::lift(nav_mesh, 0.4f);
    state.nav_mesh_drawing = renderer::create_static(nav_mesh);
    debug::release(nav_mesh);

    DrawList points = {};
    for (uint32_t i = 0; i < state.walkable_points.length / 3; i++) {
        renderer::point(points, vec3::load(state.walkable_points.data, i * 3), 0.05f, {0, 0, 1, 1});
    }
    state.walkable_points_drawing = renderer::create_static(points);
    renderer::release(points);

    DrawList triangulation = {};
    const Vec3 lift = {0, 0.2f, 0};
    for (uint32_t i = 0; i + 2 < state.indices.length; i += 3) {
        Vec3 a = vec3::add(vec3::load(state.walkable_points.data, state.indices[i] * 3), lift);
        Vec3 b = vec3::add(vec3::load(state.walkable_points.data, state.indices[i + 1] * 3), lift);
        Vec3 c = vec3::add(vec3::load(state.walkable_points.data, state.indices[i + 2] * 3), lift);
        renderer::triangle(triangulation, a, b, c, {0, 0, 1, 1});
        renderer::line(triangulation, a, b, {0, 0, 0, 1});
        renderer::line(triangulation, b, c, {0, 0, 0, 1});
        renderer::line(triangulation, c, a, {0, 0, 0, 1});
    }
    state.triangulation_drawing = renderer::create_static(triangulation);
    renderer::release(triangulation);
}

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void sphere(DrawList &list, Vec3 center, float radius, Color color) {
    const int rings = 8;
    const int segments = 12;
    auto at = [&](int ring, int segment) {
        float phi = (float)ring / rings * scalar::PI;
        float theta = (float)segment / segments * scalar::PI * 2;
        return Vec3{center.x + sinf(phi) * cosf(theta) * radius, center.y + cosf(phi) * radius, center.z + sinf(phi) * sinf(theta) * radius};
    };
    for (int ring = 0; ring < rings; ring++) {
        for (int segment = 0; segment < segments; segment++) {
            renderer::triangle(list, at(ring, segment), at(ring + 1, segment), at(ring + 1, segment + 1), color);
            renderer::triangle(list, at(ring, segment), at(ring + 1, segment + 1), at(ring, segment + 1), color);
        }
    }
}

static void update_path() {
    CustomNavMeshGenerationExample &state = custom_generation;
    renderer::release(state.path_drawing);

    DrawList list = {};
    flag(list, state.start, {0.13f, 0.59f, 0.95f, 1});
    flag(list, state.end, {0, 1, 0, 1});

    FindPathResult path = {};
    find_path::find_path(path, state.workspace, state.nav_mesh, state.start, state.end, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER,
                         state.points, MAX_PATH_POINTS, {.node_path = state.node_path, .max_node_path = MAX_NODE_PATH});
    state.success = path.success;
    state.partial = (path.straight_path_flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) != 0;
    state.point_count = path.point_count;

    DebugPrimitives nodes = debug::create_search_nodes_helper(state.workspace.nodes);
    renderer::add_primitives(list, nodes);
    debug::release(nodes);

    for (uint32_t n = 0; n < path.node_path_count; n++) {
        NodeRef node_ref = state.node_path[n];
        if (node::get_node_ref_type(node_ref) != NODE_TYPE_POLY) continue;
        DebugPrimitives poly = debug::create_nav_mesh_poly_helper(state.nav_mesh, node_ref);
        app::lift(poly, 0.15f);
        renderer::add_primitives(list, poly);
        debug::release(poly);
    }

    const Vec3 lift = {0, 0.2f, 0};
    for (uint32_t i = 0; i < path.point_count; i++) {
        Vec3 position = vec3::add(state.points[i].position, lift);
        sphere(list, position, 0.2f, {1, 0, 0, 1});
        if (i > 0) renderer::line(list, vec3::add(state.points[i - 1].position, lift), position, {1, 1, 0, 1});
    }

    state.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void init(App &app) {
    CustomNavMeshGenerationExample &state = custom_generation;
    state = {};
    state.workspace = nav_mesh_search::create_search_workspace();
    state.show_nav_mesh = true;
    state.start = {-6.1f, 0.3f, 5};
    state.end = {8.6f, -0.4f, -3.7f};

    generate_scene();
    app::set_level(app, create_level_model());
    custom_generation.previous_fov_y = app.camera.fov_y;
    app.camera.fov_y = 75.0f * scalar::PI / 180.0f;
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});

    generate_nav_mesh();
    create_drawings();
    update_path();
}

static void update(App &app, float) {
    CustomNavMeshGenerationExample &state = custom_generation;
    if (app.click.clicked && app.click.hit) {
        if (app.click.secondary || app.click.shift) {
            state.end = app.click.point;
        } else {
            state.start = app.click.point;
        }
        update_path();
    }
    if (state.show_nav_mesh) app::draw_static(app, state.nav_mesh_drawing);
    if (state.show_walkable_points) app::draw_static(app, state.walkable_points_drawing);
    if (state.show_triangulation) app::draw_static(app, state.triangulation_drawing);
    app::draw_static(app, state.path_drawing);
}

static void ui(App &) {
    CustomNavMeshGenerationExample &state = custom_generation;
    ImGui::TextWrapped("click to set the start, right click or shift click to set the end");
    ImGui::Text("generated in %.2f ms, %u points, %u triangles", state.generation_milliseconds, state.walkable_points.length / 3,
                state.indices.length / 3);
    ImGui::Text("%s%s, %u points", state.success ? "path found" : "no path", state.partial ? " (partial)" : "", state.point_count);
    if (ImGui::CollapsingHeader("Debug Views", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("nav mesh", &state.show_nav_mesh);
        ImGui::Checkbox("walkable points", &state.show_walkable_points);
        ImGui::Checkbox("triangulation", &state.show_triangulation);
    }
}

static void shutdown(App &app) {
    CustomNavMeshGenerationExample &state = custom_generation;
    renderer::release(state.nav_mesh_drawing);
    nav_mesh_search::release(state.workspace);
    renderer::release(state.walkable_points_drawing);
    renderer::release(state.triangulation_drawing);
    renderer::release(state.path_drawing);
    if (state.generated) nav_mesh_api::release_nav_mesh(state.nav_mesh);
    state.generated = false;
    state.terrain_positions.release();
    state.terrain_indices.release();
    state.walkable_points.release();
    state.indices.release();
    app.camera.fov_y = custom_generation.previous_fov_y;
}

const Example EXAMPLE_CUSTOM_NAV_MESH_GENERATION = {"custom nav mesh generation", init, update, ui, shutdown};
