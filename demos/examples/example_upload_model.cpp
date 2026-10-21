#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/query/find_path.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum UploadHelper {
    UPLOAD_HELPER_TRIANGLE_AREA_IDS,
    UPLOAD_HELPER_HEIGHTFIELD,
    UPLOAD_HELPER_COMPACT_HEIGHTFIELD_SOLID,
    UPLOAD_HELPER_COMPACT_HEIGHTFIELD_DISTANCES,
    UPLOAD_HELPER_COMPACT_HEIGHTFIELD_REGIONS,
    UPLOAD_HELPER_RAW_CONTOURS,
    UPLOAD_HELPER_SIMPLIFIED_CONTOURS,
    UPLOAD_HELPER_POLY_MESH,
    UPLOAD_HELPER_POLY_MESH_DETAIL,
    UPLOAD_HELPER_NAV_MESH_BV_TREE,
    UPLOAD_HELPER_NAV_MESH,
    UPLOAD_HELPER_NAV_MESH_LINKS,
    UPLOAD_HELPER_NAV_MESH_PORTALS,
    UPLOAD_HELPER_COUNT,
};

static const char *const UPLOAD_HELPER_NAMES[UPLOAD_HELPER_COUNT] = {
    "triangle area ids", "heightfield",      "compact heightfield solid", "compact heightfield distances", "compact heightfield regions",
    "raw contours",      "simplified contours", "poly mesh",              "poly mesh detail",              "nav mesh bv tree",
    "nav mesh",          "nav mesh links",   "nav mesh portals",
};

enum UploadTool {
    UPLOAD_TOOL_NONE,
    UPLOAD_TOOL_PATHFINDING,
    UPLOAD_TOOL_QUERY,
};

struct UploadModelExample {
    char model_name[256];
    bool tiled;
    NavMeshConfig config;

    bool generated;
    bool generated_tiled;
    SoloNavMeshResult solo;
    TiledNavMeshResult tiled_result;
    double generation_milliseconds;
    uint32_t poly_count;
    char status[256];

    bool show[UPLOAD_HELPER_COUNT];
    StaticDrawing drawings[UPLOAD_HELPER_COUNT];

    int tool;
    Vec3 path_half_extents;
    bool show_search_nodes;
    bool has_start;
    bool has_end;
    Vec3 start;
    Vec3 end;
    StaticDrawing path_drawing;
    SearchWorkspace workspace;
    StraightPathPoint points[MAX_PATH_POINTS];
    NodeRef node_path[MAX_NODE_PATH];
    bool path_success;
    bool path_partial;
    uint32_t path_points;
    float path_length;
    double path_milliseconds;

    bool query_on_hover;
    Vec3 query_half_extents;
    bool query_done;
    Vec3 query_point;
    FindNearestPolyResult query_result;
    StaticDrawing query_drawing;
};

static UploadModelExample upload;

static const NavMesh &current_nav_mesh() { return upload.generated_tiled ? upload.tiled_result.nav_mesh : upload.solo.nav_mesh; }

static NavMeshConfig upload_config() {
    NavMeshConfig config = nav_mesh_config::create();
    config.cell_size = 0.15f;
    config.cell_height = 0.25f;
    config.walkable_radius_world = 0.3f;
    config.walkable_climb_world = 0.4f;
    config.walkable_height_world = 2;
    config.walkable_slope_angle_degrees = 45;
    config.tile_size_voxels = 32;
    return config;
}

static void add_helper(DrawList &list, DebugPrimitives primitives, float lift) {
    if (lift != 0) app::lift(primitives, lift);
    renderer::add_primitives(list, primitives);
    debug::release(primitives);
}

static DebugPrimitives create_stage_helper(int helper, const Array<uint8_t> *tri_area_ids, const Heightfield &solid,
                                           const CompactHeightfield &compact, const ContourSet &contours, const PolyMesh &polys,
                                           const PolyMeshDetail &detail, const App &app) {
    switch (helper) {
        case UPLOAD_HELPER_TRIANGLE_AREA_IDS:
            if (!tri_area_ids) return {};
            return debug::create_triangle_area_ids_helper(app.level.walkable_positions.data, app.level.walkable_indices.data,
                                                          app.level.walkable_indices.length, tri_area_ids->data);
        case UPLOAD_HELPER_HEIGHTFIELD: return debug::create_heightfield_helper(solid);
        case UPLOAD_HELPER_COMPACT_HEIGHTFIELD_SOLID: return debug::create_compact_heightfield_solid_helper(compact);
        case UPLOAD_HELPER_COMPACT_HEIGHTFIELD_DISTANCES: return debug::create_compact_heightfield_distances_helper(compact);
        case UPLOAD_HELPER_COMPACT_HEIGHTFIELD_REGIONS: return debug::create_compact_heightfield_regions_helper(compact);
        case UPLOAD_HELPER_RAW_CONTOURS: return debug::create_raw_contours_helper(contours);
        case UPLOAD_HELPER_SIMPLIFIED_CONTOURS: return debug::create_simplified_contours_helper(contours);
        case UPLOAD_HELPER_POLY_MESH: return debug::create_poly_mesh_helper(polys);
        case UPLOAD_HELPER_POLY_MESH_DETAIL: return debug::create_poly_mesh_detail_helper(detail);
    }
    return {};
}

static void update_drawing(const App &app, int helper) {
    renderer::release(upload.drawings[helper]);
    if (!upload.generated || !upload.show[helper]) return;

    DrawList list = {};
    const NavMesh &nav_mesh = current_nav_mesh();
    if (helper == UPLOAD_HELPER_NAV_MESH_BV_TREE) {
        add_helper(list, debug::create_nav_mesh_bv_tree_helper(nav_mesh), 0);
    } else if (helper == UPLOAD_HELPER_NAV_MESH) {
        add_helper(list, debug::create_nav_mesh_helper(nav_mesh), 0.1f);
    } else if (helper == UPLOAD_HELPER_NAV_MESH_LINKS) {
        add_helper(list, debug::create_nav_mesh_links_helper(nav_mesh), 0);
    } else if (helper == UPLOAD_HELPER_NAV_MESH_PORTALS) {
        if (upload.generated_tiled) add_helper(list, debug::create_nav_mesh_portals_helper(nav_mesh), 0);
    } else if (upload.generated_tiled) {
        const TiledNavMeshIntermediates &intermediates = upload.tiled_result.intermediates;
        if (helper == UPLOAD_HELPER_TRIANGLE_AREA_IDS) {
            Array<uint8_t> tri_area_ids = {};
            tri_area_ids.resize(app.level.walkable_indices.length / 3, 0);
            input_triangle_mesh::mark_walkable_triangles(app.level.walkable_positions.data, app.level.walkable_indices.data,
                                                         app.level.walkable_indices.length, tri_area_ids.data, upload.config.walkable_slope_angle_degrees);
            add_helper(list,
                       debug::create_triangle_area_ids_helper(app.level.walkable_positions.data, app.level.walkable_indices.data,
                                                              app.level.walkable_indices.length, tri_area_ids.data),
                       0);
            tri_area_ids.release();
        } else {
            for (uint32_t tile = 0; tile < intermediates.poly_mesh.length; tile++) {
                add_helper(list,
                           create_stage_helper(helper, nullptr, intermediates.heightfield[tile], intermediates.compact_heightfield[tile],
                                               intermediates.contour_set[tile], intermediates.poly_mesh[tile], intermediates.poly_mesh_detail[tile], app),
                           0);
            }
        }
    } else {
        const SoloNavMeshIntermediates &intermediates = upload.solo.intermediates;
        add_helper(list,
                   create_stage_helper(helper, &intermediates.tri_area_ids, intermediates.heightfield,
                                       intermediates.compact_heightfield, intermediates.contour_set, intermediates.poly_mesh,
                                       intermediates.poly_mesh_detail, app),
                   0);
    }

    upload.drawings[helper] = renderer::create_static(list);
    renderer::release(list);
}

static void clear_path() {
    renderer::release(upload.path_drawing);
    upload.has_start = false;
    upload.has_end = false;
    upload.path_points = 0;
}

static void clear_query() {
    renderer::release(upload.query_drawing);
    upload.query_done = false;
}

static void release_result() {
    if (!upload.generated) return;
    if (upload.generated_tiled) {
        generate_tiled_nav_mesh::release(upload.tiled_result.intermediates);
        nav_mesh_api::release_nav_mesh(upload.tiled_result.nav_mesh);
    } else {
        generate_solo_nav_mesh::release(upload.solo.intermediates);
        nav_mesh_api::release_nav_mesh(upload.solo.nav_mesh);
    }
    upload.generated = false;
}

static void flag(DrawList &list, Vec3 position, Color color) {
    renderer::box(list, {position.x, position.y + 0.6f, position.z}, {0.06f, 0.6f, 0.06f}, {0.53f, 0.53f, 0.53f, 1});
    renderer::box(list, {position.x + 0.23f, position.y + 1.0f, position.z}, {0.16f, 0.11f, 0.02f}, color);
}

static void update_path() {
    renderer::release(upload.path_drawing);
    upload.path_points = 0;
    if (!upload.generated || (!upload.has_start && !upload.has_end)) return;

    DrawList list = {};
    if (upload.has_start) flag(list, upload.start, {0.13f, 0.59f, 0.95f, 1});
    if (upload.has_end) flag(list, upload.end, {0, 1, 0, 1});

    if (upload.has_start && upload.has_end) {
        const NavMesh &nav_mesh = current_nav_mesh();
        double begin = app::now_milliseconds();
        FindPathResult path = {};
        find_path::find_path(path, upload.workspace, nav_mesh, upload.start, upload.end, upload.path_half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER,
                             upload.points, MAX_PATH_POINTS, {.node_path = upload.node_path, .max_node_path = MAX_NODE_PATH});
        upload.path_milliseconds = app::now_milliseconds() - begin;
        upload.path_success = path.success;
        upload.path_partial = (path.straight_path_flags & FIND_STRAIGHT_PATH_RESULT_PARTIAL_PATH) != 0;
        upload.path_points = path.point_count;
        upload.path_length = 0;

        if (upload.show_search_nodes) add_helper(list, debug::create_search_nodes_helper(upload.workspace.nodes), 0);

        for (uint32_t n = 0; n < path.node_path_count; n++) {
            NodeRef node_ref = upload.node_path[n];
            if (node::get_node_ref_type(node_ref) == NODE_TYPE_POLY) add_helper(list, debug::create_nav_mesh_poly_helper(nav_mesh, node_ref), 0.15f);
        }

        for (uint32_t i = 0; i < path.point_count; i++) {
            renderer::point(list, upload.points[i].position, 0.2f, {1, 0, 0, 1});
            if (i == 0) continue;
            renderer::thick_line(list, upload.points[i - 1].position, upload.points[i].position, 0.1f, {1, 1, 0, 1});
            upload.path_length += vec3::distance(upload.points[i - 1].position, upload.points[i].position);
        }
    }

    upload.path_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void update_query(Vec3 point) {
    renderer::release(upload.query_drawing);
    if (!upload.generated) return;

    upload.query_done = true;
    upload.query_point = point;
    upload.query_result = {};
    nav_mesh_api::find_nearest_poly(upload.query_result, current_nav_mesh(), point, upload.query_half_extents, nav_mesh_api::DEFAULT_QUERY_FILTER);
    if (!upload.query_result.success) return;

    DrawList list = {};
    add_helper(list, debug::create_nav_mesh_poly_helper(current_nav_mesh(), upload.query_result.node_ref), 0.1f);
    upload.query_drawing = renderer::create_static(list);
    renderer::release(list);
}

static void generate(App &app) {
    if (!app.level.loaded) {
        snprintf(upload.status, sizeof(upload.status), "load a model first");
        return;
    }
    release_result();
    clear_path();
    clear_query();

    const Model &level = app.level;
    double begin = app::now_milliseconds();
    upload.generated_tiled = upload.tiled;
    if (upload.tiled) {
        upload.tiled_result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
            {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(upload.config));
    } else {
        upload.solo = generate_solo_nav_mesh::generate_solo_nav_mesh(
            {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::solo_options(upload.config));
    }
    upload.generation_milliseconds = app::now_milliseconds() - begin;
    upload.generated = true;

    upload.poly_count = 0;
    for (const NavMeshTile &tile : current_nav_mesh().tiles) upload.poly_count += tile.allocated ? tile.polys.length : 0;
    snprintf(upload.status, sizeof(upload.status), "generated %u polys in %.1f ms", upload.poly_count, upload.generation_milliseconds);

    for (int i = 0; i < UPLOAD_HELPER_COUNT; i++) update_drawing(app, i);
}

static void use_model(App &app, Model model, const char *name) {
    if (!model.loaded) {
        model::release(model);
        snprintf(upload.status, sizeof(upload.status), "could not load %s", name);
        return;
    }
    snprintf(upload.model_name, sizeof(upload.model_name), "%s", name);
    app::set_level(app, model);
    generate(app);
}

static const char *file_name(const char *path) {
    const char *slash = strrchr(path, '/');
    const char *backslash = strrchr(path, '\\');
    if (backslash && (!slash || backslash > slash)) slash = backslash;
    return slash ? slash + 1 : path;
}

static void init(App &app) {
    upload = {};
    upload.workspace = nav_mesh_search::create_search_workspace();
    upload.config = upload_config();
    upload.show[UPLOAD_HELPER_NAV_MESH] = true;
    upload.tool = UPLOAD_TOOL_PATHFINDING;
    upload.path_half_extents = {1, 1, 1};
    upload.query_half_extents = {1, 1, 1};
    snprintf(upload.status, sizeof(upload.status), "no model loaded");

    app::set_level(app, Model{});
    camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});
    app.want_hover = true;
}

static void update(App &app, float) {
    if (app.dropped.ready) {
        use_model(app, model::load_from_memory(app.dropped.data.data, app.dropped.data.length, app.dropped.path), file_name(app.dropped.path));
    } else if (app.dropped.failed) {
        snprintf(upload.status, sizeof(upload.status), "could not read the dropped file");
    }

    if (app.click.clicked && app.click.hit && upload.generated) {
        if (upload.tool == UPLOAD_TOOL_PATHFINDING) {
            if (app.click.secondary || app.click.shift) {
                upload.end = app.click.point;
                upload.has_end = true;
            } else {
                upload.start = app.click.point;
                upload.has_start = true;
            }
            update_path();
        } else if (upload.tool == UPLOAD_TOOL_QUERY && !upload.query_on_hover) {
            update_query(app.click.point);
        }
    }
    if (upload.tool == UPLOAD_TOOL_QUERY && upload.query_on_hover && app.hover.hit) update_query(app.hover.point);

    DrawList &list = app::frame(app);
    if (!app.level.loaded) {
        const int half = 10;
        for (int i = -half; i <= half; i++) {
            Color color = i == 0 ? Color{0.4f, 0.4f, 0.4f, 1} : Color{0.25f, 0.25f, 0.25f, 1};
            renderer::line(list, {(float)i, 0, (float)-half}, {(float)i, 0, (float)half}, color);
            renderer::line(list, {(float)-half, 0, (float)i}, {(float)half, 0, (float)i}, color);
        }
    }

    for (int i = 0; i < UPLOAD_HELPER_COUNT; i++) {
        if (upload.show[i]) app::draw_static(app, upload.drawings[i]);
    }
    if (upload.tool == UPLOAD_TOOL_PATHFINDING) app::draw_static(app, upload.path_drawing);
    if (upload.tool == UPLOAD_TOOL_QUERY) app::draw_static(app, upload.query_drawing);
}

static void ui(App &app) {
    ImGui::TextWrapped("drop a .glb or .gltf file onto the window, then generate and explore its nav mesh");
    ImGui::TextWrapped("%s", upload.status);
    if (upload.model_name[0]) ImGui::Text("model: %s", upload.model_name);

    if (!app.level.loaded && ImGui::Button("load sample (nav-test.glb)")) use_model(app, model::load("nav-test.glb"), "nav-test.glb");

    if (ImGui::Button("generate")) generate(app);
    ImGui::SameLine();
    if (ImGui::Button("reset camera")) {
        if (app.level.loaded) {
            camera::frame_bounds(app.camera, app.level.bounds);
        } else {
            camera::look_at(app.camera, {-2, 10, 10}, {0, 0, 0});
        }
    }

    if (ImGui::CollapsingHeader("Nav mesh type", ImGuiTreeNodeFlags_DefaultOpen)) {
        int type = upload.tiled ? 1 : 0;
        ImGui::RadioButton("solo", &type, 0);
        ImGui::SameLine();
        ImGui::RadioButton("tiled", &type, 1);
        upload.tiled = type == 1;
    }
    nav_mesh_config::ui(upload.config, upload.tiled);

    if (ImGui::CollapsingHeader("Debug helpers")) {
        for (int i = 0; i < UPLOAD_HELPER_COUNT; i++) {
            if (i == UPLOAD_HELPER_NAV_MESH_PORTALS && !upload.tiled) continue;
            if (ImGui::Checkbox(UPLOAD_HELPER_NAMES[i], &upload.show[i])) update_drawing(app, i);
        }
    }

    if (!upload.generated) return;

    if (ImGui::CollapsingHeader("Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::RadioButton("none", &upload.tool, UPLOAD_TOOL_NONE);
        ImGui::SameLine();
        ImGui::RadioButton("pathfinding", &upload.tool, UPLOAD_TOOL_PATHFINDING);
        ImGui::SameLine();
        ImGui::RadioButton("query", &upload.tool, UPLOAD_TOOL_QUERY);

        if (upload.tool == UPLOAD_TOOL_PATHFINDING) {
            ImGui::TextWrapped("click to set the start, right click or shift click to set the end");
            bool changed = ImGui::SliderFloat3("half extents", &upload.path_half_extents.x, 0.1f, 5, "%.1f");
            changed |= ImGui::Checkbox("show search nodes", &upload.show_search_nodes);
            if (changed) update_path();
            if (ImGui::Button("clear path")) clear_path();
            if (upload.has_start && upload.has_end) {
                ImGui::Text("%s, %u waypoints, length %.2f", upload.path_success ? "found" : "failed", upload.path_points, upload.path_length);
                ImGui::Text("partial %s, %.2f ms", upload.path_partial ? "yes" : "no", upload.path_milliseconds);
            }
        } else if (upload.tool == UPLOAD_TOOL_QUERY) {
            ImGui::Checkbox("query on hover", &upload.query_on_hover);
            ImGui::SliderFloat3("half extents##query", &upload.query_half_extents.x, 0.1f, 5, "%.1f");
            if (upload.query_done) {
                const Vec3 &p = upload.query_point;
                ImGui::Text("point %.2f, %.2f, %.2f", p.x, p.y, p.z);
                if (upload.query_result.success) {
                    const Vec3 &n = upload.query_result.position;
                    ImGui::Text("nearest %.2f, %.2f, %.2f", n.x, n.y, n.z);
                    ImGui::Text("distance %.3f, ref %llu", vec3::distance(p, n), (unsigned long long)upload.query_result.node_ref);
                } else {
                    ImGui::Text("nearest not found");
                }
            }
        }
    }
}

static void shutdown(App &) {
    for (StaticDrawing &drawing : upload.drawings) renderer::release(drawing);
    renderer::release(upload.path_drawing);
    renderer::release(upload.query_drawing);
    nav_mesh_search::release(upload.workspace);
    release_result();
}

const Example EXAMPLE_UPLOAD_MODEL = {"upload model", init, update, ui, shutdown};
