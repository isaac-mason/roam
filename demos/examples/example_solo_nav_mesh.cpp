#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum SoloHelper {
    SOLO_HELPER_TRIANGLE_AREA_IDS,
    SOLO_HELPER_HEIGHTFIELD,
    SOLO_HELPER_COMPACT_HEIGHTFIELD_SOLID,
    SOLO_HELPER_COMPACT_HEIGHTFIELD_DISTANCES,
    SOLO_HELPER_COMPACT_HEIGHTFIELD_REGIONS,
    SOLO_HELPER_RAW_CONTOURS,
    SOLO_HELPER_SIMPLIFIED_CONTOURS,
    SOLO_HELPER_POLY_MESH,
    SOLO_HELPER_POLY_MESH_DETAIL,
    SOLO_HELPER_NAV_MESH_BV_TREE,
    SOLO_HELPER_NAV_MESH,
    SOLO_HELPER_NAV_MESH_LINKS,
    SOLO_HELPER_COUNT,
};

static const char *const SOLO_HELPER_NAMES[SOLO_HELPER_COUNT] = {
    "triangle area ids", "heightfield",    "compact heightfield solid", "compact heightfield distances",
    "compact heightfield regions", "raw contours", "simplified contours", "poly mesh",
    "poly mesh detail", "nav mesh bv tree", "nav mesh", "nav mesh links",
};

struct SoloNavMeshExample {
    NavMeshConfig config;
    bool generated;
    SoloNavMeshResult result;
    double generation_milliseconds;
    bool show[SOLO_HELPER_COUNT];
    StaticDrawing drawings[SOLO_HELPER_COUNT];
};

static SoloNavMeshExample solo;

static DebugPrimitives create_helper(int helper) {
    const SoloNavMeshIntermediates &intermediates = solo.result.intermediates;
    const NavMesh &nav_mesh = solo.result.nav_mesh;
    switch (helper) {
        case SOLO_HELPER_TRIANGLE_AREA_IDS:
            return debug::create_triangle_area_ids_helper(intermediates.input.positions, intermediates.input.indices, intermediates.input.index_count,
                                                          intermediates.tri_area_ids.data);
        case SOLO_HELPER_HEIGHTFIELD: return debug::create_heightfield_helper(intermediates.heightfield);
        case SOLO_HELPER_COMPACT_HEIGHTFIELD_SOLID: return debug::create_compact_heightfield_solid_helper(intermediates.compact_heightfield);
        case SOLO_HELPER_COMPACT_HEIGHTFIELD_DISTANCES: return debug::create_compact_heightfield_distances_helper(intermediates.compact_heightfield);
        case SOLO_HELPER_COMPACT_HEIGHTFIELD_REGIONS: return debug::create_compact_heightfield_regions_helper(intermediates.compact_heightfield);
        case SOLO_HELPER_RAW_CONTOURS: return debug::create_raw_contours_helper(intermediates.contour_set);
        case SOLO_HELPER_SIMPLIFIED_CONTOURS: return debug::create_simplified_contours_helper(intermediates.contour_set);
        case SOLO_HELPER_POLY_MESH: return debug::create_poly_mesh_helper(intermediates.poly_mesh);
        case SOLO_HELPER_POLY_MESH_DETAIL: return debug::create_poly_mesh_detail_helper(intermediates.poly_mesh_detail);
        case SOLO_HELPER_NAV_MESH_BV_TREE: return debug::create_nav_mesh_bv_tree_helper(nav_mesh);
        case SOLO_HELPER_NAV_MESH: {
            DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
        case SOLO_HELPER_NAV_MESH_LINKS: return debug::create_nav_mesh_links_helper(nav_mesh);
    }
    return {};
}

static void update_drawing(int helper) {
    renderer::release(solo.drawings[helper]);
    if (!solo.generated || !solo.show[helper]) return;
    DebugPrimitives primitives = create_helper(helper);
    solo.drawings[helper] = renderer::create_static(primitives);
    debug::release(primitives);
}

static void release_result() {
    if (!solo.generated) return;
    generate_solo_nav_mesh::release(solo.result.intermediates);
    nav_mesh_api::release_nav_mesh(solo.result.nav_mesh);
    solo.generated = false;
}

static void generate(App &app) {
    release_result();
    const Model &level = app.level;
    double start = app::now_milliseconds();
    solo.result = generate_solo_nav_mesh::generate_solo_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::solo_options(solo.config));
    solo.generation_milliseconds = app::now_milliseconds() - start;
    solo.generated = true;
    for (int i = 0; i < SOLO_HELPER_COUNT; i++) update_drawing(i);
}

static void init(App &app) {
    solo = {};
    solo.config = nav_mesh_config::create();
    solo.show[SOLO_HELPER_NAV_MESH] = true;
    app::load_level(app, "nav-test.glb");
    generate(app);
}

static void update(App &app, float) {
    for (int i = 0; i < SOLO_HELPER_COUNT; i++) {
        if (solo.show[i]) app::draw_static(app, solo.drawings[i]);
    }
}

static void ui(App &app) {
    ImGui::Text("generated in %.2f ms", solo.generation_milliseconds);
    if (solo.generated) ImGui::Text("%d polys", solo.result.intermediates.poly_mesh.poly_count);
    if (ImGui::Button("generate")) generate(app);
    nav_mesh_config::ui(solo.config, false);
    if (ImGui::CollapsingHeader("Debug helpers", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < SOLO_HELPER_COUNT; i++) {
            if (ImGui::Checkbox(SOLO_HELPER_NAMES[i], &solo.show[i])) update_drawing(i);
        }
    }
}

static void shutdown(App &) {
    for (StaticDrawing &drawing : solo.drawings) renderer::release(drawing);
    release_result();
}

const Example EXAMPLE_SOLO_NAV_MESH = {"solo nav mesh", init, update, ui, shutdown};
