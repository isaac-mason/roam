#include "app/examples.h"
#include "app/nav_mesh_config.h"
#include "imgui.h"
#include "roam/debug/debug.h"
#include "roam/query/nav_mesh_api.h"

using namespace roam;

enum TiledHelper {
    TILED_HELPER_HEIGHTFIELD,
    TILED_HELPER_COMPACT_HEIGHTFIELD_SOLID,
    TILED_HELPER_COMPACT_HEIGHTFIELD_DISTANCES,
    TILED_HELPER_COMPACT_HEIGHTFIELD_REGIONS,
    TILED_HELPER_RAW_CONTOURS,
    TILED_HELPER_SIMPLIFIED_CONTOURS,
    TILED_HELPER_POLY_MESH,
    TILED_HELPER_POLY_MESH_DETAIL,
    TILED_HELPER_NAV_MESH_BV_TREE,
    TILED_HELPER_NAV_MESH,
    TILED_HELPER_NAV_MESH_LINKS,
    TILED_HELPER_NAV_MESH_PORTALS,
    TILED_HELPER_COUNT,
};

static const char *const TILED_HELPER_NAMES[TILED_HELPER_COUNT] = {
    "heightfield", "compact heightfield solid", "compact heightfield distances", "compact heightfield regions",
    "raw contours", "simplified contours", "poly mesh", "poly mesh detail",
    "nav mesh bv tree", "nav mesh", "nav mesh links", "nav mesh portals",
};

struct TiledNavMeshExample {
    NavMeshConfig config;
    bool generated;
    TiledNavMeshResult result;
    double generation_milliseconds;
    uint32_t tile_count;
    bool show[TILED_HELPER_COUNT];
    StaticDrawing drawings[TILED_HELPER_COUNT];
};

static TiledNavMeshExample tiled;

static DebugPrimitives create_tile_helper(int helper, uint32_t tile) {
    const TiledNavMeshIntermediates &intermediates = tiled.result.intermediates;
    switch (helper) {
        case TILED_HELPER_HEIGHTFIELD: return debug::create_heightfield_helper(intermediates.heightfield[tile]);
        case TILED_HELPER_COMPACT_HEIGHTFIELD_SOLID: return debug::create_compact_heightfield_solid_helper(intermediates.compact_heightfield[tile]);
        case TILED_HELPER_COMPACT_HEIGHTFIELD_DISTANCES:
            return debug::create_compact_heightfield_distances_helper(intermediates.compact_heightfield[tile]);
        case TILED_HELPER_COMPACT_HEIGHTFIELD_REGIONS: return debug::create_compact_heightfield_regions_helper(intermediates.compact_heightfield[tile]);
        case TILED_HELPER_RAW_CONTOURS: return debug::create_raw_contours_helper(intermediates.contour_set[tile]);
        case TILED_HELPER_SIMPLIFIED_CONTOURS: return debug::create_simplified_contours_helper(intermediates.contour_set[tile]);
        case TILED_HELPER_POLY_MESH: return debug::create_poly_mesh_helper(intermediates.poly_mesh[tile]);
        case TILED_HELPER_POLY_MESH_DETAIL: return debug::create_poly_mesh_detail_helper(intermediates.poly_mesh_detail[tile]);
    }
    return {};
}

static DebugPrimitives create_nav_mesh_helper(int helper) {
    const NavMesh &nav_mesh = tiled.result.nav_mesh;
    switch (helper) {
        case TILED_HELPER_NAV_MESH_BV_TREE: return debug::create_nav_mesh_bv_tree_helper(nav_mesh);
        case TILED_HELPER_NAV_MESH: {
            DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
            app::lift(primitives, 0.1f);
            return primitives;
        }
        case TILED_HELPER_NAV_MESH_LINKS: return debug::create_nav_mesh_links_helper(nav_mesh);
        case TILED_HELPER_NAV_MESH_PORTALS: return debug::create_nav_mesh_portals_helper(nav_mesh);
    }
    return {};
}

static void update_drawing(int helper) {
    renderer::release(tiled.drawings[helper]);
    if (!tiled.generated || !tiled.show[helper]) return;

    DrawList list = {};
    if (helper < TILED_HELPER_NAV_MESH_BV_TREE) {
        for (uint32_t tile = 0; tile < tiled.result.intermediates.poly_mesh.length; tile++) {
            DebugPrimitives primitives = create_tile_helper(helper, tile);
            renderer::add_primitives(list, primitives);
            debug::release(primitives);
        }
    } else {
        DebugPrimitives primitives = create_nav_mesh_helper(helper);
        renderer::add_primitives(list, primitives);
        debug::release(primitives);
    }
    tiled.drawings[helper] = renderer::create_static(list);
    renderer::release(list);
}

static void release_result() {
    if (!tiled.generated) return;
    generate_tiled_nav_mesh::release(tiled.result.intermediates);
    nav_mesh_api::release(tiled.result.nav_mesh);
    tiled.generated = false;
}

static void generate(App &app) {
    release_result();
    const Model &level = app.level;
    double start = app::now_milliseconds();
    tiled.result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(
        {level.walkable_positions.data, level.walkable_indices.data, level.walkable_indices.length}, nav_mesh_config::tiled_options(tiled.config));
    tiled.generation_milliseconds = app::now_milliseconds() - start;
    tiled.generated = true;

    tiled.tile_count = 0;
    for (const NavMeshTile &tile : tiled.result.nav_mesh.tiles) tiled.tile_count += tile.allocated ? 1 : 0;

    for (int i = 0; i < TILED_HELPER_COUNT; i++) update_drawing(i);
}

static void init(App &app) {
    tiled = {};
    tiled.config = nav_mesh_config::create();
    tiled.show[TILED_HELPER_NAV_MESH] = true;
    tiled.show[TILED_HELPER_NAV_MESH_PORTALS] = true;
    app::load_level(app, "nav-test.glb");
    generate(app);
}

static void update(App &app, float) {
    for (int i = 0; i < TILED_HELPER_COUNT; i++) {
        if (tiled.show[i]) app::draw_static(app, tiled.drawings[i]);
    }
}

static void ui(App &app) {
    ImGui::Text("generated %u tiles in %.2f ms", tiled.tile_count, tiled.generation_milliseconds);
    if (ImGui::Button("generate")) generate(app);
    nav_mesh_config::ui(tiled.config, true);
    if (ImGui::CollapsingHeader("Debug helpers", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < TILED_HELPER_COUNT; i++) {
            if (ImGui::Checkbox(TILED_HELPER_NAMES[i], &tiled.show[i])) update_drawing(i);
        }
    }
}

static void shutdown(App &) {
    for (StaticDrawing &drawing : tiled.drawings) renderer::release(drawing);
    release_result();
}

const Example EXAMPLE_TILED_NAV_MESH = {"tiled nav mesh", init, update, ui, shutdown};
