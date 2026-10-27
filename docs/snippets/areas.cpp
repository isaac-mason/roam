#include "areas.h"
#include "snippets.h"
#include "roam/query/find_path.h"

namespace roam {

/* SNIPPET_START: area_cost */
struct AreaCosts {
    float ground;
    float water;
};

// a filter's cost is for the step from pa to pb across current_ref: here the distance, scaled
// by its area's cost
float area_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef,
                NodeRef current_ref, NodeRef) {
    const AreaCosts &costs = *static_cast<const AreaCosts *>(filter.user_data);
    const uint8_t area = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref).area;
    return vec3::distance(pa, pb) * (area == AREA_WATER ? costs.water : costs.ground);
}
/* SNIPPET_END: area_cost */

void run_area_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;

    /* SNIPPET_START: flag_filters */
    // a poly passes when its flags share a bit with include_flags and none with exclude_flags
    QueryFilter walkers = nav_mesh_api::create_default_query_filter();
    walkers.include_flags = FLAG_WALK;
    QueryFilter swimmers = nav_mesh_api::create_default_query_filter();
    swimmers.include_flags = FLAG_WALK | FLAG_SWIM;
    /* SNIPPET_END: flag_filters */

    /* SNIPPET_START: cost_filter */
    // swimming is allowed but costs four times walking: paths go round the water where that is
    // shorter
    AreaCosts costs = {.ground = 1, .water = 4};
    QueryFilter careful = {
        .pass_filter = nav_mesh_api::pass_flags,
        .get_cost = area_cost,
        .include_flags = FLAG_WALK | FLAG_SWIM,
        .exclude_flags = 0,
        .user_data = &costs,
    };

    StraightPathPoint points[64];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, {-9, 0, 2}, {-3, 0, 9}, {1, 1, 1}, careful,
                         points);
    /* SNIPPET_END: cost_filter */

    (void)walkers;
    (void)swimmers;
}

}
