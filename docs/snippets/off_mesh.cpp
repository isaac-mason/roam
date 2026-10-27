#include <stdio.h>

#include "snippets.h"
#include "roam/query/find_path.h"

namespace roam {

void run_off_mesh_snippets(Scene &scene) {
    NavMesh &nav_mesh = scene.nav_mesh;
    QueryWorkspace &workspace = scene.workspace;
    const QueryFilter &filter = scene.filter;

    /* SNIPPET_START: point_connection */
    // a jump up onto the platform: from a point on the floor to a point on top, one way. each
    // end attaches to the nearest poly within the radius, and attaches again as tiles come and go
    const OffMeshConnectionParams jump_params = {
        .start = {6.5f, 0, -3.8f},
        .end = {6.5f, 1, -6},
        .radius = 0.5f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
        .flags = 0xffffffff,
        .area = 0,
    };
    const uint32_t jump = nav_mesh_api::add_off_mesh_connection(nav_mesh, jump_params);
    if (!nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, jump)) {
        printf("an end found no poly\n");
    }
    /* SNIPPET_END: point_connection */

    /* SNIPPET_START: point_connection_path */
    // a path up takes the jump: the point where it leaves the floor is an off-mesh point, and
    // the next is where it lands
    StraightPathPoint points[32];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, {0, 0, -6}, {7, 1, -7}, {1, 1, 1}, filter,
                         points);
    for (uint32_t i = 0; i + 1 < path.point_count; i++) {
        if (points[i].type != NODE_TYPE_OFFMESH) continue;
        const Vec3 from = points[i].position;
        const Vec3 to = points[i + 1].position;
        printf("jump from %.2f %.2f %.2f", from.x, from.y, from.z);
        printf(" to %.2f %.2f %.2f\n", to.x, to.y, to.z);
    }
    /* SNIPPET_END: point_connection_path */

    /* SNIPPET_START: remove_connection */
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, jump);
    /* SNIPPET_END: remove_connection */

    /* SNIPPET_START: segment_connection */
    // the whole front edge of the platform as one wide link: from anywhere along the floor's
    // edge (start_a to start_b) up to the matching place along the top (end_a to end_b). it is
    // split into a piece per pair of polys it joins, and a path crosses where its funnel meets
    // it. inset keeps crossings this far from the ends
    const OffMeshSegmentConnectionParams ledge_params = {
        .start_a = {4.5f, 0, -4.4f},
        .start_b = {8.5f, 0, -4.4f},
        .end_a = {4.5f, 1, -5.6f},
        .end_b = {8.5f, 1, -5.6f},
        .radius = 0.6f,
        .inset = 0.3f,
        .direction = OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
        .flags = 0xffffffff,
        .area = 0,
        .align = false,
    };
    const uint32_t ledge = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, ledge_params);

    // a link from a centre line and a width, as Unity's are
    const OffMeshSegmentConnectionParams wide = nav_mesh_api::off_mesh_segment_connection_from_width(
        {6.5f, 0, -4.4f}, {6.5f, 1, -5.6f}, 4, 0.6f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL,
        0xffffffff, 0);
    /* SNIPPET_END: segment_connection */

    (void)wide;
    nav_mesh_api::remove_off_mesh_connection(nav_mesh, ledge);
}

}
