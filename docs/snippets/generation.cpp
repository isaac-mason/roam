#include <math.h>
#include <stdio.h>

#include "areas.h"
#include "snippets.h"
#include "roam/blocks/generators/generate_tiled_nav_mesh.h"
#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"

namespace roam {

static void solo_snippets(Scene &scene) {
    Level &level = scene.level;

    /* SNIPPET_START: solo */
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    const SoloNavMeshOptions options = generate_solo_nav_mesh::derive_sizes({
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_radius_world = 0.3f,
        .walkable_climb_world = 0.4f,
        .walkable_height_world = 1.8f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 0,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 6 * cell_size,
        .detail_sample_max_error = 1 * cell_height,
    });

    // the mesh is borrowed for the call: spans of the level's positions and indices
    const InputTriangleMesh mesh = {level.positions, level.indices};
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(mesh, options);

    // every stage's output is kept: for debug drawing, or to look into a bad build
    const SoloNavMeshIntermediates &intermediates = result.intermediates;
    printf("%d polys\n", intermediates.poly_mesh.poly_count);
    for (const BuildContextLog &log : intermediates.build_context.logs) printf("%s\n", log.message);

    // the nav mesh and the intermediates are released separately, so the nav mesh can outlive
    // them
    NavMesh nav_mesh = result.nav_mesh;
    generate_solo_nav_mesh::release(result.intermediates);
    /* SNIPPET_END: solo */

    nav_mesh_api::release(nav_mesh);
}

static void tiled_snippets(Scene &scene) {
    Level &level = scene.level;

    /* SNIPPET_START: tiled */
    // the same settings as a solo build, plus the tile size in voxels (derive_sizes also sets
    // tile_size_world). the border is the agent radius and a few voxels more, so each tile sees
    // the geometry just past its edges and neighbouring tiles meet cleanly
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    const TiledNavMeshOptions options = generate_tiled_nav_mesh::derive_sizes({
        .cell_size = cell_size,
        .cell_height = cell_height,
        .tile_size_voxels = 32,
        .walkable_radius_world = 0.3f,
        .walkable_climb_world = 0.4f,
        .walkable_height_world = 1.8f,
        .walkable_slope_angle_degrees = 45,
        .border_size = 2 + 3,
        .min_region_area = 8,
        .merge_region_area = 20,
        .max_simplification_error = 1.3f,
        .max_edge_length = 12,
        .max_vertices_per_poly = 6,
        .detail_sample_distance = 6 * cell_size,
        .detail_sample_max_error = 1 * cell_height,
    });

    const InputTriangleMesh mesh = {level.positions, level.indices};
    TiledNavMeshResult result = generate_tiled_nav_mesh::generate_tiled_nav_mesh(mesh, options);
    generate_tiled_nav_mesh::release(result.intermediates);
    NavMesh nav_mesh = result.nav_mesh;

    // tiles sit on a grid from the nav mesh's origin, tile_width by tile_height
    for (const NavMeshTile &tile : nav_mesh.tiles) {
        if (!tile.allocated) continue;
        printf("tile %d, %d: %u polys\n", tile.tile_x, tile.tile_y, tile.polys.length);
    }
    /* SNIPPET_END: tiled */

    nav_mesh_api::release(nav_mesh);
}

static void deep_dive_snippets(Scene &scene) {
    Level &level = scene.level;

    /* SNIPPET_START: input */
    // borrowed: generation only reads the level's positions and indices
    const InputTriangleMesh mesh = {level.positions, level.indices};

    // the build's log: each step that can fail writes why here
    BuildContext context = {};
    /* SNIPPET_END: input */

    /* SNIPPET_START: walkable_triangles */
    // CONFIG: agent walkable slope angle
    const float walkable_slope_angle_degrees = 45;

    // one area id per triangle: WALKABLE_AREA where the slope is walkable, NULL_AREA elsewhere
    Array<uint8_t> tri_area_ids = {};
    tri_area_ids.resize(mesh.indices.length / 3, NULL_AREA);
    input_triangle_mesh::mark_walkable_triangles(tri_area_ids, mesh, walkable_slope_angle_degrees);
    /* SNIPPET_END: walkable_triangles */

    /* SNIPPET_START: rasterize */
    // CONFIG: heightfield cell size and height, in world units
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;

    // CONFIG: agent walkable climb
    const float walkable_climb_world = 0.4f;
    const int walkable_climb_voxels = (int)ceilf(walkable_climb_world / cell_height);

    // CONFIG: agent walkable height
    const float walkable_height_world = 1.8f;
    const int walkable_height_voxels = (int)ceilf(walkable_height_world / cell_height);

    // the bounds of the input, and the heightfield's grid over it
    const Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(mesh);
    const GridSize grid = heightfield::calculate_grid_size(bounds, cell_size);
    Heightfield solid = heightfield::create(grid.width, grid.height, bounds, cell_size, cell_height);

    // rasterise the triangles into the heightfield
    heightfield::rasterize_triangles(context, solid, mesh, tri_area_ids, walkable_climb_voxels);

    // filter out the spans an agent cannot stand on
    heightfield::filter_low_hanging_walkable_obstacles(solid, walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, walkable_height_voxels, walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, walkable_height_voxels);
    /* SNIPPET_END: rasterize */

    /* SNIPPET_START: compact_heightfield */
    // CONFIG: agent radius
    const float walkable_radius_world = 0.3f;
    const int walkable_radius_voxels = (int)ceilf(walkable_radius_world / cell_size);

    // the open space above the solid spans, with each span's neighbours
    CompactHeightfield compact = compact_heightfield::build(context, solid, walkable_height_voxels,
                                                            walkable_climb_voxels);

    // erode the walkable area by the agent's radius
    compact_heightfield::erode_walkable_area(compact, walkable_radius_voxels);

    // OPTIONAL: mark areas of your own. area ids are yours to give meaning to, see "Custom Query
    // Filters and Custom Area Types"
    const Box3 pond = {{-10, -1, 4}, {-4, 1, 10}};
    compact_heightfield::mark_box_area(compact, pond, AREA_WATER);
    /* SNIPPET_END: compact_heightfield */

    /* SNIPPET_START: regions */
    // CONFIG: border size, in voxels: 0 for a solo nav mesh, the agent radius plus a few voxels
    // for a tiled one
    const int border_size = 0;

    // CONFIG: min region area and merge region area, in voxels
    const int min_region_area = 8;
    const int merge_region_area = 20;

    // prepare for watershed partitioning with a distance field along the walkable surface
    compact_heightfield_regions::build_distance_field(compact);

    // split the walkable surface into simple regions without holes
    compact_heightfield_regions::build_regions(context, compact, border_size, min_region_area,
                                               merge_region_area);
    /* SNIPPET_END: regions */

    /* SNIPPET_START: contours */
    // CONFIG: max simplification error, in voxels
    const float max_simplification_error = 1.3f;

    // CONFIG: max edge length, in voxels
    const int max_edge_length = 12;

    // trace and simplify the regions' outlines
    ContourSet contours = contour_set::build_contours(context, compact, max_simplification_error,
                                                      max_edge_length, CONTOUR_TESS_WALL_EDGES);
    /* SNIPPET_END: contours */

    /* SNIPPET_START: poly_mesh */
    // CONFIG: max vertices per polygon: 3 to 6, higher gives fewer but more complex polys
    const int max_vertices_per_poly = 6;

    PolyMesh polys = poly_mesh::build_poly_mesh(context, contours, max_vertices_per_poly);

    // give each poly an area and flags. the presets give every walkable poly area 0 and flags 1;
    // here walkers may use the ground and swimmers the water too
    for (uint32_t i = 0; i < (uint32_t)polys.poly_count; i++) {
        if (polys.areas[i] == WALKABLE_AREA) {
            polys.areas[i] = AREA_GROUND;
            polys.flags[i] = FLAG_WALK;
        } else if (polys.areas[i] == AREA_WATER) {
            polys.flags[i] = FLAG_SWIM;
        }
    }

    // CONFIG: detail mesh sample distance and max sample error, in world units
    const float sample_distance = 6 * cell_size;
    const float sample_max_error = 1 * cell_height;

    PolyMeshDetail detail = poly_mesh_detail::build_poly_mesh_detail(context, polys, compact,
                                                                     sample_distance, sample_max_error);
    /* SNIPPET_END: poly_mesh */

    /* SNIPPET_START: convert */
    // the build-time meshes in the runtime tile format
    NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
    NavMeshTileDetailMesh tile_detail =
        nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, detail);
    /* SNIPPET_END: convert */

    /* SNIPPET_START: nav_mesh */
    const NavMeshTileParams params = {
        .bounds = polys.bounds,
        .vertices = tile_polys.vertices,
        .polys = tile_polys.polys,
        .poly_vertices = tile_polys.poly_vertices,
        .poly_neis = tile_polys.poly_neis,
        .detail_meshes = tile_detail.detail_meshes,
        .detail_vertices = tile_detail.detail_vertices,
        .detail_triangles = tile_detail.detail_triangles,
        .tile_x = 0,
        .tile_y = 0,
        .tile_layer = 0,
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_height = walkable_height_world,
        .walkable_radius = walkable_radius_world,
        .walkable_climb = walkable_climb_world,
    };

    // a nav mesh one tile wide: the tile's size is the poly mesh's bounds
    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.origin = polys.bounds.min;
    nav_mesh.tile_width = polys.bounds.max.x - polys.bounds.min.x;
    nav_mesh.tile_height = polys.bounds.max.z - polys.bounds.min.z;

    // build the tile, which makes its BV tree, and add it
    nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));
    /* SNIPPET_END: nav_mesh */

    /* SNIPPET_START: release_intermediates */
    // build_tile copied what it needs: everything else can go
    nav_mesh_tile::release(tile_polys);
    nav_mesh_tile::release(tile_detail);
    poly_mesh_detail::release(detail);
    poly_mesh::release(polys);
    contour_set::release(contours);
    compact_heightfield::release(compact);
    heightfield::release(solid);
    tri_area_ids.release();
    build_context::release(context);
    /* SNIPPET_END: release_intermediates */

    nav_mesh_api::release(nav_mesh);
}

void run_generation_snippets(Scene &scene) {
    solo_snippets(scene);
    tiled_snippets(scene);
    deep_dive_snippets(scene);
}

}
