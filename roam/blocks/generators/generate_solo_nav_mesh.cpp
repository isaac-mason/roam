#include "roam/blocks/generators/generate_solo_nav_mesh.h"

#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"

namespace roam {

namespace generate_solo_nav_mesh {

SoloNavMeshResult generate_solo_nav_mesh(const SoloNavMeshInput &input, const SoloNavMeshOptions &options) {
    SoloNavMeshResult result = {};
    SoloNavMeshIntermediates &intermediates = result.intermediates;
    BuildContext &context = intermediates.build_context;
    intermediates.input = input;

    intermediates.tri_area_ids.resize(input.index_count / 3, 0);
    input_triangle_mesh::mark_walkable_triangles(input.positions, input.indices, input.index_count, intermediates.tri_area_ids.data,
                                                 options.walkable_slope_angle_degrees);

    Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(input.positions, input.indices, input.index_count);
    GridSize grid_size = heightfield::calculate_grid_size(bounds, options.cell_size);

    intermediates.heightfield = heightfield::create(grid_size.width, grid_size.height, bounds, options.cell_size, options.cell_height);
    Heightfield &solid = intermediates.heightfield;

    heightfield::rasterize_triangles(context, solid, input.positions, input.indices, input.index_count, intermediates.tri_area_ids.data,
                                     options.walkable_climb_voxels);

    heightfield::filter_low_hanging_walkable_obstacles(solid, options.walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, options.walkable_height_voxels);

    intermediates.compact_heightfield = compact_heightfield::build(context, solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    CompactHeightfield &compact = intermediates.compact_heightfield;

    compact_heightfield::erode_walkable_area(compact, options.walkable_radius_voxels);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    intermediates.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    intermediates.poly_mesh = poly_mesh::build_poly_mesh(context, intermediates.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = intermediates.poly_mesh;

    for (int poly_index = 0; poly_index < polys.poly_count; poly_index++) {
        if (polys.areas[(uint32_t)poly_index] == WALKABLE_AREA) polys.areas[(uint32_t)poly_index] = 0;
        if (polys.areas[(uint32_t)poly_index] == 0) polys.flags[(uint32_t)poly_index] = 1;
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

void release(SoloNavMeshIntermediates &intermediates) {
    build_context::release(intermediates.build_context);
    intermediates.tri_area_ids.release();
    heightfield::release(intermediates.heightfield);
    compact_heightfield::release(intermediates.compact_heightfield);
    contour_set::release(intermediates.contour_set);
    poly_mesh::release(intermediates.poly_mesh);
    poly_mesh_detail::release(intermediates.poly_mesh_detail);
}

}

}
