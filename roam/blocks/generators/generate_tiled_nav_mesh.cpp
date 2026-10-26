#include "roam/blocks/generators/generate_tiled_nav_mesh.h"

#include <math.h>
#include <string.h>

#include "roam/generate/common.h"
#include "roam/generate/compact_heightfield_regions.h"
#include "roam/generate/input_triangle_mesh.h"
#include "roam/generate/nav_mesh_tile.h"
#include "roam/query/nav_mesh_api.h"

namespace roam {

namespace generate_tiled_nav_mesh {

TiledNavMeshOptions derive_sizes(TiledNavMeshOptions options) {
    options.tile_size_world = (float)options.tile_size_voxels * options.cell_size;
    options.walkable_radius_voxels = (int)ceilf(options.walkable_radius_world / options.cell_size);
    options.walkable_climb_voxels = (int)ceilf(options.walkable_climb_world / options.cell_height);
    options.walkable_height_voxels = (int)ceilf(options.walkable_height_world / options.cell_height);
    return options;
}

struct NavMeshTileBuild {
    Array<uint8_t> tri_area_ids;
    Box3 expanded_tile_bounds;
    Heightfield heightfield;
    CompactHeightfield compact_heightfield;
    ContourSet contour_set;
    PolyMesh poly_mesh;
    PolyMeshDetail poly_mesh_detail;
};

static NavMeshTileBuild build_nav_mesh_tile(BuildContext &context, Span<const float> positions, const ChunkyTriMesh &chunky_tri_mesh, Box3 tile_bounds,
                                            const TiledNavMeshOptions &options) {
    NavMeshTileBuild build = {};

    float border = (float)options.border_size * options.cell_size;
    Box3 expanded = tile_bounds;
    expanded.min.x -= border;
    expanded.min.z -= border;
    expanded.max.x += border;
    expanded.max.z += border;
    build.expanded_tile_bounds = expanded;

    Array<uint32_t> chunks = {};
    chunks.reserve(chunky_tri_mesh.nodes.length);
    ChunkyTriMeshChunksResult found;
    chunky_tri_mesh::get_chunks_overlapping_rect(found, chunky_tri_mesh, {expanded.min.x, expanded.min.z}, {expanded.max.x, expanded.max.z}, chunks);
    chunks.resize(found.chunk_count);

    int heightfield_size = options.tile_size_voxels + options.border_size * 2;
    build.heightfield = heightfield::create(heightfield_size, heightfield_size, expanded, options.cell_size, options.cell_height);
    Heightfield &solid = build.heightfield;

    build.tri_area_ids.resize(chunky_tri_mesh.max_tris_per_chunk, 0);

    for (uint32_t chunk : chunks) {
        const ChunkyTriMeshNode &node = chunky_tri_mesh.nodes[chunk];
        const uint32_t *chunk_triangles = &chunky_tri_mesh.triangles.data[(uint32_t)node.index * 3];
        uint32_t chunk_index_count = node.count * 3;
        InputTriangleMesh chunk_mesh = {positions, {chunk_triangles, chunk_index_count}};

        memset(build.tri_area_ids.data, 0, build.tri_area_ids.length);
        input_triangle_mesh::mark_walkable_triangles(build.tri_area_ids, chunk_mesh, options.walkable_slope_angle_degrees);
        heightfield::rasterize_triangles(context, solid, chunk_mesh, build.tri_area_ids, options.walkable_climb_voxels);
    }
    chunks.release();

    heightfield::filter_low_hanging_walkable_obstacles(solid, options.walkable_climb_voxels);
    heightfield::filter_ledge_spans(solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    heightfield::filter_walkable_low_height_spans(solid, options.walkable_height_voxels);

    build.compact_heightfield = compact_heightfield::build(context, solid, options.walkable_height_voxels, options.walkable_climb_voxels);
    CompactHeightfield &compact = build.compact_heightfield;

    compact_heightfield::erode_walkable_area(compact, options.walkable_radius_voxels);
    compact_heightfield_regions::build_distance_field(compact);
    compact_heightfield_regions::build_regions(context, compact, options.border_size, options.min_region_area, options.merge_region_area);

    build.contour_set =
        contour_set::build_contours(context, compact, options.max_simplification_error, options.max_edge_length, CONTOUR_TESS_WALL_EDGES);

    build.poly_mesh = poly_mesh::build_poly_mesh(context, build.contour_set, options.max_vertices_per_poly);
    PolyMesh &polys = build.poly_mesh;

    for (int poly_index = 0; poly_index < polys.poly_count; poly_index++) {
        if (polys.areas[(uint32_t)poly_index] == WALKABLE_AREA) polys.areas[(uint32_t)poly_index] = 0;
        if (polys.areas[(uint32_t)poly_index] == 0) polys.flags[(uint32_t)poly_index] = 1;
    }

    build.poly_mesh_detail =
        poly_mesh_detail::build_poly_mesh_detail(context, polys, compact, options.detail_sample_distance, options.detail_sample_max_error);

    return build;
}

static void release(NavMeshTileBuild &build) {
    build.tri_area_ids.release();
    heightfield::release(build.heightfield);
    compact_heightfield::release(build.compact_heightfield);
    contour_set::release(build.contour_set);
    poly_mesh::release(build.poly_mesh);
    poly_mesh_detail::release(build.poly_mesh_detail);
}

TiledNavMeshResult generate_tiled_nav_mesh(const InputTriangleMesh &input, const TiledNavMeshOptions &options) {
    TiledNavMeshResult result = {};
    TiledNavMeshIntermediates &intermediates = result.intermediates;
    BuildContext &context = intermediates.build_context;
    intermediates.input = input;

    Box3 mesh_bounds = input_triangle_mesh::calculate_mesh_bounds(input);
    GridSize grid_size = heightfield::calculate_grid_size(mesh_bounds, options.cell_size);
    intermediates.input_bounds = mesh_bounds;

    NavMesh &nav_mesh = result.nav_mesh;
    nav_mesh = nav_mesh_api::create_nav_mesh();
    nav_mesh.tile_width = options.tile_size_world;
    nav_mesh.tile_height = options.tile_size_world;
    nav_mesh.origin = mesh_bounds.min;

    intermediates.chunky_tri_mesh = chunky_tri_mesh::create(input);

    int tile_count_x = (grid_size.width + options.tile_size_voxels - 1) / options.tile_size_voxels;
    int tile_count_y = (grid_size.height + options.tile_size_voxels - 1) / options.tile_size_voxels;

    for (int tile_x = 0; tile_x < tile_count_x; tile_x++) {
        for (int tile_y = 0; tile_y < tile_count_y; tile_y++) {
            Box3 tile_bounds = {
                {mesh_bounds.min.x + (float)tile_x * options.tile_size_world, mesh_bounds.min.y, mesh_bounds.min.z + (float)tile_y * options.tile_size_world},
                {mesh_bounds.min.x + (float)(tile_x + 1) * options.tile_size_world, mesh_bounds.max.y,
                 mesh_bounds.min.z + (float)(tile_y + 1) * options.tile_size_world},
            };

            NavMeshTileBuild build = build_nav_mesh_tile(context, input.positions, intermediates.chunky_tri_mesh, tile_bounds, options);

            if (build.poly_mesh.vertices.length == 0) {
                release(build);
                continue;
            }

            intermediates.tri_area_ids.push(build.tri_area_ids);
            intermediates.heightfield.push(build.heightfield);
            intermediates.compact_heightfield.push(build.compact_heightfield);
            intermediates.contour_set.push(build.contour_set);
            intermediates.poly_mesh.push(build.poly_mesh);
            intermediates.poly_mesh_detail.push(build.poly_mesh_detail);

            const PolyMesh &polys = build.poly_mesh;
            NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
            NavMeshTileDetailMesh tile_detail_mesh = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, build.poly_mesh_detail);

            NavMeshTileParams tile_params = {
                .bounds = polys.bounds,
                .vertices = tile_polys.vertices,
                .polys = tile_polys.polys,
                .poly_vertices = tile_polys.poly_vertices,
                .poly_neis = tile_polys.poly_neis,
                .detail_meshes = tile_detail_mesh.detail_meshes,
                .detail_vertices = tile_detail_mesh.detail_vertices,
                .detail_triangles = tile_detail_mesh.detail_triangles,
                .tile_x = tile_x,
                .tile_y = tile_y,
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
        }
    }

    return result;
}

void release(TiledNavMeshIntermediates &intermediates) {
    build_context::release(intermediates.build_context);
    chunky_tri_mesh::release(intermediates.chunky_tri_mesh);
    for (Array<uint8_t> &tri_area_ids : intermediates.tri_area_ids) tri_area_ids.release();
    for (Heightfield &solid : intermediates.heightfield) heightfield::release(solid);
    for (CompactHeightfield &compact : intermediates.compact_heightfield) compact_heightfield::release(compact);
    for (ContourSet &contours : intermediates.contour_set) contour_set::release(contours);
    for (PolyMesh &polys : intermediates.poly_mesh) poly_mesh::release(polys);
    for (PolyMeshDetail &detail : intermediates.poly_mesh_detail) poly_mesh_detail::release(detail);
    intermediates.tri_area_ids.release();
    intermediates.heightfield.release();
    intermediates.compact_heightfield.release();
    intermediates.contour_set.release();
    intermediates.poly_mesh.release();
    intermediates.poly_mesh_detail.release();
}

}

}
