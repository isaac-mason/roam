#pragma once

#include <stdint.h>

#include "roam/generate/compact_heightfield.h"
#include "roam/generate/contour_set.h"
#include "roam/generate/heightfield.h"
#include "roam/generate/poly_mesh.h"
#include "roam/generate/poly_mesh_detail.h"
#include "roam/math/vec3.h"
#include "roam/query/nav_mesh.h"
#include "roam/query/nav_mesh_search.h"
#include "roam/util/array.h"

namespace roam {

enum DebugPrimitiveType : uint8_t {
    DEBUG_PRIMITIVE_TRIANGLES,
    DEBUG_PRIMITIVE_LINES,
    DEBUG_PRIMITIVE_POINTS,
    DEBUG_PRIMITIVE_BOXES,
};

struct DebugPrimitive {
    DebugPrimitiveType type;
    Array<float> positions;
    Array<float> colors;
    Array<uint32_t> indices;
    Array<float> scales;
    float size;
    float line_width;
    float opacity;
    bool transparent;
    bool double_sided;
};

using DebugPrimitives = Array<DebugPrimitive>;

namespace debug {

void release(DebugPrimitives &primitives);

DebugPrimitives create_triangle_area_ids_helper(const float *positions, const uint32_t *indices, uint32_t index_count, const uint8_t *tri_area_ids);
DebugPrimitives create_heightfield_helper(const Heightfield &heightfield);
DebugPrimitives create_compact_heightfield_solid_helper(const CompactHeightfield &compact_heightfield);
DebugPrimitives create_compact_heightfield_distances_helper(const CompactHeightfield &compact_heightfield);
DebugPrimitives create_compact_heightfield_regions_helper(const CompactHeightfield &compact_heightfield);
DebugPrimitives create_raw_contours_helper(const ContourSet &contour_set);
DebugPrimitives create_simplified_contours_helper(const ContourSet &contour_set);
DebugPrimitives create_poly_mesh_helper(const PolyMesh &poly_mesh);
DebugPrimitives create_poly_mesh_detail_helper(const PolyMeshDetail &poly_mesh_detail);
DebugPrimitives create_nav_mesh_helper(const NavMesh &nav_mesh);
DebugPrimitives create_nav_mesh_tile_helper(const NavMeshTile &tile);
DebugPrimitives create_nav_mesh_poly_helper(const NavMesh &nav_mesh, NodeRef node_ref, Vec3 color = {0, 0.75f, 1});
DebugPrimitives create_nav_mesh_tile_bv_tree_helper(const NavMeshTile &tile);
DebugPrimitives create_nav_mesh_bv_tree_helper(const NavMesh &nav_mesh);
DebugPrimitives create_nav_mesh_links_helper(const NavMesh &nav_mesh);
DebugPrimitives create_nav_mesh_tile_portals_helper(const NavMeshTile &tile);
DebugPrimitives create_nav_mesh_portals_helper(const NavMesh &nav_mesh);
DebugPrimitives create_search_nodes_helper(const SearchNodePool &pool);
DebugPrimitives create_nav_mesh_off_mesh_connections_helper(const NavMesh &nav_mesh);

}

}
