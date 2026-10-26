# roam

roam is a navigation mesh library for 3D floor-based navigation: it builds nav meshes from triangle soups and finds
paths over them. It is C++20 with no dependencies: no C++ standard library, no exceptions, no RTTI.

roam is a port of [navcat](https://github.com/isaac-mason/navcat), the JavaScript library, with the same algorithms and
the same results. navcat is the reference for how everything behaves, and parity tests hold roam to it. For
JavaScript, use navcat: roam has no JS bindings.

- [Getting roam](#getting-roam)
  - [CMake](#cmake)
  - [Without CMake](#without-cmake)
- [Quick start](#quick-start)
- [Conventions](#conventions)
  - [Results and spans](#results-and-spans)
  - [Public API](#public-api)
- [Memory](#memory)
  - [Errors](#errors)
- [Generation](#generation)
  - [Input](#input)
  - [Solo nav meshes](#solo-nav-meshes)
  - [Tiled nav meshes](#tiled-nav-meshes)
  - [Settings](#settings)
  - [Step by step](#step-by-step)
  - [Your own polygons](#your-own-polygons)
- [The nav mesh](#the-nav-mesh)
  - [Nodes and refs](#nodes-and-refs)
  - [Tiles](#tiles)
  - [Saving and loading](#saving-and-loading)
- [Queries](#queries)
  - [Nearest poly](#nearest-poly)
  - [Paths](#paths)
  - [Node paths](#node-paths)
  - [Straight paths](#straight-paths)
  - [Smooth paths](#smooth-paths)
  - [Sliced searches](#sliced-searches)
  - [Raycasts](#raycasts)
  - [Moving along the surface](#moving-along-the-surface)
  - [Random points](#random-points)
  - [Reachability](#reachability)
  - [Local neighbourhoods](#local-neighbourhoods)
  - [Polys](#polys)
- [Filters and areas](#filters-and-areas)
- [Off-mesh connections](#off-mesh-connections)
  - [Point connections](#point-connections)
  - [Segment connections](#segment-connections)
- [Agents](#agents)
  - [Crowds](#crowds)
  - [Off-mesh connections in a crowd](#off-mesh-connections-in-a-crowd)
  - [Path corridors](#path-corridors)
- [Other blocks](#other-blocks)
  - [Flood fill](#flood-fill)
  - [Merging meshes](#merging-meshes)
  - [Chunky triangle meshes](#chunky-triangle-meshes)
- [Debug drawing](#debug-drawing)
- [Differences from navcat](#differences-from-navcat)
- [Demos](#demos)
- [Tests and benchmarks](#tests-and-benchmarks)
- [Acknowledgements](#acknowledgements)

## Getting roam

### CMake

roam's `CMakeLists.txt` makes two libraries:

- `roam`: generation, queries and debug drawing.
- `roam_blocks`: the generators, agents and helpers built on `roam`, ported from navcat's `blocks`. It links `roam`.

Fetch roam, or add a checkout (a git submodule, say) with `add_subdirectory(path/to/roam)`, then link a target. roam's
headers need C++20, so set it for your project too:

```cmake
cmake_minimum_required(VERSION 3.24)
project(game LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(roam GIT_REPOSITORY https://github.com/isaac-mason/roam.git GIT_TAG main)
FetchContent_MakeAvailable(roam)

add_executable(game main.cpp)
target_link_libraries(game PRIVATE roam_blocks)
```

The tests and benchmarks build only when roam is the top-level project. Its options:

- `ROAM_BUILD_TESTS` and `ROAM_BUILD_BENCH`: the tests and the benchmarks.
- `ROAM_BUILD_DEMOS`: the demo app, which fetches sokol, Dear ImGui and cgltf. Off by default.

### Without CMake

Compile every `.cpp` under `roam/` with the repository root on the include path (headers are included as
`"roam/query/nav_mesh_api.h"`), as C++20. Leave out `roam/blocks/` if you use none of it. roam's own targets build with
`-fno-exceptions -fno-rtti -ffp-contract=off`; the last keeps results from depending on whether the compiler fuses
multiply-adds, as navcat's do not.

```sh
c++ -std=c++20 -O2 -fno-exceptions -fno-rtti -ffp-contract=off -I path/to/roam main.cpp $(find path/to/roam/roam -name '*.cpp')
```

## Quick start

A floor with a block in the middle, a nav mesh built over it, and a path from one side of the block to the other:

```cpp
#include <stdio.h>

#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/query/find_path.h"

using namespace roam;

int main() {
    // a 20 m floor with a 4 m wide, 2 m tall block in the middle, triangles wound counter clockwise from above
    const float positions[] = {
        -10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10,
        -2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2,
        -2, 2, -2, 2, 2, -2, 2, 2, 2, -2, 2, 2,
    };
    const uint32_t indices[] = {
        0, 2, 1, 0, 3, 2,
        8, 10, 9, 8, 11, 10,
        4, 9, 5, 4, 8, 9, 5, 10, 6, 5, 9, 10,
        6, 11, 7, 6, 10, 11, 7, 8, 4, 7, 11, 8,
    };
    const InputTriangleMesh level = {positions, indices};

    // voxels of 0.2 m for an agent 0.3 m wide and 1.8 m tall, which can climb 0.4 m and walk up 45 degree slopes
    const float cell_size = 0.2f;
    const float cell_height = 0.2f;
    const SoloNavMeshOptions options = {
        .cell_size = cell_size,
        .cell_height = cell_height,
        .walkable_radius_voxels = 2,
        .walkable_radius_world = 0.3f,
        .walkable_climb_voxels = 2,
        .walkable_climb_world = 0.4f,
        .walkable_height_voxels = 9,
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
    };
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh(level, options);
    // the stages it went through are kept for debugging; let them go
    generate_solo_nav_mesh::release(result.intermediates);
    NavMesh &nav_mesh = result.nav_mesh;

    // a path from one side of the block to the other: the start and end are snapped to the nav mesh within the half
    // extents, and the path's corners are written into the points buffer
    QueryWorkspace workspace = nav_mesh_search::create_query_workspace();
    StraightPathPoint points[64];
    FindPathResult path = {};
    find_path::find_path(path, workspace, nav_mesh, {-8, 0, 0}, {8, 0, 0}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);

    printf("%s, %u points\n", path.success ? "found a path" : "no path", path.point_count);
    for (uint32_t i = 0; i < path.point_count; i++) {
        printf("  %.2f %.2f %.2f\n", points[i].position.x, points[i].position.y, points[i].position.z);
    }

    nav_mesh_search::release(workspace);
    nav_mesh_api::release(nav_mesh);
    return 0;
}
```

It prints the path's corners, from the start, around the block and on to the end:

```
found a path, 4 points
  -8.00 0.20 0.00
  -2.20 0.20 2.60
  2.20 0.20 2.60
  8.00 0.20 0.00
```

The rest of this guide leaves out the includes and `using namespace roam;`. Most of its examples share one level,
the quick start's floor and block plus a platform 1 m up, its triangles in `Array`s:

```cpp
// a box's top and four sides, wound to face out
static void add_box(Array<float> &positions, Array<uint32_t> &indices, Vec3 min, Vec3 max) {
    uint32_t base = positions.length / 3;
    for (int i = 0; i < 8; i++) {
        positions.push((i & 1) ? max.x : min.x);
        positions.push((i & 2) ? max.y : min.y);
        positions.push((i & 4) ? max.z : min.z);
    }
    const uint32_t faces[] = {
        2, 6, 7, 2, 7, 3,
        0, 2, 3, 0, 3, 1,
        4, 5, 7, 4, 7, 6,
        0, 4, 6, 0, 6, 2,
        1, 3, 7, 1, 7, 5,
    };
    for (uint32_t index : faces) indices.push(base + index);
}

struct Level {
    Array<float> positions;
    Array<uint32_t> indices;
};

Level create_level() {
    Level level = {};
    const float floor[] = {-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10};
    for (float value : floor) level.positions.push(value);
    const uint32_t floor_indices[] = {0, 2, 1, 0, 3, 2};
    for (uint32_t index : floor_indices) level.indices.push(index);
    // a block in the middle to walk around, and a platform 1 m up that the floor does not reach
    add_box(level.positions, level.indices, {-2, 0, -2}, {2, 2, 2});
    add_box(level.positions, level.indices, {4, 0, -9}, {9, 1, -5});
    return level;
}
```

## Conventions

- Everything is in `namespace roam`. Types are directly in it (`NavMesh`, `QueryFilter`, `StraightPathPoint`). Functions
  and constants are in a namespace per module, named after its header: `nav_mesh_api::add_tile` is in
  `roam/query/nav_mesh_api.h`, `find_path::find_path` in `roam/query/find_path.h`. Folders are only for finding files.
- Functions are snake_case, types PascalCase, constants and enum values UPPER_SNAKE_CASE: `NODE_TYPE_OFFMESH`,
  `OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL`.
- Data is plain structs and their fields are yours to read. Settings structs take designated initializers, which C++20
  requires in declaration order.
- `float` everywhere. Positions are `Vec3 {x, y, z}`, y up, and triangles are wound counter clockwise seen from above.
  The examples use metres, but roam has no units of its own.
- A function that builds or changes something takes it first (after a `BuildContext` when it logs):
  `heightfield::rasterize_triangles(context, heightfield, ...)`, `nav_mesh_api::add_tile(nav_mesh, tile)`.
- Lifecycles: `create` makes something that owns memory, and `release` frees what a value holds.
  `nav_mesh_api::create_nav_mesh` and `nav_mesh_api::release`, `crowd::create` and `crowd::release`. Only what you
  create owns memory.
- Math returns by value: `vec3::add(a, b)`, `box3::expand_by_point(box, point)`.

### Results and spans

Queries write their result into `out`, their first parameter. A result is a plain value: flags, refs, positions and
counts. It never owns memory, so it needs no release, and it can be reused from call to call.

Anything of variable size goes into a buffer of yours, passed as a `Span<T>`: a pointer and a capacity. A span converts
from a C array, from an `Array` (roam's growable array), or is built as `{data, capacity}`. A writable span of an
`Array` covers its capacity; a read-only one, `Span<const T>`, covers its length. Here with `start` and `end` from
[`find_nearest_poly`](#nearest-poly):

```cpp
// a C array: its size is the span's capacity
NodeRef fixed[256];
FindNodePathResult path = {};
nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, fixed);

// an Array: a writable span covers its capacity, so reserve, pass it, then resize to what was written
Array<NodeRef> grown = {};
grown.reserve(16);
nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, grown);
grown.resize(path.path_count);

// a pointer and a capacity, and a read-only span of the refs written so far
NodeRef *heap = memory::allocate_array<NodeRef>(256);
nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, {heap, 256});
Span<const NodeRef> written = {heap, path.path_count};
```

A result names its counts after the buffer: `path_count` is how many it wrote, `path_total` the true size where that
is known, and `truncated` says the buffer filled up. Searches take a `QueryWorkspace &` after `out`: the search nodes
and scratch arrays they reuse from call to call, so that a warm query allocates nothing. A workspace serves one query
at a time.

### Public API

roam's headers are its API: there are no private ones. The ones most programs use:

- **Generation:** `blocks/generators/generate_solo_nav_mesh.h` and `generate_tiled_nav_mesh.h`, and for a pipeline of
  your own, the headers under `generate/`: `input_triangle_mesh.h`, `heightfield.h`, `compact_heightfield.h`,
  `compact_heightfield_regions.h`, `contour_set.h`, `poly_mesh.h`, `poly_mesh_detail.h` and `nav_mesh_tile.h`.
- **The nav mesh:** `query/nav_mesh.h` (its structs), `query/nav_mesh_api.h` (tiles, off-mesh connections, nearest
  polys, filters) and `query/nav_mesh_io.h` (saving and loading).
- **Queries:** `query/nav_mesh_search.h` (node paths, raycasts, moving along the surface, random points,
  reachability), `query/find_path.h`, `query/find_straight_path.h`, `query/find_smooth_path.h` and
  `query/local_neighbourhood.h`.
- **Agents:** `blocks/agents/crowd.h`, `path_corridor.h`, `local_boundary.h` and `obstacle_avoidance.h`.
- **The rest:** `debug/debug.h`, `blocks/search/flood_fill_nav_mesh.h`, `blocks/geometry/`, and `util/` and `math/`.

## Memory

roam takes all its memory through one allocator, `malloc` and `free` to start with. Set your own before anything
allocates. Every block comes back with the size it was asked for, so your allocator needs no headers of its own. Blocks
need `malloc`'s alignment:

```cpp
struct CountingAllocator {
    size_t bytes;
    size_t peak;
};

void *counting_allocate(size_t size, void *user_data) {
    CountingAllocator &counter = *static_cast<CountingAllocator *>(user_data);
    counter.bytes += size;
    if (counter.bytes > counter.peak) counter.peak = counter.bytes;
    return malloc(size);
}

void counting_free(void *block, size_t size, void *user_data) {
    static_cast<CountingAllocator *>(user_data)->bytes -= size;
    free(block);
}
```

```cpp
static CountingAllocator counter = {};
memory::set_allocator({counting_allocate, counting_free, &counter});
```

What owns memory: anything made by a `create` (a nav mesh, a workspace, a crowd, a corridor, a heightfield), the
generators' results and intermediates, and `Array`s. Each is freed by its module's `release`: `nav_mesh_api::release`,
`nav_mesh_search::release`, `crowd::release`, `generate_solo_nav_mesh::release`, `heightfield::release`, or an
`Array`'s own `release()`. Results, spans, `QueryFilter`s, params and settings hold no memory.

roam built for debug, or with `ROAM_TRACK_MEMORY` defined, counts what it holds:

```cpp
printf("%zu bytes in use, %zu at most\n", counter.bytes, counter.peak);
if (memory::tracking_enabled()) {
    memory::MemoryStats stats = memory::outstanding();
    printf("%lld live allocations, %lld bytes\n", (long long)stats.allocations, (long long)stats.bytes);
}
```

`Scoped<T>` (`roam/util/scoped.h`) is an opt-in holder that releases a value as it leaves scope, for code that wants it:

```cpp
{
    Scoped<NavMesh> nav_mesh(nav_mesh_api::release);
    SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh({level.positions, level.indices}, options);
    generate_solo_nav_mesh::release(result.intermediates);
    *nav_mesh = result.nav_mesh;
    printf("%u tiles\n", nav_mesh->tiles.length);
}
```

### Errors

Running out of memory stops the program: when the allocator returns null, roam prints the size it asked for and
aborts. Nothing in roam fails for want of memory, so nothing checks for it.

Everything else that can fail says so in its result: `success` and `flags` on a query's result, a `bool` from a
generation step, a `LoadStatus` from loading. Generation steps that fail log why in their `BuildContext`. Invalid and
stale refs, positions far outside the nav mesh and NaNs are failures, not crashes. `ROAM_ASSERT` checks roam's own
invariants in debug builds, and in release builds with `ROAM_ENABLE_ASSERTS` defined.

## Generation

A nav mesh is a set of convex polygons that cover where an agent can walk, joined where they share edges. roam builds
it from triangles, as navcat and Recast do: it voxelises the triangles, works out where an agent of a given size fits,
splits that space into regions, traces their outlines and fills them with polygons, and samples heights for a detail
mesh that keeps the polygons' heights true to the ground.

The nav mesh is made of tiles. Each tile holds polygons, their detail mesh and a bounding volume tree. Behind them, the
nav mesh keeps a graph of nodes (a node per poly and per off-mesh connection) and the links between them, which is what
searches walk.

### Input

An `InputTriangleMesh` is a borrowed triangle mesh: a span of positions (3 floats a vertex) and a span of indices (3 a
triangle). Generation only reads it, but the generators' intermediates keep its spans, for debug drawing.

```cpp
const InputTriangleMesh input = {level.positions, level.indices};
```

Pass what an agent might stand on and whatever gets in its way. Leave out what agents walk through (foliage, say), and
anything that moves.

### Solo nav meshes

`generate_solo_nav_mesh` builds a nav mesh of one tile over the whole input. Start here: it suits most levels.

```cpp
// voxels of 0.2 m, for an agent 0.3 m in radius and 1.8 m tall that climbs steps of 0.4 m and slopes of 45 degrees.
// each agent measure is given in voxels and in metres: the voxels drive the build, the metres are kept in the tiles
const float cell_size = 0.2f;
const float cell_height = 0.2f;
const float agent_radius = 0.3f;
const float agent_height = 1.8f;
const float agent_climb = 0.4f;
const SoloNavMeshOptions options = {
    .cell_size = cell_size,
    .cell_height = cell_height,
    .walkable_radius_voxels = (int)ceilf(agent_radius / cell_size),
    .walkable_radius_world = agent_radius,
    .walkable_climb_voxels = (int)ceilf(agent_climb / cell_height),
    .walkable_climb_world = agent_climb,
    .walkable_height_voxels = (int)ceilf(agent_height / cell_height),
    .walkable_height_world = agent_height,
    .walkable_slope_angle_degrees = 45,
    .border_size = 0,
    .min_region_area = 8,
    .merge_region_area = 20,
    .max_simplification_error = 1.3f,
    .max_edge_length = 12,
    .max_vertices_per_poly = 6,
    .detail_sample_distance = 6 * cell_size,
    .detail_sample_max_error = 1 * cell_height,
};

// the mesh is borrowed for the call: spans of the level's positions and indices
SoloNavMeshResult result = generate_solo_nav_mesh::generate_solo_nav_mesh({level.positions, level.indices}, options);

// every stage's output is kept: for debug drawing, or to look into a bad build
const SoloNavMeshIntermediates &intermediates = result.intermediates;
printf("%d polys\n", intermediates.poly_mesh.poly_count);
for (const BuildContextLog &log : intermediates.build_context.logs) printf("%s\n", log.message);

// the nav mesh and the intermediates are released separately, so the nav mesh can outlive them
NavMesh nav_mesh = result.nav_mesh;
generate_solo_nav_mesh::release(result.intermediates);
```

### Tiled nav meshes

`generate_tiled_nav_mesh` builds a grid of tiles. Use tiles for large worlds (a solo build of a big level has to fit
its whole heightfield in memory at once, and tiles also give better results over large areas), for tiles you build or
stream in and out at runtime, or to rebuild only the part of a level that changed.

```cpp
// the same settings as a solo build, plus the tile size. the border is the agent radius and a few voxels more, so
// each tile sees the geometry just past its edges and neighbouring tiles meet cleanly
const float cell_size = 0.2f;
const float cell_height = 0.2f;
const int tile_size_voxels = 32;
const TiledNavMeshOptions options = {
    .cell_size = cell_size,
    .cell_height = cell_height,
    .tile_size_voxels = tile_size_voxels,
    .tile_size_world = tile_size_voxels * cell_size,
    .walkable_radius_voxels = 2,
    .walkable_radius_world = 0.3f,
    .walkable_climb_voxels = 2,
    .walkable_climb_world = 0.4f,
    .walkable_height_voxels = 9,
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
};
TiledNavMeshResult result = generate_tiled_nav_mesh::generate_tiled_nav_mesh({level.positions, level.indices}, options);
generate_tiled_nav_mesh::release(result.intermediates);
NavMesh nav_mesh = result.nav_mesh;

// tiles sit on a grid from the nav mesh's origin, tile_width by tile_height
for (const NavMeshTile &tile : nav_mesh.tiles) {
    if (tile.allocated) printf("tile %d, %d: %u polys\n", tile.tile_x, tile.tile_y, tile.polys.length);
}
```

Both generators are a hundred or two lines over the steps below, kept out of `roam` so that you can copy one into your
project and change it when your needs outgrow it.

### Settings

- `cell_size` and `cell_height`: the voxel size, across and up. Smaller is more accurate and slower. A cell size of
  a third to a half of the agent's radius is a good start; the cell height is often the same, or half it.
- `walkable_radius_*`, `walkable_height_*`, `walkable_climb_*`: the agent's radius, its height, and the step it can
  climb. The nav mesh is eroded by the radius, so paths keep the agent's body off walls.
- `walkable_slope_angle_degrees`: the steepest slope that is walkable.
- `border_size`: voxels of padding around a tile, for tiled builds. Use the radius in voxels plus 3.
- `min_region_area` and `merge_region_area`: in voxels. Regions smaller than the first are dropped, and regions
  smaller than the second are merged into a neighbour.
- `max_simplification_error` and `max_edge_length`: how far, in voxels, a simplified outline may stray from the
  voxels, and the longest edge along walls (0 for no limit).
- `max_vertices_per_poly`: 3 for triangles, 6 is usual.
- `detail_sample_distance` and `detail_sample_max_error`: how far apart heights are sampled for the detail mesh, and
  how far it may stray from them. Larger values give a coarser, smaller detail mesh.

### Step by step

The generators are these steps, each public, for a pipeline of your own: to mark areas, to rasterise extra shapes, or
to keep or drop intermediates as you go. They follow navcat's and Recast's.

Each triangle gets an area id: walkable or not, by its slope.

```cpp
// the build's log: each step that can fail writes why here
BuildContext context = {};

// one area id per triangle: WALKABLE_AREA where the slope is walkable, NULL_AREA elsewhere
Array<uint8_t> tri_area_ids = {};
tri_area_ids.resize(mesh.indices.capacity / 3, NULL_AREA);
input_triangle_mesh::mark_walkable_triangles(tri_area_ids, mesh, 45.0f);
```

The triangles are rasterised into a heightfield of solid spans, and spans an agent cannot stand on are filtered out:
low obstacles it can step over become walkable, ledges and spans without the headroom it needs do not.
`heightfield::rasterize_sphere`, `rasterize_capsule`, `rasterize_cylinder`, `rasterize_box` and `rasterize_convex` add
shapes that are not triangles.

```cpp
// the triangles as columns of solid spans, then the spans an agent cannot stand on are filtered out
Box3 bounds = input_triangle_mesh::calculate_mesh_bounds(mesh);
GridSize grid = heightfield::calculate_grid_size(bounds, cell_size);
Heightfield solid = heightfield::create(grid.width, grid.height, bounds, cell_size, cell_height);
heightfield::rasterize_triangles(context, solid, mesh, tri_area_ids, walkable_climb_voxels);
heightfield::filter_low_hanging_walkable_obstacles(solid, walkable_climb_voxels);
heightfield::filter_ledge_spans(solid, walkable_height_voxels, walkable_climb_voxels);
heightfield::filter_walkable_low_height_spans(solid, walkable_height_voxels);
```

The compact heightfield is the open space above each walkable span, with each span's neighbours. Erode it by the
agent's radius, and mark areas into it: `mark_box_area`, `mark_rotated_box_area`, `mark_convex_poly_area` and
`mark_cylinder_area`. `erode_and_mark_walkable_areas` erodes areas by radii of their own.

```cpp
// the open space above the solid spans, with each span's neighbours. eroding by the agent's radius keeps it off
// walls, and areas marked now become the area of the polys over them
CompactHeightfield compact = compact_heightfield::build(context, solid, walkable_height_voxels, walkable_climb_voxels);
compact_heightfield::erode_walkable_area(compact, walkable_radius_voxels);
compact_heightfield::mark_box_area(compact, {{-10, -1, 4}, {-4, 1, 10}}, AREA_WATER);
```

Then, as in navcat: regions, their outlines, polygons over them and a detail mesh for their heights.

```cpp
// the walkable spans split into regions, each simple enough to become polygons: watershed (here) gives the best
// polygons, monotone is fastest, layers suit tiles with overlapping floors
compact_heightfield_regions::build_distance_field(compact);
compact_heightfield_regions::build_regions(context, compact, 0, 8, 20);

// each region's outline, simplified to within 1.3 voxels and with no edge longer than 12
ContourSet contours = contour_set::build_contours(context, compact, 1.3f, 12, CONTOUR_TESS_WALL_EDGES);

// the outlines as convex polygons of up to 6 vertices. each poly carries an area (what it is) and flags (who may
// use it), both yours to set
PolyMesh polys = poly_mesh::build_poly_mesh(context, contours, 6);
for (uint32_t i = 0; i < (uint32_t)polys.poly_count; i++) {
    if (polys.areas[i] == WALKABLE_AREA) {
        polys.areas[i] = AREA_GROUND;
        polys.flags[i] = FLAG_WALK;
    } else if (polys.areas[i] == AREA_WATER) {
        polys.flags[i] = FLAG_SWIM;
    }
}

// the height detail the polygons lost, sampled from the compact heightfield
PolyMeshDetail detail = poly_mesh_detail::build_poly_mesh_detail(context, polys, compact, 6 * cell_size, 1 * cell_height);
```

The build-time meshes become a tile in the runtime format, and the tile goes into a nav mesh:

```cpp
// the build-time meshes in the runtime tile format, then the tile into a nav mesh one tile wide
NavMeshTilePolys tile_polys = nav_mesh_tile::poly_mesh_to_tile_polys(polys);
NavMeshTileDetailMesh tile_detail = nav_mesh_tile::poly_mesh_detail_to_tile_detail_mesh(tile_polys.polys, detail);
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

NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
nav_mesh.origin = polys.bounds.min;
nav_mesh.tile_width = polys.bounds.max.x - polys.bounds.min.x;
nav_mesh.tile_height = polys.bounds.max.z - polys.bounds.min.z;
nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile(params));
```

```cpp
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
```

A tiled build does the same per tile, over the triangles under that tile and its border, with the tile's grid position
in the tile params and the nav mesh's `origin`, `tile_width` and `tile_height` set once for the grid.

### Your own polygons

A nav mesh does not have to come from triangles. `polygons_to_nav_mesh_tile_polys` makes a tile from convex polygons
you already have, such as a designer's nav mesh from another tool, or a grid:

```cpp
// two squares side by side, sharing an edge: indexed vertices, so shared edges become links
const float vertices[] = {0, 0, 0, 4, 0, 0, 8, 0, 0, 0, 0, 4, 4, 0, 4, 8, 0, 4};
const uint32_t left[] = {0, 3, 4, 1};
const uint32_t right[] = {1, 4, 5, 2};
const ExternalPolygon polygons[] = {
    {.vertices = left, .vertex_count = 4, .flags = 1, .area = 0},
    {.vertices = right, .vertex_count = 4, .flags = 1, .area = 0},
};
const Box3 bounds = {{0, 0, 0}, {8, 1, 4}};

NavMeshTilePolys tile_polys = nav_mesh_tile::polygons_to_nav_mesh_tile_polys(polygons, 2, vertices, 6, 0, bounds);
// no detail mesh of its own: one built from the polygons themselves
NavMeshTileDetailMesh tile_detail = nav_mesh_tile::polys_to_tile_detail_mesh(tile_polys.polys);

NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
nav_mesh.origin = bounds.min;
nav_mesh.tile_width = 8;
nav_mesh.tile_height = 4;
nav_mesh_api::add_tile(nav_mesh, nav_mesh_api::build_tile({
                                     .bounds = bounds,
                                     .vertices = tile_polys.vertices,
                                     .polys = tile_polys.polys,
                                     .poly_vertices = tile_polys.poly_vertices,
                                     .poly_neis = tile_polys.poly_neis,
                                     .detail_meshes = tile_detail.detail_meshes,
                                     .detail_vertices = tile_detail.detail_vertices,
                                     .detail_triangles = tile_detail.detail_triangles,
                                     .cell_size = 0.2f,
                                     .cell_height = 0.2f,
                                     .walkable_height = 1.8f,
                                     .walkable_radius = 0.3f,
                                     .walkable_climb = 0.4f,
                                 }));
nav_mesh_tile::release(tile_polys);
nav_mesh_tile::release(tile_detail);
```

Polygons must share vertices where they meet: neighbours are found by shared edges.

## The nav mesh

### Nodes and refs

A `NodeRef` names a node: a poly or an off-mesh connection. It is a `uint64_t`, with `INVALID_NODE_REF` for none
(0 is a valid ref).

```cpp
// a ref packs the node's type, its index in nav_mesh.nodes and its tile's sequence. removing a tile and adding
// another bumps the sequence, so a ref kept from before no longer resolves
if (nav_mesh_api::is_valid_node_ref(nav_mesh, node_ref)) {
    const NavMeshNode &node = nav_mesh_api::get_node_by_ref(nav_mesh, node_ref);
    printf("%s, area %u, flags %u\n", node.type == NODE_TYPE_POLY ? "a poly" : "an off-mesh connection", node.area, node.flags);
}

// a poly's tile and record, and its vertices in world space
TileAndPoly found = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, node_ref);
if (found.success) {
    for (uint32_t i = 0; i < found.poly->vertex_count; i++) {
        Vec3 vertex = vec3::load(found.tile->vertices.data, found.tile->poly_vertices[found.poly->vertex_base + i] * 3);
        printf("  %.2f %.2f %.2f\n", vertex.x, vertex.y, vertex.z);
    }
}

// every node and the links leaving it: the graph that searches walk
for (const NavMeshNode &node : nav_mesh.nodes) {
    if (!node.allocated) continue;
    for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
        NodeRef neighbour = nav_mesh.links[(uint32_t)link].to_node_ref;
        (void)neighbour;
    }
}
```

Everything in `NavMesh` and `NavMeshTile` is there to read, as the "Flow Field Pathfinding", "Find Shortest Paths"
and "Find Diverse Paths" demos do to search the graph their own way. Change a nav mesh only through `nav_mesh_api`.

### Tiles

`add_tile` takes a built tile, links it with its neighbours and the off-mesh connections that reach it, and returns
its id. `remove_tile` takes one out by grid position. A tile's polys get new refs each time it is added, so hold refs,
not pointers, and check old refs with `is_valid_node_ref`. Never keep a tile pointer across `add_tile` or
`remove_tile`.

```cpp
// the tile at a grid position, and the grid position of a point
const Vec2 cell = nav_mesh_api::world_to_tile_position(nav_mesh, {0, 0, 0});
const int x = (int)cell.x;
const int y = (int)cell.y;
const NavMeshTile *tile = nav_mesh_api::get_tile_at(nav_mesh, x, y, 0);

// a copy of it, as bytes, before taking it out: its polys' refs go with it
SaveResult size = {};
nav_mesh_io::save_tile(size, *tile, {});
Array<uint8_t> bytes = {};
bytes.resize(size.byte_total);
SaveResult saved = {};
nav_mesh_io::save_tile(saved, *tile, bytes);
nav_mesh_api::remove_tile(nav_mesh, x, y, 0);

// and back: add_tile takes the tile, links it to its neighbours and returns its id
NavMeshTile copy = {};
if (nav_mesh_io::load_tile(copy, bytes) == LOAD_STATUS_OK) nav_mesh_api::add_tile(nav_mesh, copy);
bytes.release();
```

Tiles at the same `tile_x` and `tile_y` with different `tile_layer`s stack in one column, for floors above each other.

### Saving and loading

A saved nav mesh is a snapshot: its tiles, nodes, links and off-mesh connections, little-endian, floats bit for bit.
Loaded, it gives the same refs and the same query results as the one saved, and loading checks every count and index,
so bad data is refused with a status rather than read.

```cpp
// an empty buffer first, for the size; then the bytes
SaveResult size = {};
nav_mesh_io::save_nav_mesh(size, nav_mesh, {});
Array<uint8_t> bytes = {};
bytes.resize(size.byte_total);
SaveResult saved = {};
nav_mesh_io::save_nav_mesh(saved, nav_mesh, bytes);

// the same nav mesh back: the same refs, the same query results
NavMesh loaded = {};
LoadStatus status = nav_mesh_io::load_nav_mesh(loaded, bytes);
if (status != LOAD_STATUS_OK) printf("not a nav mesh: %u\n", status);
```

`save_tile` and `load_tile` do the same for one tile, as in [Tiles](#tiles): a tile built offline, or kept to add
again later. A saved tile has no links; `add_tile` makes them.

## Queries

Queries take a `QueryFilter`, which says which polys a query may use and what crossing them costs. Use
`nav_mesh_api::DEFAULT_QUERY_FILTER` (every poly with any flag set, costed by distance) until you need your own; see
[Filters and areas](#filters-and-areas). The examples here use the shared level's solo nav mesh, a `QueryWorkspace` and
the default filter.

### Nearest poly

Most queries start from a poly and a point on it. `find_nearest_poly` finds them for a point in the world:

```cpp
// the poly nearest a point, within a box of these half extents around it, and the nearest point on it
FindNearestPolyResult nearest = {};
nav_mesh_api::find_nearest_poly(nearest, nav_mesh, {-6, 1, 3}, {1, 2, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
if (nearest.success) printf("poly %llu at %.2f %.2f %.2f\n", (unsigned long long)nearest.node_ref, nearest.position.x, nearest.position.y, nearest.position.z);
```

### Paths

`find_path` is the whole of a path in one call: the nearest polys to the start and end, a node path between them, and
the node path's corners. Its points are where an agent turns, and where it goes onto an off-mesh connection.

```cpp
// nearest polys for both ends, a node path between them, and its corners, in one call
StraightPathPoint points[64];
NodeRef node_path[256];
FindPathResult path = {};
find_path::find_path(path, workspace, nav_mesh, {-8, 0, 0}, {8, 0, 0}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                     {.node_path = node_path});
if (path.flags & FIND_PATH_RESULT_PARTIAL_PATH) printf("the end is out of reach: the path goes as near as it can\n");
if (path.flags & FIND_PATH_RESULT_MAX_POINTS_REACHED) printf("more than 64 corners\n");
for (uint32_t i = 0; i < path.point_count; i++) {
    printf("%.2f %.2f %.2f%s\n", points[i].position.x, points[i].position.y, points[i].position.z,
           points[i].type == NODE_TYPE_OFFMESH ? ", onto an off-mesh connection" : "");
}
```

`truncated` is set when either buffer fills up, and that includes a node path buffer you do not pass: check
`FIND_PATH_RESULT_MAX_POINTS_REACHED` in `flags` for the points alone. When the end cannot be reached, the path goes to
the reachable poly nearest it, `success` is still true, and `flags` has `FIND_PATH_RESULT_PARTIAL_PATH`.

```cpp
// raycast_distance: the search shortcuts through open space within this distance, where it can see
find_path::find_path(path, workspace, nav_mesh, {-8, 0, 0}, {8, 0, 0}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                     {.raycast_distance = 10});
```

### Node paths

`find_node_path` is the search alone: the polys (and off-mesh connections) from start to end, by A*.

```cpp
FindNearestPolyResult start = {};
nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
FindNearestPolyResult end = {};
nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, {1, 1, 1}, filter);

// the polys and off-mesh connections from start to end, A* over the nav mesh's graph
Array<NodeRef> node_path = {};
node_path.reserve(4);
FindNodePathResult path = {};
nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, node_path);

// path_total is its true length: grow and copy it out of the workspace, without searching again
if (path.truncated) {
    node_path.reserve(path.path_total);
    nav_mesh_search::copy_last_node_path(path, workspace, node_path);
}
node_path.resize(path.path_count);
```

### Straight paths

`find_straight_path` string-pulls a node path into corners, as `find_path` does after its search. Each point says what
it is in `flags` (`STRAIGHT_PATH_POINT_START`, `_END`, `_OFFMESH`), its node's `type`, and the `node_ref` it leaves
from.

```cpp
// the node path string-pulled into corners. AREA_CROSSINGS adds a point where the path changes area, and
// ALL_CROSSINGS at every poly edge
StraightPathPoint points[64];
FindStraightPathResult straight = {};
find_straight_path::find_straight_path(straight, nav_mesh, start.position, end.position, node_path, points, FIND_STRAIGHT_PATH_AREA_CROSSINGS);
```

### Smooth paths

`find_smooth_path` walks the path in small steps along the surface instead, each point on the detail mesh: for paths
that follow hills and stairs, or for drawing.

```cpp
// the path walked in steps of 0.5 m along the surface, each point on the detail mesh: for following hills and
// stairs. slop is how near a corner counts as reached
SmoothPathPoint points[512];
FindSmoothPathResult path = {};
find_smooth_path::find_smooth_path(path, workspace, nav_mesh, {-8, 0, 0}, {8, 0, 0}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points,
                                   {.step_size = 0.5f, .slop = 0.01f});
```

### Sliced searches

A sliced search runs a node path search a number of iterations at a time, to spread a long search over frames. The
query holds its own search state, so several can be in progress at once.

```cpp
// a search spread over frames: a few iterations at a time until it is no longer in progress
SlicedNodePathQuery query = nav_mesh_search::create_sliced_node_path_query();
nav_mesh_search::init_sliced_find_node_path(query, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter);
while (query.status & SLICED_FIND_NODE_PATH_IN_PROGRESS) nav_mesh_search::update_sliced_find_node_path(query, nav_mesh, 10);

NodeRef node_path[256];
SlicedFindNodePathResult path = {};
nav_mesh_search::finalize_sliced_find_node_path(path, nav_mesh, query, node_path);
nav_mesh_search::release(query);
```

`finalize_sliced_find_node_path_partial` finishes a search early: the path to the furthest node of an existing path
that the search has reached.

### Raycasts

`raycast` walks a straight line across the nav mesh's surface (in 2D, ignoring height) from a point on a poly, and stops
at the first wall. `t` is how far along it got, from 0 to 1, or `FLT_MAX` when nothing stopped it. Use it for lines of
sight across the ground and to check that a shortcut is walkable.

```cpp
// along the surface from a point on a poly, towards another: can an agent walk straight there?
NodeRef visited[64];
RaycastResult ray = {};
nav_mesh_search::raycast(ray, nav_mesh, start.node_ref, start.position, {8, 0, 0}, filter, visited);
if (ray.t == FLT_MAX) {
    printf("clear all the way\n");
} else {
    Vec3 hit = vec3::lerp(start.position, {8, 0, 0}, ray.t);
    printf("blocked at %.2f %.2f, by a wall facing %.2f %.2f\n", hit.x, hit.z, ray.hit_normal.x, ray.hit_normal.z);
}
```

```cpp
// the same, adding up the filter's cost of the polys crossed
nav_mesh_search::raycast_with_costs(ray, nav_mesh, start.node_ref, start.position, {-8, 0, 8}, filter, INVALID_NODE_REF, visited);
printf("costs %.2f\n", ray.path_cost);
```

### Moving along the surface

`move_along_surface` moves a point across polys towards a target, sliding along the walls it meets, as a character
controller constrained to the nav mesh does. It looks only at polys near the start, so keep moves short: a frame's
worth.

```cpp
// a character controller's step: from where it is towards where its input would take it, sliding along walls
// and staying on the nav mesh. keep its poly ref between frames
FindNearestPolyResult start = {};
nav_mesh_api::find_nearest_poly(start, nav_mesh, {-4, 0, 0}, {1, 1, 1}, filter);
NodeRef visited[16];
MoveAlongSurfaceResult move = {};
nav_mesh_search::move_along_surface(move, workspace, nav_mesh, start.node_ref, start.position, {0, 0, 0.5f}, filter, visited);
if (move.success) {
    // the move is in 2D: the height is the detail mesh's under the new position
    TileAndPoly found = nav_mesh_api::get_tile_and_poly_by_ref(nav_mesh, move.node_ref);
    GetPolyHeightResult height = {};
    nav_mesh_api::get_poly_height(height, *found.tile, *found.poly, found.poly_index, move.position);
    Vec3 position = {move.position.x, height.success ? height.height : move.position.y, move.position.z};
    printf("now at %.2f %.2f %.2f, on poly %llu\n", position.x, position.y, position.z, (unsigned long long)move.node_ref);
}
```

### Random points

The random point queries take a function that returns a float in [0, 1), and its user data:

```cpp
// any generator that returns a float in [0, 1): here mulberry32, seeded
float next_random(void *user_data) {
    uint32_t &state = *static_cast<uint32_t *>(user_data);
    uint32_t t = state += 0x6d2b79f5u;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return (float)((t ^ (t >> 14)) >> 8) / 16777216.0f;
}
```

```cpp
uint32_t seed = 42;

// anywhere on the nav mesh, polys weighted by area
FindRandomPointResult anywhere = {};
nav_mesh_search::find_random_point(anywhere, nav_mesh, filter, next_random, &seed);

// within 3 m of a point, through polys connected to its own
FindNearestPolyResult centre = {};
nav_mesh_api::find_nearest_poly(centre, nav_mesh, {-6, 0, -6}, {1, 1, 1}, filter);
FindRandomPointResult nearby = {};
nav_mesh_search::find_random_point_around_circle(nearby, workspace, nav_mesh, centre.node_ref, centre.position, 3, filter, next_random, &seed);
```

### Reachability

Every nav mesh keeps track of which polys connect to which, as tiles and off-mesh connections come and go.
`is_reachable` answers from that in well under a microsecond, without searching. It follows one-way off-mesh
connections in their direction and ignores filters.

```cpp
// the floor and the top of the platform are islands of their own until a connection joins them
FindNearestPolyResult floor = {};
nav_mesh_api::find_nearest_poly(floor, nav_mesh, {6, 0, -2}, {1, 1, 1}, filter);
FindNearestPolyResult platform = {};
nav_mesh_api::find_nearest_poly(platform, nav_mesh, {6.5f, 1, -7}, {1, 1, 1}, filter);
if (!nav_mesh_search::is_reachable(workspace, nav_mesh, floor.node_ref, platform.node_ref)) printf("no way up\n");
```

Ask it before a search that might fail: a search for an unreachable end visits every poly it can reach before giving
up with a partial path. `debug::create_nav_mesh_islands_helper` draws the connected islands.

### Local neighbourhoods

`find_local_neighbourhood` finds the polys around a point, out to a radius, that connect to its own and do not overlap
each other: the area an agent can see around it. `get_poly_wall_segments` gives a poly's walls, and its portals too
when asked. The crowd uses them to keep agents off walls.

```cpp
// the polys within 2 m of a point that connect to its own without overlapping each other, and their walls
NodeRef polys[32];
FindLocalNeighbourhoodResult neighbourhood = {};
local_neighbourhood::find_local_neighbourhood(neighbourhood, workspace, nav_mesh, start.node_ref, start.position, 2, filter, polys);
for (uint32_t i = 0; i < neighbourhood.poly_count; i++) {
    WallSegment walls[16];
    PolyWallSegmentsResult segments = {};
    local_neighbourhood::get_poly_wall_segments(segments, nav_mesh, polys[i], filter, false, walls);
    printf("poly %llu: %u walls\n", (unsigned long long)polys[i], segments.segment_count);
}
```

### Polys

Smaller queries over polys: those in a box, the closest point on one, and the edge two share. `get_poly_height` gives
the detail mesh's height under a point, `get_closest_point_on_poly_boundary` the nearest point on a poly's edge.

```cpp
// every poly whose bounds overlap a box
NodeRef found[64];
QueryPolygonsResult query = {};
nav_mesh_api::query_polygons(query, nav_mesh, {{-10, -1, -10}, {0, 1, 0}}, filter, found);

// the nearest point on a poly to a point, and whether the point is over the poly
GetClosestPointOnPolyResult closest = {};
nav_mesh_api::get_closest_point_on_poly(closest, nav_mesh, node_ref, {-8, 3, 0});

// the shared edge between two neighbouring polys, as the funnel sees it
GetPortalPointsResult portal = {};
if (query.poly_count > 1) nav_mesh_search::get_portal_points(portal, nav_mesh, found[0], found[1]);
```

## Filters and areas

Each poly has an `area` and `flags`. The area is what the ground is (grass, water, a road); the flags say who may use
it (walkers, swimmers, vehicles). Both are set at build time, or after (the step by step build above marks water and
sets flags), and both are yours to give meaning to.

The generators set walkable polys to area 0 with flags 1, as navcat's do. A poly with no flags passes no filter that
checks flags, the default one included.

A `QueryFilter` is two functions and their data. `pass_filter` decides whether a query may use a node, and `get_cost`
what crossing it costs. The default, `pass_flags`, checks the node's flags against `include_flags` and
`exclude_flags`:

```cpp
// a poly passes when its flags share a bit with include_flags and none with exclude_flags
QueryFilter walkers = nav_mesh_api::create_default_query_filter();
walkers.include_flags = FLAG_WALK;
QueryFilter swimmers = nav_mesh_api::create_default_query_filter();
swimmers.include_flags = FLAG_WALK | FLAG_SWIM;
```

A cost function makes some ground dearer than other: paths take it only where it saves enough. Its `user_data` is
yours:

```cpp
struct AreaCosts {
    float ground;
    float water;
};

// a filter's cost is for the step from pa to pb across current_ref: here the distance, scaled by its area's cost
float area_cost(const QueryFilter &filter, Vec3 pa, Vec3 pb, const NavMesh &nav_mesh, NodeRef, NodeRef current_ref, NodeRef) {
    const AreaCosts &costs = *static_cast<const AreaCosts *>(filter.user_data);
    const uint8_t area = nav_mesh_api::get_node_by_ref(nav_mesh, current_ref).area;
    return vec3::distance(pa, pb) * (area == AREA_WATER ? costs.water : costs.ground);
}
```

```cpp
// swimming is allowed but costs four times walking: paths go round the water where that is shorter
AreaCosts costs = {.ground = 1, .water = 4};
QueryFilter careful = {
    .pass_filter = nav_mesh_api::pass_flags,
    .get_cost = area_cost,
    .include_flags = FLAG_WALK | FLAG_SWIM,
    .exclude_flags = 0,
    .user_data = &costs,
};
FindPathResult path = {};
find_path::find_path(path, workspace, nav_mesh, {-9, 0, 2}, {-3, 0, 9}, {1, 1, 1}, careful, points);
```

The default functions are recognised and inlined in the searches, so a filter that needs only flags is fastest with
`pass_flags` and `distance_cost`. `ANY_QUERY_FILTER` passes every node, flags or not.

## Off-mesh connections

Off-mesh connections join places the nav mesh's polys do not: a jump across a gap or down a ledge, a ladder, a
teleporter, a door. They are part of the nav mesh, not of a tile: each end attaches to the polys under it, and attaches
again as tiles are added and removed, so they work across tiles and survive rebuilding them. Each has an area and
flags, as polys do, and filters treat it as a node like any other.

### Point connections

A point connection goes from one point to another, one way or both ways.

```cpp
// a jump up onto the platform: from a point on the floor to a point on top, one way. each end attaches to the
// nearest poly within the radius, and attaches again as tiles come and go
const uint32_t jump = nav_mesh_api::add_off_mesh_connection(nav_mesh, {
                                                                           .start = {6.5f, 0, -3.8f},
                                                                           .end = {6.5f, 1, -6},
                                                                           .radius = 0.5f,
                                                                           .direction = OFF_MESH_CONNECTION_DIRECTION_START_TO_END,
                                                                           .flags = 0xffffffff,
                                                                           .area = 0,
                                                                       });
if (!nav_mesh_api::is_off_mesh_connection_connected(nav_mesh, jump)) printf("an end found no poly\n");
```

A path that takes it has an off-mesh point where it leaves, then the point where it lands:

```cpp
// a path up takes the jump: the point where it leaves the floor is an off-mesh point, and the next is where it lands
StraightPathPoint points[32];
FindPathResult path = {};
find_path::find_path(path, workspace, nav_mesh, {0, 0, -6}, {7, 1, -7}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER, points);
for (uint32_t i = 0; i + 1 < path.point_count; i++) {
    if (points[i].type == NODE_TYPE_OFFMESH) {
        printf("jump from %.2f %.2f %.2f to %.2f %.2f %.2f\n", points[i].position.x, points[i].position.y, points[i].position.z,
               points[i + 1].position.x, points[i + 1].position.y, points[i + 1].position.z);
    }
}
```

```cpp
nav_mesh_api::remove_off_mesh_connection(nav_mesh, jump);
```

### Segment connections

A segment connection is wide: it joins a whole edge to another, such as the length of a ledge to the floor below it,
and a path crosses it wherever along it suits the path. `start_a` maps to `end_a` and `start_b` to `end_b`, and each
point between to the point the same fraction along the other. Agents coming from different places cross at different
points, rather than all at one.

```cpp
// the whole front edge of the platform as one wide link: from anywhere along the floor's edge (start_a to start_b)
// up to the matching place along the top (end_a to end_b). it is split into a piece per pair of polys it joins,
// and a path crosses where its funnel meets it. inset keeps crossings this far from the ends
const uint32_t ledge = nav_mesh_api::add_off_mesh_segment_connection(nav_mesh, {
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
                                                                               });

// a link from a centre line and a width, as Unity's are
const OffMeshSegmentConnectionParams wide =
    nav_mesh_api::off_mesh_segment_connection_from_width({6.5f, 0, -4.4f}, {6.5f, 1, -5.6f}, 4, 0.6f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 0xffffffff, 0);
```

roam splits a segment connection into pieces, one per pair of polys it joins, and maps each piece's crossing at the
same fraction along both ends. `align` swaps `end_a` and `end_b` when that uncrosses the mapping. Segment connections
are roam's own: navcat has only point connections.

## Agents

### Crowds

`crowd` is navcat's crowd simulation: agents that plan paths spread over frames, follow them, steer around each other
and the walls, and take off-mesh connections. It uses the other agent blocks, `path_corridor`, `local_boundary` and
`obstacle_avoidance`. Use it as it is, or copy it into your project to change it.

```cpp
// a crowd for agents up to 0.6 m in radius
Crowd crowd = crowd::create(0.6f);
const AgentParams params = {
    .radius = 0.3f,
    .height = 1.8f,
    .max_acceleration = 15,
    .max_speed = 3.5f,
    .collision_query_range = 2,
    .separation_weight = 0.5f,
    .update_flags = CROWD_UPDATE_ANTICIPATE_TURNS | CROWD_UPDATE_OBSTACLE_AVOIDANCE | CROWD_UPDATE_SEPARATION | CROWD_UPDATE_OPTIMIZE_VIS |
                    CROWD_UPDATE_OPTIMIZE_TOPO,
};
uint32_t agents[4];
for (uint32_t i = 0; i < 4; i++) agents[i] = crowd::add_agent(crowd, nav_mesh, {-8, 0, -6 + 4 * (float)i}, params);

// everyone to the far side of the block: the target is a poly and a point on it
FindNearestPolyResult target = {};
nav_mesh_api::find_nearest_poly(target, nav_mesh, {8, 0, 2}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
for (uint32_t agent : agents) crowd::request_move_target(crowd, agent, target.node_ref, target.position);

// each frame: plan, steer, avoid each other and move
for (int frame = 0; frame < 600; frame++) crowd::update(crowd, nav_mesh, 1.0f / 60.0f);
for (uint32_t agent : agents) {
    const Vec3 position = crowd.agents[agent].position;
    printf("agent %u at %.2f %.2f %.2f%s\n", agent, position.x, position.y, position.z,
           crowd::is_agent_at_target(crowd, agent, 1.0f) ? ", arrived" : "");
}
```

An agent's `update_flags` choose what it does each update: `CROWD_UPDATE_ANTICIPATE_TURNS` slows into corners,
`_OBSTACLE_AVOIDANCE` steers around other agents, `_SEPARATION` keeps them apart (by `separation_weight`),
`_OPTIMIZE_VIS` shortcuts the path where the agent can see further along it, and `_OPTIMIZE_TOPO` searches again
nearby now and then for a better path. Each agent's fields, `position` and `velocity` among them, are yours to read
after an update.

```cpp
// steered by hand, as a player's input would: the agent keeps to the nav mesh and others avoid it
crowd::request_move_velocity(crowd, agents[0], {0, 0, 2});
// and stopped
crowd::reset_move_target(crowd, agents[0]);
crowd::remove_agent(crowd, agents[3]);
```

```cpp
crowd::release(crowd);
```

### Off-mesh connections in a crowd

By default an agent crosses an off-mesh connection in half a second, in a straight line. To animate it yourself (a
jump's arc, a ladder climb, a door that must open first), turn off `auto_traverse_off_mesh_connections`:

```cpp
// an agent that animates its own jumps: when it reaches an off-mesh connection, its state is AGENT_STATE_OFFMESH
// and its off_mesh_animation says from where to where. move it there, then hand it back to the crowd
AgentParams jumper_params = params;
jumper_params.auto_traverse_off_mesh_connections = false;
const uint32_t jumper = crowd::add_agent(crowd, nav_mesh, {6.5f, 0, -1}, jumper_params);
FindNearestPolyResult up = {};
nav_mesh_api::find_nearest_poly(up, nav_mesh, {7, 1, -7}, {1, 1, 1}, nav_mesh_api::DEFAULT_QUERY_FILTER);
crowd::request_move_target(crowd, jumper, up.node_ref, up.position);

for (int frame = 0; frame < 600; frame++) {
    crowd::update(crowd, nav_mesh, 1.0f / 60.0f);
    Agent &agent = crowd.agents[jumper];
    if (agent.state != AGENT_STATE_OFFMESH || !agent.off_mesh_animation.active) continue;
    OffMeshAnimation &animation = agent.off_mesh_animation;
    animation.t += 1.0f / 60.0f;
    const float progress = animation.t / 0.4f;
    if (progress >= 1) {
        crowd::complete_off_mesh_connection(crowd, jumper);
        continue;
    }
    const float arc = 4 * progress * (1 - progress);
    agent.position = vec3::lerp(animation.start_position, animation.end_position, progress);
    agent.position.y += arc;
}
```

### Path corridors

A path corridor is one agent's node path, kept in step with it as it moves: as the agent moves along the surface, the
polys it has passed are trimmed from the front. It is what a crowd agent follows, and a start for movement of your own.

```cpp
// a corridor is an agent's node path, kept in step with the agent as it moves: the crowd's agents each have one,
// and one is a start on movement of your own
FindNearestPolyResult start = {};
nav_mesh_api::find_nearest_poly(start, nav_mesh, {-8, 0, 0}, {1, 1, 1}, filter);
FindNearestPolyResult end = {};
nav_mesh_api::find_nearest_poly(end, nav_mesh, {8, 0, 0}, {1, 1, 1}, filter);
NodeRef node_path[256];
FindNodePathResult path = {};
nav_mesh_search::find_node_path(path, workspace, nav_mesh, start.node_ref, end.node_ref, start.position, end.position, filter, node_path);

PathCorridor corridor = path_corridor::create();
path_corridor::reset(corridor, start.node_ref, start.position);
path_corridor::set_path(corridor, end.position, {node_path, path.path_count});

for (int frame = 0; frame < 600; frame++) {
    // the next corners, from wherever the agent now is
    StraightPathPoint corners[3];
    FindCornersResult found = {};
    path_corridor::find_corners(found, corridor, nav_mesh, corners);
    if (found.corner_count == 0) break;

    // a step towards the first corner: the corridor moves along the surface and trims what is behind it
    const Vec3 to_corner = vec3::subtract(corners[0].position, corridor.position);
    const float distance = vec3::length(to_corner);
    if (distance < 0.01f) break;
    const Vec3 step = vec3::scale(to_corner, fminf(3.0f / 60.0f, distance) / distance);
    path_corridor::move_position(corridor, workspace, vec3::add(corridor.position, step), nav_mesh, filter);
}
printf("corridor at %.2f %.2f %.2f, %u polys left\n", corridor.position.x, corridor.position.y, corridor.position.z, corridor.path.length);
path_corridor::release(corridor);
```

`corridor_is_valid` checks that a corridor's next polys are still there and pass the filter, after tiles change, and
`fix_path_start`, `optimize_path_visibility` and `optimize_path_topology` repair and shorten it.

## Other blocks

### Flood fill

Generation marks every walkable surface, including ones no agent can get to: the tops of walls and furniture, pockets
inside closed geometry. A flood fill from points agents can be (spawn points, say) finds what is reachable and what
is not, to prune:

```cpp
// every poly reachable from the seeds, and every one that is not: the polys to prune, such as the tops of walls
// and enclosed pockets
Array<NodeRef> unreachable = {};
unreachable.reserve(nav_mesh_api::get_max_node_count(nav_mesh));
FloodFillNavMeshResult flood = {};
flood_fill_nav_mesh::flood_fill_nav_mesh(flood, nav_mesh, {&seed, 1}, {}, unreachable);
unreachable.resize(flood.unreachable_count);
printf("%u reachable, %u not\n", flood.reachable_total, flood.unreachable_total);
```

The "Flood Fill Pruning" demo builds a new nav mesh without the unreachable polys.

### Merging meshes

```cpp
// meshes from several sources as one, with vertices at the same position merged so their triangles connect
const InputTriangleMesh meshes[] = {
    {level.positions, level.indices},
    {extra_positions, extra_indices},
};
MergedMesh merged = merge_positions_and_indices::merge_positions_and_indices(meshes);
const InputTriangleMesh input = {merged.positions, merged.indices};
```

### Chunky triangle meshes

```cpp
// triangles bucketed in xz, for the triangles under an area without visiting them all: what the tiled generator
// builds each tile from
ChunkyTriMesh chunks = chunky_tri_mesh::create(input, 256);
Array<uint32_t> nodes = chunky_tri_mesh::get_chunks_overlapping_rect(chunks, {-4, -4}, {4, 4});
for (uint32_t node_index : nodes) {
    const ChunkyTriMeshNode &node = chunks.nodes[node_index];
    for (uint32_t i = 0; i < node.count; i++) {
        const uint32_t triangle = chunks.triangles[(uint32_t)node.index + i];
        (void)triangle;
    }
}
nodes.release();
chunky_tri_mesh::release(chunks);
```

## Debug drawing

`roam/debug/debug.h` turns each stage of generation, and the nav mesh itself, into primitives to draw:
`create_heightfield_helper`, `create_compact_heightfield_regions_helper`, `create_raw_contours_helper`,
`create_poly_mesh_helper`, `create_nav_mesh_helper`, `create_nav_mesh_bv_tree_helper`, `create_nav_mesh_links_helper`,
`create_nav_mesh_islands_helper`, `create_nav_mesh_off_mesh_connections_helper`, `create_search_nodes_helper` (the
last search's nodes) and more. They draw nothing themselves:

```cpp
// primitives for any renderer: triangles, lines, points or boxes, each with positions, colours and indices
DebugPrimitives primitives = debug::create_nav_mesh_helper(nav_mesh);
for (const DebugPrimitive &primitive : primitives) {
    if (primitive.type == DEBUG_PRIMITIVE_TRIANGLES) {
        printf("%u triangles\n", primitive.indices.length / 3);
    } else if (primitive.type == DEBUG_PRIMITIVE_LINES) {
        printf("%u lines\n", primitive.positions.length / 6);
    }
}
debug::release(primitives);
```

## Differences from navcat

roam gives navcat's results, with a few exceptions on purpose:

- **Additions:** segment off-mesh connections, `is_reachable`, a binary save format where navcat's data is JSON, and
  `Crowd::max_neighbours`, an optional cap on the neighbours each agent considers.
- **Refs:** 0 is a valid ref, and none is `INVALID_NODE_REF`. Stale refs are rejected everywhere, not only by
  `is_valid_node_ref`.
- **Edge cases:** an empty input builds an empty nav mesh, and too many regions is an error in every region builder.
  Smooth paths take zero-length steps without producing NaNs.
- **Numbers:** roam computes in `float` where navcat computes in JavaScript's doubles. Ties between equal values (in the
  detail mesh, and in paths at exact vertex and edge ties) can go the other way, and BV tree bounds are quantised to
  16 bits.

## Demos

`demos/` is an app of 30 examples, 29 ported from navcat's and one for segment connections: generation (solo, tiled,
step by step, custom areas, your own glTF), each query, off-mesh connections, crowds, a character controller, flow
fields and more. Each is one file in `demos/examples/`, and they are the place to see roam used in full. The app builds
natively and with Emscripten.

```sh
demos/native.sh                 # build and run natively
demos/native.sh "find path"     # open an example by name
demos/web.sh                    # build with Emscripten (needs EMSDK) and serve on localhost:8080 (PORT overrides)
demos/smoke.sh                  # run every example headlessly for 30 frames, failing on crashes and leaks
```

## Tests and benchmarks

```sh
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
```

The tests include navcat's own, fuzzing of queries, tile churn and generation inputs, and a leak check on every test.
Parity tests run each stage of generation and a set of queries through navcat and roam and compare the results. They
build when Node and a navcat checkout are found (`ROAM_NAVCAT_DIR`, `../navcat` by default), with navcat's `dist`
built. Where roam differs from navcat on purpose (see below), a stock navcat fails those comparisons by design.

`roam_bench` times generation and queries on navcat's bench fixture, best built with the `release` preset:
`build/release/roam_bench --fixture <navcat>/bench/fixtures/nav-test.json`.

## Acknowledgements

- roam is a port of [navcat](https://github.com/isaac-mason/navcat).
- navcat's generation and queries build on the approach of [Recast & Detour](https://github.com/recastnavigation/recastnavigation)
  by Mikko Mononen.
