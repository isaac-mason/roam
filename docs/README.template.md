<!-- built from docs/README.template.md by docs/build.mjs: edit the template, not this file -->

# roam

roam is a C++ navigation mesh construction and querying library for 3D floor-based navigation.

roam is ideal for games, simulations and tools that need agents to find their way around complex 3D environments.

roam started life as a port of [navcat](https://github.com/isaac-mason/navcat), which is a javascript library based on [recastnavigation](https://github.com/recastnavigation/recastnavigation) that recieved improvements to offmesh links and string pulling functionality. roam has expanded on both in introducing new features like islands and segment-to-segment offmesh links.

**Features**

- Navigation mesh generation from 3D geometry
- Navigation mesh querying: paths, raycasts, moving along the surface, random points, reachability
- Single and multi-tile navigation mesh support
- Point and segment off-mesh connections that live on the nav mesh, not in tiles
- Crowd simulation, path corridors and obstacle avoidance
- A binary save format that loads back to the same refs and the same query results
- C++20 with no dependencies: no C++ standard library, no exceptions, no RTTI, one allocator hook
- Works with any engine or renderer: debug helpers produce plain primitives for you to draw

**Core and blocks**

The core, `roam`, has the steps of generation, the nav mesh, its queries and debug drawing.

On top of it sit the blocks, `roam_blocks`: the solo and tiled nav mesh generators, the crowd, flood fill pruning and
some geometry helpers. Blocks are half library, half example. Use them as they are to get going, and when your needs
outgrow them, copy them into your project and make them yours. They use only roam's public headers, so there is
nothing hidden to untangle. See [Generation presets](#generation-presets-half-library-half-example).

**Examples**

`demos/` is an app of 30 examples, each one file in [`demos/examples/`](./demos/examples). They are the place to see
roam used in full. Try them in your browser at [isaac-mason.github.io/roam](https://isaac-mason.github.io/roam/):
each picture below opens its example there, and "source" opens its code. The app also builds natively, see
[Demos](#demos).

<Examples />

## Table of Contents

<TOC />

## What is a Navigation Mesh?

A navigation mesh (or navmesh) is a simplified representation of a 3D environment that is used for pathfinding and AI
navigation in games and simulations. It consists of interconnected convex polygons that cover the walkable areas of the
environment. Polygons are connected by their shared edges and by off-mesh connections, allowing agents (characters) to
move from one polygon to another.

![./docs/1-whats-a-navmesh](./docs/1-whats-a-navmesh.png)

## Can roam be integrated with my engine?

roam knows nothing of rendering, physics or any engine, so it works with whichever you use, or your own. It takes
triangles in and gives positions, polygons and paths out. The debug helpers turn its data into plain triangles, lines,
points and boxes for you to draw however you draw things.

roam's conventions:

- Positions are `Vec3 {x, y, z}` with y up, in a right-handed coordinate system, as in OpenGL.
- Triangles are wound counter clockwise seen from above.
- `float` everywhere. The examples use metres, but roam has no units of its own.

If your environment uses a different coordinate system, transform coordinates going into and out of roam.

If you are importing a navmesh created externally, its polygons must share vertices where they meet: neighbours are
found by shared edges.

## Getting roam

### CMake

roam's `CMakeLists.txt` makes two libraries:

- `roam` (alias `roam::roam`): generation, queries and debug drawing.
- `roam_blocks` (alias `roam::blocks`): the generators, agents and helpers built on `roam`. It links
  `roam`.

Fetch roam, or add a checkout (a git submodule, say) with `add_subdirectory(path/to/roam)`, then link a target. roam's
headers need C++20, and linking either target asks for it:

```cmake
cmake_minimum_required(VERSION 3.24)
project(game LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(roam GIT_REPOSITORY https://github.com/isaac-mason/roam.git GIT_TAG main)
FetchContent_MakeAvailable(roam)

add_executable(game main.cpp)
target_link_libraries(game PRIVATE roam::blocks)
```

Once you have ejected every block you use, link `roam::roam` alone.

The tests and benchmarks build only when roam is the top-level project. Its options:

- `ROAM_BUILD_TESTS` and `ROAM_BUILD_BENCH`: the tests and the benchmarks.
- `ROAM_BUILD_DEMOS`: the demo app, which fetches sokol, Dear ImGui and cgltf. Off by default.

### Without CMake

Compile every `.cpp` under `roam/` with the repository root on the include path (headers are included as
`"roam/query/nav_mesh_api.h"`), as C++20. Leave out `roam/blocks/` if you use none of it. roam's own targets build with
`-fno-exceptions -fno-rtti -ffp-contract=off`; the last keeps results from depending on whether the compiler fuses
multiply-adds.

```sh
c++ -std=c++20 -O2 -fno-exceptions -fno-rtti -ffp-contract=off -I path/to/roam main.cpp $(find path/to/roam/roam -name '*.cpp')
```

## Quick Start / Minimal Example

Below is a minimal example of using the `generate_solo_nav_mesh` block to generate a navigation mesh, and then using
roam's core to find a path on it: a floor with a block in the middle, and a path from one side of the block to the
other.

For how to tune these options, and how the generation process works under the hood with images, see the
[Navigation Mesh Generation](#navigation-mesh-generation) section below.

<Snippet source="./snippets/quickstart.cpp" select="quickstart" />

It prints the path's corners, from the start, around the block and on to the end:

```
found a path, 4 points
  -8.00 0.20 0.00
  -2.20 0.20 2.60
  2.20 0.20 2.60
  8.00 0.20 0.00
```

Below is a quick summary of the generation options used above, and how to start tuning them:

| Option | Description | Range / heuristic for 1 = 1 m humanoid agents |
| --- | --- | --- |
| `cell_size` | Horizontal voxel size (XZ). Smaller = finer detail, slower generation. | ≈ `walkable_radius_world / 3` |
| `cell_height` | Vertical voxel size (Y). Controls height resolution. | ≈ `walkable_climb_world / 2` |
| `walkable_radius_world` | Agent radius. The walkable area is eroded by it, so paths keep the agent's body off walls. | 0.2–0.5 m |
| `walkable_height_world` | Agent height. Areas with ceilings lower than this are excluded. | 1.6–2.0 m |
| `walkable_slope_angle_degrees` | Steepest walkable slope. This filters input triangles at the very start of generation. | 35–50° |
| `walkable_climb_world` | Max step height, up or down. This filters at the heightfield stage. | 0.3–0.5 m |
| `border_size` | Voxels of padding around each tile. 0 for a solo nav mesh. | tiled: `walkable_radius_voxels + 3` |
| `min_region_area` | Smallest isolated region kept, in voxels. | 4–16 |
| `merge_region_area` | Regions smaller than this merge into neighbours, in voxels. | 8–32 |
| `max_simplification_error` | How far, in voxels, simplified outlines may stray from the voxels (higher = simpler mesh). | 1–2 |
| `max_edge_length` | Longest polygon edge along walls before it is split, in voxels. 0 for no limit. | 8–24 |
| `max_vertices_per_poly` | Max vertices per polygon. 3 gives triangles. | 3–6 |
| `detail_sample_distance` | Distance between height samples for the detail mesh. | `cell_size * 4–8` |
| `detail_sample_max_error` | Allowed height deviation of the detail mesh from its samples. | `cell_height * 1–2` |

Each agent measure has a voxel field and a world field: the voxels drive the build, and the world values are kept in
the tiles. `derive_sizes` fills the voxel fields from the world ones, rounding up; set them yourself to choose
otherwise.

The rest of this guide leaves out the includes and `using namespace roam;`. Most of its examples share one level, the
quick start's floor and block plus a platform 1 m up, its triangles in `Array`s:

<Snippet source="./snippets/level.cpp" select="level" />

## Reading the API

### Conventions

- Everything is in `namespace roam`. Types are directly in it (`NavMesh`, `QueryFilter`, `StraightPathPoint`). Functions
  and constants are in a namespace per module, named after its header: `nav_mesh_api::add_tile` is in
  `roam/query/nav_mesh_api.h`, `find_path::find_path` in `roam/query/find_path.h`. Folders are only for finding files.
- Functions are snake_case, types PascalCase, constants and enum values UPPER_SNAKE_CASE: `NODE_TYPE_OFFMESH`,
  `OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL`.
- Data is plain structs and their fields are yours to read. Settings structs take designated initializers, which C++20
  requires in declaration order.
- A function that builds or changes something takes it first (after a `BuildContext` when it logs):
  `heightfield::rasterize_triangles(context, heightfield, ...)`, `nav_mesh_api::add_tile(nav_mesh, tile)`.
- Lifecycles: `create` makes something that owns memory, and `release` frees what a value holds.
  `nav_mesh_api::create_nav_mesh` and `nav_mesh_api::release`, `crowd::create` and `crowd::release`. Only what you
  create owns memory. See [Memory](#memory).
- Math returns by value: `vec3::add(a, b)`, `box3::expand_by_point(box, point)`.

### Results and spans

Queries write their result into `out`, their first parameter. A result is a plain value: flags, refs, positions and
counts. It never owns memory, so it needs no release, and it can be reused from call to call.

Anything of variable size goes into a buffer of yours, passed as a `Span<T>`: a pointer and a `length`. A span converts
from a C array, from an `Array` (roam's growable array), or is built as `{data, length}`. A writable span of an
`Array` covers its capacity; a read-only one, `Span<const T>`, covers its length. Here with `start` and `end` from
[`find_nearest_poly`](#find_nearest_poly):

<Snippet source="./snippets/memory.cpp" select="spans" />

A result names its counts after the buffer: `path_count` is how many it wrote, `path_total` the true size where that
is known, and `truncated` says the buffer filled up. Searches take a `QueryWorkspace &` after `out`: the search nodes
and scratch arrays they reuse from call to call, so that a warm query allocates nothing. A workspace serves one query
at a time.

### Public headers

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
allocates. Every block comes back with the size and alignment it was asked for, so your allocator needs no headers of
its own. roam asks for at most `alignof(max_align_t)`:

<Snippet source="./snippets/memory.cpp" select="counting_allocator" />

<Snippet source="./snippets/memory.cpp" select="set_allocator" />

What owns memory: anything made by a `create` (a nav mesh, a workspace, a crowd, a corridor, a heightfield), the
generators' results and intermediates, and `Array`s. Each is freed by its module's `release`: `nav_mesh_api::release`,
`nav_mesh_search::release`, `crowd::release`, `generate_solo_nav_mesh::release`, `heightfield::release`, or an
`Array`'s own `release()`. Results, spans, `QueryFilter`s, params and settings hold no memory.

roam built for debug, or with `ROAM_TRACK_MEMORY` defined, counts what it holds:

<Snippet source="./snippets/memory.cpp" select="memory_stats" />

`Scoped<T>` (`roam/util/scoped.h`) is an opt-in holder that releases a value as it leaves scope, for code that wants it:

<Snippet source="./snippets/memory.cpp" select="scoped" />

### Errors

Running out of memory stops the program: when the allocator returns null, roam prints the size it asked for and
aborts. Nothing in roam fails for want of memory, so nothing checks for it.

Everything else that can fail says so in its result: `success` and `flags` on a query's result, a `bool` from a
generation step, a `LoadStatus` from loading. Generation steps that fail log why in their `BuildContext`. Invalid and
stale refs, positions far outside the nav mesh and NaNs are failures, not crashes. `ROAM_ASSERT` checks roam's own
invariants in debug builds, and in release builds with `ROAM_ENABLE_ASSERTS` defined.

## Navigation Mesh Querying

This section covers the main features you'll use for navigation mesh querying: pathfinding, spatial queries and
surface movement. For lower-level APIs and the nav mesh's internals, see
[Advanced Navigation Mesh APIs](#advanced-navigation-mesh-apis).

Queries take a `QueryFilter`, which says which polys a query may use and what crossing them costs. Use
`nav_mesh_api::DEFAULT_QUERY_FILTER` (every poly with any flag set, costed by distance) until you need your own; see
[Custom Query Filters and Custom Area Types](#custom-query-filters-and-custom-area-types). The examples here use the
shared level's solo nav mesh, a `QueryWorkspace` and the default filter, as `filter`.

### `find_path`

`find_path` is the whole of a path in one call: `find_nearest_poly` for the start and end, `find_node_path` between
them, and `find_straight_path` for the node path's corners. Its points are where an agent turns, and where it goes onto
an off-mesh connection.

**When to use:** This is the simplest way to find a complete path. Use it for one-off pathfinding queries, or when you
aren't steering agents along a path and re-querying frequently.

<Snippet source="./snippets/queries.cpp" select="find_path" />

`truncated` is set when the points buffer or a node path buffer you pass fills up. When the end cannot be reached, the
path goes to the reachable poly nearest it, `success` is still true, and `flags` has `FIND_PATH_RESULT_PARTIAL_PATH`.

<Snippet source="./snippets/queries.cpp" select="find_path_raycast" />

<ExamplesTable ids="example_find_path" />

### `find_smooth_path`

`find_smooth_path` combines `find_node_path`, `find_straight_path` and `move_along_surface`: it walks the path in small
steps along the surface, each point on the detail mesh.

**When to use:** When you want a path that follows hills and stairs without sharp corners, and you need it
infrequently, such as for drawing a preview rather than for many agents a frame.

<Snippet source="./snippets/queries.cpp" select="find_smooth_path" />

<ExamplesTable ids="example_find_smooth_path" />

### `find_node_path`

`find_node_path` is the search alone: the polys and off-mesh connections from start to end, by A* over the nav mesh's
graph.

**When to use:** When you want to keep a node path and recalculate its straight path many times, such as for an agent
whose position changes while its destination stays the same. This is cheaper than calling `find_path` again and again.

<Snippet source="./snippets/queries.cpp" select="find_node_path" />

<ExamplesTable ids="example_find_node_path" />

### `find_straight_path`

`find_straight_path` performs "string pulling" to turn a node path into the corners an agent should walk to, as
`find_path` does after its search. Each point says what it is in `flags` (`STRAIGHT_PATH_POINT_START`, `_END`,
`_OFFMESH`), its node's `type`, and the `node_ref` it leaves from. The start and end are clamped to the first and last
polys of the node path.

**When to use:** After `find_node_path`, to get the actual waypoints. You might call it often while keeping the same
node path, or when implementing path following of your own.

<Snippet source="./snippets/queries.cpp" select="find_straight_path" />

<ExamplesTable ids="example_find_straight_path" />

### Sliced searches

A sliced search runs a node path search a number of iterations at a time. The query holds its own search state, so
several can be in progress at once.

**When to use:** To spread long searches over frames so that no one frame pays for them. The crowd plans its agents'
paths this way.

<Snippet source="./snippets/queries.cpp" select="sliced" />

`finalize_sliced_find_node_path_partial` finishes a search early: the path to the furthest node of an existing path
that the search has reached.

### `move_along_surface`

`move_along_surface` moves a point across polys towards a target, sliding along the walls it meets and staying on the
nav mesh. It looks only at polys near the start, so keep moves short: a frame's worth.

**When to use:** For character controllers constrained to the nav mesh without full pathfinding: local movement,
sliding along walls, or movement logic of your own that respects the nav mesh.

<Snippet source="./snippets/queries.cpp" select="move_along_surface" />

<ExamplesTable ids="example_navmesh_constrained_character_controller,example_move_along_surface" />

### `raycast` & `raycast_with_costs`

`raycast` walks a straight line across the nav mesh's surface (in 2D, ignoring height) from a point on a poly, and stops
at the first wall. `t` is how far along it got, from 0 to 1, or `FLT_MAX` when nothing stopped it.

**When to use:** Lines of sight across the ground, checking that a shortcut is walkable, finding walls. It is best
suited to short-range checks, given its two dimensional nature.

<Snippet source="./snippets/queries.cpp" select="raycast" />

<Snippet source="./snippets/queries.cpp" select="raycast_with_costs" />

<ExamplesTable ids="example_raycast" />

### `find_nearest_poly`

`find_nearest_poly` finds the poly nearest a point in the world, within a box of half extents around it, and the
nearest point on that poly.

**When to use:** This is often your first step: use it to snap world positions onto the nav mesh before pathfinding
or querying, when placing agents, or to check whether a position is on the nav mesh.

<Snippet source="./snippets/queries.cpp" select="find_nearest_poly" />

<ExamplesTable ids="example_find_nearest_poly" />

### `find_random_point` & `find_random_point_around_circle`

`find_random_point` finds a random point anywhere on the nav mesh, polys weighted by area.
`find_random_point_around_circle` finds one within a radius of a point, through polys connected to its own.

**When to use:** Spawn points, patrol destinations, procedural placement, scatter formations, "move near the target"
behaviours.

Both take a function that returns a float in [0, 1), and its user data:

<Snippet source="./snippets/queries.cpp" select="next_random" />

<Snippet source="./snippets/queries.cpp" select="random_points" />

<ExamplesTable ids="example_find_random_point,example_find_random_point_around_circle" />

### `is_reachable`

Every nav mesh keeps track of which polys connect to which, as tiles and off-mesh connections come and go.
`is_reachable` answers from that in well under a microsecond, without searching. It follows one-way off-mesh
connections in their direction and ignores filters.

**When to use:** Before a search that might fail: a search for an unreachable end visits every poly it can reach
before giving up with a partial path.

<Snippet source="./snippets/queries.cpp" select="is_reachable" />

`debug::create_nav_mesh_islands_helper` draws the connected islands.

### `find_local_neighbourhood`

`find_local_neighbourhood` finds the polys around a point, out to a radius, that connect to its own and do not overlap
each other: the area an agent can see around it. `get_poly_wall_segments` gives a poly's walls, and its portals too
when asked.

**When to use:** Local steering and wall avoidance. The crowd uses them to keep agents off walls.

<Snippet source="./snippets/queries.cpp" select="find_local_neighbourhood" />

## Crowd Simulation

The `crowd` block (`roam/blocks/agents/crowd.h`) is a crowd simulation, built on roam's pathfinding and local
steering:

- Agent management: add and remove agents, set target positions or velocities
- Pathfinding spread over frames, to keep performance steady with many agents
- Agent-to-agent and wall avoidance
- Off-mesh connections, with hooks to animate them yourself

It is made of the other agent blocks: `path_corridor` for each agent's node path, `local_boundary` for the walls
around it and `obstacle_avoidance` for steering. Like every block, it is half library and half example: for simple use
cases use it directly, and for more advanced ones copy it, and the agent blocks it uses, into your project and change
them as needed.

<Snippet source="./snippets/agents.cpp" select="crowd" />

An agent's `update_flags` choose what it does each update: `CROWD_UPDATE_ANTICIPATE_TURNS` slows into corners,
`_OBSTACLE_AVOIDANCE` steers around other agents, `_SEPARATION` keeps them apart (by `separation_weight`),
`_OPTIMIZE_VIS` shortcuts the path where the agent can see further along it, and `_OPTIMIZE_TOPO` searches again
nearby now and then for a better path. `add_agent` places the agent on the nearest poly within the crowd's
`agent_placement_half_extents`; with none in reach it still returns an id, but the agent's `state` is
`AGENT_STATE_INVALID` and the crowd leaves it where it is. Each agent's fields, `position` and `velocity` among them,
are yours to read after an update.

<Snippet source="./snippets/agents.cpp" select="crowd_steer" />

<Snippet source="./snippets/agents.cpp" select="crowd_release" />

<ExamplesTable ids="example_crowd_simulation,example_crowd_simulation_stress_test" />

### Off-mesh connections in a crowd

By default an agent crosses an off-mesh connection in half a second, in a straight line. To animate it yourself (a
jump's arc, a ladder climb, a door that must open first), turn off `auto_traverse_off_mesh_connections`:

<Snippet source="./snippets/agents.cpp" select="crowd_off_mesh" />

### Path corridors

A path corridor is one agent's node path, kept in step with it as it moves: as the agent moves along the surface, the
polys it has passed are trimmed from the front. It is what a crowd agent follows, and a start for movement of your own.

<Snippet source="./snippets/agents.cpp" select="path_corridor" />

`corridor_is_valid` checks that a corridor's next polys are still there and pass the filter, after tiles change, and
`fix_path_start`, `optimize_path_visibility` and `optimize_path_topology` repair and shorten it.

## Custom Pathfinding Algorithms

If your use case is not covered by the built-in queries, the nav mesh's structure is all there to read: its nodes, the
links between them, and each tile's polys, vertices and detail meshes. Searches of your own walk the same graph roam's
do. [Advanced Navigation Mesh APIs](#advanced-navigation-mesh-apis) covers how.

These examples search the graph their own way:

<ExamplesTable ids="example_flow_field_pathfinding,example_find_shortest_paths,example_find_diverse_paths" />

## Navigation Mesh Generation

### Overview

Navigation mesh generation is the process of turning 3D geometry into a graph of walkable polygons. The graph is then
used for pathfinding and navigation queries.

#### The Structure of a Navigation Mesh

A navigation mesh is organised into one or more **tiles**. Each tile holds walkable polygons, their height detail and a
bounding volume tree. For most projects, a single tile covering your whole level is perfect. For larger or dynamic
worlds, you can split the nav mesh into a grid of tiles.

Behind the tiles, roam keeps a graph of **nodes** (one per poly and one per off-mesh connection) and the **links**
between them. This graph is what powers pathfinding: a path query searches it for a route.

#### Single-Tile vs Tiled Navigation Meshes

Most projects should start with a **single-tile nav mesh**: it is simpler and covers most use cases.

Consider **tiled nav meshes** when you need:

- Dynamic updates: rebuild only the tiles whose geometry changed
- Memory management: stream tiles in and out around the player
- Parallel generation: tiles build independently
- Large worlds: a solo build has to hold the whole level's heightfield at once, and tiled generation can give better
  results over large areas

How you manage tiles is up to you. You can build and add every tile for a level at once, or add and remove tiles at
runtime.

When you remove a tile and add one again, its polys get new node refs, and refs kept from before no longer resolve.
Check refs you hold with `is_valid_node_ref` before using them. See [Tiles](#tiles).

### Generation presets: half library, half example

`roam/blocks/generators/` has two presets that bundle the steps of generation into one call:
`generate_solo_nav_mesh` and `generate_tiled_nav_mesh`. They are blocks, and blocks are written to be both used and
copied: half library, half example.

- **Use them as a library** while your needs are simple. Link `roam::blocks`, fill in the options, call it.
- **Read them as an example.** Each is short (around 110 and 200 lines) and is nothing more than the steps of the
  [deep dive](#generation-process-deep-dive) below, in order, with the usual choices made. The intermediates they keep
  are every stage's output, ready for the [debug helpers](#debug-utilities).
- **Eject when you outgrow them.** The presets make choices for you: watershed regions, area 0 and flags 1 on every
  walkable poly, every intermediate kept. When you need other choices, don't wrap the preset and work around it: copy it
  into your project and change it. See [Ejecting](#ejecting).

#### Solo nav meshes

`generate_solo_nav_mesh` builds a nav mesh of one tile over the whole input. Start here: it suits most levels.

<Snippet source="./snippets/generation.cpp" select="solo" />

<ExamplesTable ids="example_solo_nav_mesh" />

#### Tiled nav meshes

`generate_tiled_nav_mesh` builds a grid of tiles, each from the triangles under it and its border, found through a
[chunky triangle mesh](#geometry-blocks).

<Snippet source="./snippets/generation.cpp" select="tiled" />

<ExamplesTable ids="example_tiled_nav_mesh" />

#### Ejecting

Ejecting a preset is copying it:

1. Copy `roam/blocks/generators/generate_solo_nav_mesh.h` and `.cpp` (or the tiled pair) into your project.
2. Rename the namespace and the include path to your own, so that your copy and roam's never clash:
   `game::nav_mesh_build::build`, say.
3. Change it. Some common reasons:
   - mark areas from your level's volumes into the compact heightfield, such as water, roads or doors (step 3)
   - rasterise shapes that are not triangles, such as `heightfield::rasterize_box` or `rasterize_capsule` (step 2)
   - `build_regions_monotone` for speed, or `build_layer_regions` for tiles with overlapping floors (step 4)
   - `erode_and_mark_walkable_areas` for several agent sizes on one nav mesh (step 3)
   - your own areas and flags per poly, rather than area 0 and flags 1 for everything (step 6)
   - release each intermediate as soon as the next step has it, to keep peak memory down
   - build tiles on worker threads, or only the tiles a change touched
4. Once nothing of yours includes `roam/blocks/`, link `roam::roam` alone.

Nothing in a block is private. The generators include only roam's core headers, plus `chunky_tri_mesh` for the tiled
one, which you can keep from `roam::blocks` or eject with it. The same goes for every block: the crowd and its agent
pieces, flood fill and the geometry helpers are all there to be used as they are or copied and changed.

### Generation Process: Deep Dive

This section is a deep dive into how navigation mesh generation works. Understanding the process is useful for tuning
parameters to get the best results for your environment and agents, and it is exactly what the presets do: read it
next to [`generate_solo_nav_mesh.cpp`](./roam/blocks/generators/generate_solo_nav_mesh.cpp).

Generation works by voxelising the input. At a high level:

- Input triangles are rasterised into voxels, a heightfield
- Voxels where agents (described by your parameters) cannot be are filtered out
- The walkable space the voxels describe is divided into simple polygonal regions
- Navigation mesh polygons are made from the regions' outlines

Every step is public. The images below are drawn with the debug helper named in each step.

To follow along, read [`generate_solo_nav_mesh.cpp`](./roam/blocks/generators/generate_solo_nav_mesh.cpp), or start
from these examples:

<ExamplesTable ids="example_custom_nav_mesh_generation,example_mark_compact_heightfield_areas,example_rasterize_filled_volume" />

#### 0. Input and setup

The input is a set of 3D triangles describing the environment: an `InputTriangleMesh`, which borrows a span of
positions (3 floats a vertex) and a span of indices (3 a triangle). Pass what an agent might stand on and whatever gets
in its way. Leave out what agents walk through (foliage, say), decorative geometry that shouldn't affect navigation,
and anything that moves.

Positions follow the OpenGL conventions: right-handed, y up, triangles wound counter clockwise.

Generation emits diagnostic messages, warnings and errors into a build context.

<Snippet source="./snippets/generation.cpp" select="input" />

![2-1-navmesh-gen-input](./docs/2-1-navmesh-gen-input.png)

<Declarations of="roam/generate/build_context.h#BuildContext" />

#### 1. Mark walkable triangles

The first step finds the walkable triangles, by checking each one's slope against the steepest walkable slope.
Walkable triangles are marked with `WALKABLE_AREA` (`1`).

<Snippet source="./snippets/generation.cpp" select="walkable_triangles" />

![2-2-walkable-triangles](./docs/2-2-navmesh-gen-walkable-triangles.png)

<Declarations of="roam/generate/input_triangle_mesh.h#mark_walkable_triangles roam/debug/debug.h#create_triangle_area_ids_helper" />

#### 2. Rasterize triangles into a heightfield, do filtering with the heightfield

The triangles are then voxelised into a heightfield: columns of solid spans, each carrying the walkability of the
triangles that made it.

The heightfield is then filtered to remove spans where an agent cannot stand: low obstacles it can step over become
walkable, while ledges and spans without the headroom it needs do not.

The heightfield's resolution greatly affects both the fidelity of the nav mesh and how long generation takes.

`heightfield::rasterize_sphere`, `rasterize_capsule`, `rasterize_cylinder`, `rasterize_box` and `rasterize_convex` add
shapes that are not triangles (see [Rasterize Filled Volume](./demos/examples/example_rasterize_filled_volume.cpp)).

<Snippet source="./snippets/generation.cpp" select="rasterize" />

![2-3-heightfield](./docs/2-3-navmesh-gen-heightfield.png)

<Declarations of="roam/generate/heightfield.h#create roam/generate/heightfield.h#rasterize_triangles roam/generate/heightfield.h#filter_low_hanging_walkable_obstacles roam/generate/heightfield.h#filter_ledge_spans roam/generate/heightfield.h#filter_walkable_low_height_spans roam/debug/debug.h#create_heightfield_helper" />

#### 3. Build compact heightfield, erode walkable area, mark areas

The heightfield is then compacted to the open space above each walkable span, with each span's neighbours.

The compact heightfield is eroded by the agent's radius, so that the nav mesh keeps an agent of that radius off walls.

Areas marked now become the area of the polys over them: `mark_box_area`, `mark_rotated_box_area`,
`mark_convex_poly_area` and `mark_cylinder_area` (see
[Mark Compact Heightfield Areas](./demos/examples/example_mark_compact_heightfield_areas.cpp)).

<Snippet source="./snippets/generation.cpp" select="compact_heightfield" />

![2-4-compact-heightfield](./docs/2-4-navmesh-gen-compact-heightfield.png)

`erode_and_mark_walkable_areas` erodes by the smallest agent's radius and marks narrower areas with area ids of their
own, for several agent sizes on one nav mesh: filters for larger agents then exclude those areas. It needs a cell size
small enough to represent narrow passages, thresholds of more than a voxel or two, and often does better with
`build_regions_monotone` (see [Multiple Agent Sizes](./demos/examples/example_multiple_agent_sizes.cpp)).

<Declarations of="roam/generate/compact_heightfield.h#build roam/generate/compact_heightfield.h#erode_walkable_area roam/generate/compact_heightfield.h#erode_and_mark_walkable_areas roam/debug/debug.h#create_compact_heightfield_solid_helper" />

#### 4. Build compact heightfield regions

The compact heightfield is then split into regions, each simple enough to become polygons.

Watershed partitioning, the presets' choice, first computes a distance field: each span's distance from the nearest
edge.

<Snippet source="./snippets/generation.cpp" select="regions" />

![2-5-distance-field](./docs/2-5-navmesh-gen-compact-heightfield-distances.png)

![2-6-regions](./docs/2-6-navmesh-gen-compact-heightfield-regions.png)

Watershed gives the best polygons. `build_regions_monotone` sweeps the heightfield instead, which is fastest, needs no
distance field and makes no overlapping regions, but can give long thin polygons. `build_layer_regions` handles
overlapping walkable areas by making separate layers, and suits tiles with floors above each other.

<Declarations of="roam/generate/compact_heightfield_regions.h#build_distance_field roam/generate/compact_heightfield_regions.h#build_regions roam/generate/compact_heightfield_regions.h#build_regions_monotone roam/generate/compact_heightfield_regions.h#build_layer_regions roam/debug/debug.h#create_compact_heightfield_distances_helper roam/debug/debug.h#create_compact_heightfield_regions_helper" />

#### 5. Build contours from compact heightfield regions

Contours are traced around the edges of the regions, then simplified to fewer vertices while keeping their overall
shape.

<Snippet source="./snippets/generation.cpp" select="contours" />

![2-7-raw-contours](./docs/2-7-navmesh-gen-raw-contours.png)

![2-8-simplified-contours](./docs/2-8-navmesh-gen-simplified-contours.png)

<Declarations of="roam/generate/contour_set.h#build_contours roam/debug/debug.h#create_raw_contours_helper roam/debug/debug.h#create_simplified_contours_helper" />

#### 6. Build polygon mesh from contours, build detail mesh

From the simplified contours, a polygon mesh is made: convex polygons covering the walkable areas. Each poly carries an
area (what it is) and flags (who may use it), both yours to set.

A detail triangle mesh is also built, to keep the heights the polygons lost, sampled from the compact heightfield.

<Snippet source="./snippets/generation.cpp" select="poly_mesh" />

![2-9-poly-mesh](./docs/2-9-navmesh-gen-poly-mesh.png)

![2-10-detail-mesh](./docs/2-10-navmesh-gen-detail-mesh.png)

<Declarations of="roam/generate/poly_mesh.h#build_poly_mesh roam/generate/poly_mesh_detail.h#build_poly_mesh_detail roam/debug/debug.h#create_poly_mesh_helper roam/debug/debug.h#create_poly_mesh_detail_helper" />

#### 7. Convert build-time poly mesh and poly mesh detail to runtime navmesh tile format

Next, the poly mesh and detail mesh are converted from the build-time format to the runtime tile format, which works
out each poly's neighbours along its edges.

<Snippet source="./snippets/generation.cpp" select="convert" />

<Declarations of="roam/generate/nav_mesh_tile.h#poly_mesh_to_tile_polys roam/generate/nav_mesh_tile.h#poly_mesh_detail_to_tile_detail_mesh" />

#### 8. Assemble the navigation mesh

Finally, the tile is built (which makes its BV tree) and added to a nav mesh, ready for queries.

<Snippet source="./snippets/generation.cpp" select="nav_mesh" />

![./docs/1-whats-a-navmesh](./docs/1-whats-a-navmesh.png)

<Snippet source="./snippets/generation.cpp" select="release_intermediates" />

A tiled build does the same per tile, over the triangles under that tile and its border, with the tile's grid position
in the tile params and the nav mesh's `origin`, `tile_width` and `tile_height` set once for the grid. Read
[`generate_tiled_nav_mesh.cpp`](./roam/blocks/generators/generate_tiled_nav_mesh.cpp) for how.

<Declarations of="roam/query/nav_mesh_api.h#create_nav_mesh roam/query/nav_mesh_api.h#build_tile roam/query/nav_mesh_api.h#add_tile roam/query/nav_mesh_api.h#remove_tile roam/debug/debug.h#create_nav_mesh_helper" />

### Post-Processing

A common post-processing step is to flood fill the nav mesh from seed points where agents can be (spawn points, say),
to find areas no agent can reach. Generation marks every walkable surface, including the tops of walls and furniture
and pockets inside closed geometry.

The `flood_fill_nav_mesh` block finds what is reachable and what is not:

<Snippet source="./snippets/blocks.cpp" select="flood_fill" />

The Flood Fill Pruning example builds a new nav mesh without the unreachable polys:

<ExamplesTable ids="example_flood_fill_pruning" />

### Custom Query Filters and Custom Area Types

Each poly has an `area` and `flags`. The area is what the ground is (grass, water, a road); the flags say who may use
it (walkers, swimmers, vehicles). Both are set at build time, or after, and both are yours to give meaning to. The
deep dive above uses these:

<Snippet source="./snippets/areas.h" select="area_constants" />

The presets give their polys area 0 and flags 1. Built step by step, polys start with flags 0, so set
flags for every area you mark: a poly with no flags passes no filter that checks flags, the default one included.

Most queries take a `QueryFilter`: two functions and their data. `pass_filter` decides whether a query may use a node,
and `get_cost` what crossing it costs. The default, `pass_flags`, checks the node's flags against `include_flags` and
`exclude_flags`:

<Snippet source="./snippets/areas.cpp" select="flag_filters" />

A cost function makes some ground dearer than other: paths take it only where it saves enough. Its `user_data` is
yours:

<Snippet source="./snippets/areas.cpp" select="area_cost" />

<Snippet source="./snippets/areas.cpp" select="cost_filter" />

The default functions are recognised and inlined in the searches, so a filter that needs only flags is fastest with
`pass_flags` and `distance_cost`. `ANY_QUERY_FILTER` passes every node, flags or not.

<ExamplesTable ids="example_area_filters,example_area_costs,example_multiple_agent_sizes,example_doors_and_keys" />

### Off-Mesh Connections

Off-mesh connections join places the nav mesh's polys do not: a jump across a gap or down a ledge, a ladder, a
teleporter, a door.

**When to use:** When your environment has gaps, vertical transitions or traversal mechanics that polys can't
represent. Pathfinding considers them like any other node.

They are part of the nav mesh, not of a tile: each end attaches to the polys under it, and attaches again as tiles are
added and removed, so they can be added and removed at any time, work between any tiles, and survive rebuilding them.
Each has an area and flags, as polys do, and filters treat it as a node like any other.

#### Point connections

A point connection goes from one point to another, one way or both ways.

<Snippet source="./snippets/off_mesh.cpp" select="point_connection" />

A path that takes it has an off-mesh point where it leaves, then the point where it lands:

<Snippet source="./snippets/off_mesh.cpp" select="point_connection_path" />

<Snippet source="./snippets/off_mesh.cpp" select="remove_connection" />

<ExamplesTable ids="example_off_mesh_connections" />

#### Segment connections

A segment connection is wide: it joins a whole edge to another, such as the length of a ledge to the floor below it,
and a path crosses it wherever along it suits the path. `start_a` maps to `end_a` and `start_b` to `end_b`, and each
point between to the point the same fraction along the other. Agents coming from different places cross at different
points, rather than all at one.

<Snippet source="./snippets/off_mesh.cpp" select="segment_connection" />

roam splits a segment connection into pieces, one per pair of polys it joins, and maps each piece's crossing at the
same fraction along both ends. `align` swaps `end_a` and `end_b` when that uncrosses the mapping.

<ExamplesTable ids="example_segment_off_mesh_connections" />

## Advanced Navigation Mesh APIs

This section covers lower-level APIs for working with the nav mesh's structure. Most users won't need them for
everyday pathfinding, but they are useful for understanding the internals, building
[custom pathfinding algorithms](#custom-pathfinding-algorithms), or debugging.

### Nodes and refs

A `NodeRef` names a node: a poly or an off-mesh connection. It is a `uint64_t`, with `INVALID_NODE_REF` for none
(0 is a valid ref).

<Snippet source="./snippets/advanced.cpp" select="nodes_and_refs" />

Everything in `NavMesh` and `NavMeshTile` is there to read. Change a nav mesh only through `nav_mesh_api`.

### Tiles

`add_tile` takes a built tile, links it with its neighbours and the off-mesh connections that reach it, and returns
its id. `remove_tile` takes one out by grid position. A tile's polys get new refs each time it is added, so hold refs,
not pointers, and check old refs with `is_valid_node_ref`. Never keep a tile pointer across `add_tile` or
`remove_tile`.

<Snippet source="./snippets/advanced.cpp" select="tiles" />

Tiles at the same `tile_x` and `tile_y` with different `tile_layer`s stack in one column, for floors above each other.

### Polys

Smaller queries over polys: those in a box, the closest point on one, and the edge two share. `get_poly_height` gives
the detail mesh's height under a point, `get_closest_point_on_poly_boundary` the nearest point on a poly's edge.

<Snippet source="./snippets/advanced.cpp" select="polys" />

## Using Externally Created Navigation Meshes

A nav mesh does not have to come from triangles. You can bring your own polygons, authored by hand or made with another
tool: `polygons_to_nav_mesh_tile_polys` makes a tile's polys from convex polygons, and `polys_to_tile_detail_mesh` makes
a detail mesh for them, or you can provide your own if you have height data.

<Snippet source="./snippets/advanced.cpp" select="external_polygons" />

Polygons must share vertices where they meet: neighbours are found by shared edges.

<ExamplesTable ids="example_custom_gltf_nav_mesh" />

## Saving and Loading NavMeshes

A saved nav mesh is a snapshot: its tiles, nodes, links and off-mesh connections, little-endian, floats bit for bit.
Loaded, it gives the same refs and the same query results as the one saved, and loading checks every count and index,
so bad data is refused with a status rather than read.

<Snippet source="./snippets/advanced.cpp" select="save_load" />

`save_tile` and `load_tile` do the same for one tile, as in [Tiles](#tiles): a tile built offline, or kept to add
again later. A saved tile has no links; `add_tile` makes them.

## Geometry Blocks

Two small blocks for preparing generation input, in `roam/blocks/geometry/`.

`merge_positions_and_indices` joins meshes from several sources into one, merging vertices at the same position so
their triangles connect:

<Snippet source="./snippets/blocks.cpp" select="merge" />

`chunky_tri_mesh` buckets triangles in xz, to find the triangles under an area without visiting them all. It is what
the tiled generator builds each tile from:

<Snippet source="./snippets/blocks.cpp" select="chunky" />

<ExamplesTable ids="example_chunky_tri_mesh" />

## Debug Utilities

`roam/debug/debug.h` turns each stage of generation, and the nav mesh itself, into renderer-agnostic primitives:
triangles, lines, points or boxes, each with positions, colours and indices. They draw nothing themselves; write a few
lines to draw them with whatever you render with, as the demo app does with sokol.

<Snippet source="./snippets/debug.cpp" select="debug_helpers" />

<Snippet source="./snippets/debug.cpp" select="debug_primitives" />

## Demos

`demos/` is an app of 30 examples, listed [at the top](#roam). Each is one file in `demos/examples/`. The app builds
natively and with Emscripten, and every push to `main` deploys the web build to
[isaac-mason.github.io/roam](https://isaac-mason.github.io/roam/). Open an example directly with `?example=<id>`, such
as [`?example=example_find_path`](https://isaac-mason.github.io/roam/?example=example_find_path); the URL follows the
example you pick, so it can be copied and shared. Natively, pass the id or name: `demos/native.sh example_find_path`.

```sh
demos/native.sh                 # build and run natively
demos/native.sh "find path"     # open an example by name
demos/web.sh                    # build with Emscripten (needs EMSDK) and serve on localhost:8080 (PORT overrides)
demos/smoke.sh                  # run every example headlessly for 30 frames, failing on crashes and leaks
demos/screenshots.sh            # render every example offscreen into demos/screenshots/
```

## Tests and Benchmarks

```sh
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
```

The tests cover generation and queries, fuzz queries, tile churn and generation inputs, and check every test for
leaks.

This README is built from [`docs/README.template.md`](./docs/README.template.md) by `node docs/build.mjs`. Its code
comes from [`docs/snippets/`](./docs/snippets), which the tests compile and run (`roam_snippets` and
`roam_snippet_quickstart`), and its declarations are read from roam's headers, so none of it can drift from the code.
The `roam_readme` test fails when `README.md` is out of date. The example screenshots come from
`demos/screenshots.sh`, which renders every example offscreen (macOS only, for now).

`roam_bench` times generation and queries on a level fixture, best built with the `release` preset:
`build/release/roam_bench --fixture path/to/nav-test.json`.

## License

roam is [MIT licensed](./LICENSE).
