# roam plan

## Goal

A 1:1 C++ port of navcat that is idiomatic, portable and readable. navcat's improvements over recast come with it: the world-level node and link pools, and runtime off-mesh links between any two tiles. Performance work and large-world support come after parity.

## Decisions

| topic | decision |
|---|---|
| language | C++20, written C-style |
| scalar | `float`; tile vertices in world space |
| naming | snake_case functions, PascalCase types, one namespace per module |
| math | `struct Vec3 { float x, y, z; }` and friends; functions return by value |
| containers | roam's own `Array<T>` and `Map<K, V>` (integer keys); no STL |
| memory | allocator hook; running out of memory aborts |
| errors | no exceptions; result structs with flags, as in navcat |
| node refs | `uint64_t`, same bit layout as navcat (1 type, 31 index, 20 sequence); invalid is `~0ull` |
| query filter | struct of function pointers plus `void *user_data` |
| random | `float (*rand)(void *user_data)` plus user data |
| build | CMake with presets `debug`, `release`, `web`; targets `roam` (core) and `roam_blocks` |
| sorting | `sort::stable`, since JS `Array.prototype.sort` is stable and ordering affects output |

## Layout

```
roam/                      the library; includes read "roam/generate/heightfield.h"
  util/                    assert, memory, array, map, sort, index_pool
  math/                    scalar, vec2, vec3, box3
  geometry/                navcat src/geometry.ts
  generate/                navcat src/generate/*
  query/                   navcat src/query/*
  debug/                   navcat src/debug.ts
  blocks/                  navcat blocks/*, built as the separate roam_blocks target
    geometry/  generators/  search/  agents/
tests/                     mirrors roam/, files end in .test.cpp
  harness/                 test.h, fixture loader, main
  parity/                  tests against navcat fixtures
demos/
  app/                     sokol and imgui shell, camera, debug draw, model loading
  examples/                one file per navcat example
  assets/                  the .glb models the demos use
bench/
tools/navcat-fixtures/     the Node dumper
llm/                       planning docs
```

- Folders are for finding files, not namespaces: `roam::heightfield::rasterize_triangles` lives in `roam/generate/heightfield.h`.
- File names are navcat's names in snake_case. `three/` has no counterpart; the demo app draws `debug` primitives itself.

From JS to C++:

- `Record<string, X>` keyed by pool ids becomes an `Array<X>` of slots indexed by an `IndexPool` id, each with an `allocated` flag.
- The tile position maps become a `Map<uint64_t, …>` keyed by packed (x, y, layer).
- Result objects become result structs that own `Array`s.
- Per-item variable-length data is flat (decided 2026-10-20). navcat gives each poly its own `vertices`/`neis` arrays; roam stores them in tile-wide `poly_vertices` and `poly_neis` arrays, and each `NavMeshPoly` keeps `vertex_base` and `vertex_count` (`neis` share the same base). This is the same pattern navcat already uses for detail meshes, and tiles are immutable once built. Node links follow the same idea: links stay in the world-level pool, and each node keeps a chain (`first_link`, `last_link`, `link.next`) that appends at the tail, so links are visited in navcat's order. Recast's alternative, fixed inline `verts[6]`/`neis[6]`, was rejected because it caps vertices per poly, which navcat deliberately doesn't.
- `BuildContext` keeps only logs. roam never reads a clock: in wasm that means a call out to JS. Callers time stages themselves; the shape rasterizers take no context.

## Parity testing

1. `tools/navcat-fixtures/dump.mjs` imports navcat's built `dist`, runs the bench fixture (and later a few example models) through every generation stage plus seeded queries, and dumps each stage's output. It grows a stage at a time as stages are ported.
   - Fixtures are never committed. The `navcat_fixtures` CMake target writes them to `build/<preset>/fixtures/`, and `roam_parity_tests` depends on it.
   - The dumper refuses to run when navcat's `dist` is older than its sources; run `pnpm build` in navcat.
   - Parity tests only build when Node and a navcat checkout (`ROAM_NAVCAT_DIR`, default `../navcat`) are found.
   - File format: a u32 entry count, then per entry: u32 name length, name, u32 type (0 f64, 1 i32), u32 count, values. Little-endian.
2. C++ tests load the fixtures and compare stage by stage: structure exactly, values within a float tolerance. The first stage that diverges tells you which file has the bug.
3. Port navcat's `tst/*` suites.
4. Use a tiny test harness written for roam, with no third-party test framework.

## Known divergences from navcat

Dated decisions and the reasoning behind them are in `llm/worklog.md`.

- **BV tree bounds** are quantized to `uint16_t` recast-style, with near-integers snapped first. navcat needs `llm/navcat-bv-tree-quantization.patch` to match.
- **Ref 0 is never treated as "none"**: roam uses `INVALID_NODE_REF`. navcat needs `llm/navcat-ref-zero-truthiness.patch` to match.
- **Smooth path zero-length steps** move zero distance instead of producing NaN. navcat needs `llm/navcat-smooth-path-zero-step.patch` to match.
- **Merged positions keep index 0**: navcat needs `llm/navcat-merge-positions-index-zero.patch` to match.
- **Floating point contraction is off** for roam's targets, so results don't depend on whether the compiler fuses multiply-adds.
- **Path queries** differ from navcat only at exact vertex and edge ties (see worklog).

- **Detail mesh error ties.** Detail heights are whole multiples of `cell_height`, and `sample_max_error` is usually a multiple of it too. So the two "error greater than max" checks in `poly_mesh_detail` (edge simplification and sample insertion) often compare two mathematically equal values. roam rounds these ties in `float`, as recast does (same comparisons, no epsilon). navcat rounds them in JS doubles. On nav-test this adds one extra vertex to poly 39. The same happens in `triangulate_hull`, which starts from the ear with the shortest perimeter: a lone triangle has three equal perimeters, and symmetric polys have congruent ears. So float and double sometimes start from different ears, which rotates or reshapes the triangulation (nav-test poly 131). Decision (2026-10-20): keep the recast-aligned float behaviour. The detail parity test compares mesh by mesh: vertices must match within 1e-4, triangles must match up to rotation (flags rotate too), and a small budget of tied meshes (at most 2%) may differ by one vertex or by triangulation.

## Demos

- One app with an example picker. Each example is a file `demos/examples/example_<name>.cpp` that exposes an `Example` of `{ name, init, update, ui, shutdown }`.
- 29 of navcat's 32 examples are ported. The three physics examples (dynamic navmesh, dynamic obstacles, FPS dynamic navmesh) need Rapier and are skipped for now.
- Stack: sokol (app, gfx, glue, time), Dear ImGui through sokol_imgui, and cgltf for navcat's `.glb` models, all fetched and pinned by CMake.
- Builds natively and with emscripten into a static site.

## Phases

0. Scaffolding (done): CMake and presets, `Array`, `Map`, `sort`, `index_pool`, allocator, assert, math, the test harness, the fixture dumper.
1. Foundations: `node`, `geometry`.
2. Generation, each stage golden-tested: input triangle mesh, heightfield, compact heightfield, regions, contour set, poly mesh, poly mesh detail, poly neighbours, nav mesh tile.
3. Nav mesh core: structures, BV tree, `nav_mesh_api` (tiles, node and link pools, off-mesh connections).
4. Demo app skeleton: a solo navmesh with debug draw.
5. Queries: search, `find_node_path`, sliced search, `find_path`, `find_straight_path`, `find_smooth_path`, `raycast`, `move_along_surface`, random points, local neighbourhood.
6. Blocks: chunky tri mesh, solo and tiled generators, flood fill, merge positions.
7. Agents: path corridor, local boundary, obstacle avoidance, crowd.
8. The remaining demos, and benchmarks against navcat on the same fixtures.

## After parity

- navcat patch backlog (banked for later, not sent): `bv-tree-quantization`, `ref-zero-truthiness`, `smooth-path-zero-step`, `merge-positions-index-zero` and `contour-walk-cycle` are written in `llm/`; the poly merge value cache (see `benchmarks.md`, round 4) still needs writing as a navcat patch.

- Performance: a reusable, generation-stamped search pool.
- Large worlds: tile-local vertex storage.
- A handoff from roam to navcat: roam writes a tile blob that navcat can import.
