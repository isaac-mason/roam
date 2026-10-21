# Benchmarks

`roam_bench` (release, `-O3`) against navcat's `dist` under Node, on the same workloads over navcat's `bench/fixtures/nav-test.json`. Each bench warms up 3 times, then samples until at least 10 samples and 500 ms, and reports the median. Generation stages time only their stage: earlier stages are rebuilt untimed before each sample, as navcat's labs arg factories do.

Run: `cmake --preset release && cmake --build --preset release`, then `node tools/navcat-bench/bench.mjs --navcat ../navcat --roam build/release/roam_bench`. `roam_bench --fixture <path> [filter]` runs roam alone.

## 2026-10-21, Apple M1 Pro, Node v24.10.0

| bench | navcat median ms | roam median ms | speedup |
|---|---:|---:|---:|
| mark walkable triangles | 0.0303 | 0.0082 | 3.7x |
| rasterize triangles | 3.6997 | 1.2538 | 3.0x |
| filter heightfield | 0.9484 | 0.3284 | 2.9x |
| build compact heightfield | 0.6945 | 0.1361 | 5.1x |
| erode walkable area | 0.6981 | 0.2055 | 3.4x |
| build distance field | 0.9578 | 0.2365 | 4.0x |
| build regions | 1.3518 | 0.4145 | 3.3x |
| build contours | 0.7050 | 0.1818 | 3.9x |
| build poly mesh | 0.8348 | 0.2288 | 3.6x |
| build poly mesh detail | 1.1540 | 0.2650 | 4.4x |
| generate solo nav mesh | 11.1159 | 3.8228 | 2.9x |
| generate tiled nav mesh | 17.3537 | 6.1217 | 2.8x |
| find path (100 pairs) | 1.4514 | 0.6538 | 2.2x |
| raycast (100 rays) | 0.0325 | 0.0170 | 1.9x |
| find nearest poly (200 points) | 0.5290 | 0.2892 | 1.8x |
| crowd (100 agents, 60 updates) | 97.7336 | 40.9435 | 2.4x |

## Search workspace, before and after

| bench | before ms | after ms |
|---|---:|---:|
| find path (100 pairs) | 0.7541 | 0.6152 |
| find path (100 pairs, reused workspace) | - | 0.6305 |
| crowd (100 agents, 60 updates) | 38.6392 | 38.9412 |

The chained-hash pool alone makes `find_path` about 18% faster, even with a temporary workspace per call; reusing one saves little on top. The crowd doesn't move, so its time is elsewhere (obstacle avoidance sampling is the likely cost). Timings vary by a few percent between runs.

## Perf round 1: crowd (2026-10-21)

Profiled `roam_bench crowd` with Instruments' Time Profiler (RelWithDebInfo, `-O3 -g -fno-omit-frame-pointer`). Before the round the update split roughly into obstacle avoidance sampling (about 45%), `update_corners` with its straight path (20%), local boundary refreshes (11-15%) and corridor moves (8-15%), with about 10% of all time in malloc and free.

Changes, all bit-identical (unit and parity tests unchanged, including the 240-frame flat crowd):
- Obstacle avoidance: per-obstacle values that don't depend on the candidate velocity (the circle sweep offset and `|s|^2 - r^2`, each segment's direction, agent offset and `perp(v, w)`) are computed once in `prepare_obstacles` instead of once per sample.
- `get_poly_height`, `get_closest_point_on_poly` and `get_closest_point_on_poly_boundary` gather poly vertices into a 32-vertex stack buffer, falling back to the heap only for larger polys.
- `merge_start_moved` and `merge_start_shortcut` merge corridors in place instead of building a new array.
- `_into` variants refill a caller-owned result and reuse its capacity: `find_straight_path_into`, `move_along_surface_into`, `raycast_into`, `get_poly_wall_segments_into` and `find_corners_into`. The `SearchWorkspace` keeps a scratch move and raycast result; the corridor uses them when given a workspace, the local boundary reuses one wall segment result across its polys, and the crowd swaps corners with a scratch array.

Results (alternating runs of the before and after binaries, `--time 5000`):

| bench | before median / min ms | after median / min ms |
|---|---:|---:|
| crowd (100 agents, 60 updates), quiet machine, after the first three changes | 36.64 / 35.60 | 33.67 / 32.63 |
| crowd, busy machine, all changes | 43.35 / 38.32 | 39.85 / 34.15 |
| find path (100 pairs) | 0.898 / 0.629 | 0.708 / 0.565 |
| find nearest poly (200 points) | 0.309 / 0.278 | 0.218 / 0.187 |

About 8-10% on the crowd and 30% on nearest poly. The crowd's remaining time is mostly inherent work: obstacle avoidance sampling is about 75 candidate velocities per agent per update against every neighbour. navcat collects every neighbour within the collision query range, where recast caps it at the 6 nearest; a cap would be the next big win but changes behaviour against navcat.

Correction: round 1's profiling build set `CMAKE_CXX_FLAGS_RELWITHDEBINFO` without `-DNDEBUG`, so `ROAM_ASSERT` bounds checks were compiled in for both the before and after binaries. The comparisons are like for like, but absolute times include assert overhead. Profile with `-O3 -g -fno-omit-frame-pointer -DNDEBUG`.

## Perf round 2: generation (2026-10-21)

Profiled `generate solo nav mesh` and `generate tiled nav mesh` (asserts off). Time is spread over the stages with no single dominant cost: `filter_ledge_spans` (14-16%, pointer-chasing span columns), detail height patches (5%), compact heightfield connections, region expansion, rasterization clipping (`divide_poly`), distance fields. Each is the same algorithm as navcat, already 2.9-5.1x faster than it.

Change: `sort::stable_with_scratch` takes a caller scratch buffer; the chunky tri mesh's recursive subdivision allocates one scratch buffer instead of one per sort. Identical output; no measurable change on nav-test (5.48 vs 5.50 ms tiled), since the sort was under 1%.

Findings for later:
- Tiled generation spends about 15% in malloc, free and memset, mostly allocating and freeing each tile's heightfield, compact heightfield, contours and meshes. Reusing those buffers across tiles is the next single-threaded win, with an estimated 5-10%.
- Tiles are independent, so the big win for tiled builds is building them in parallel. That needs a public per-tile build function that callers can run on their own threads (roam doesn't own threads, which also matters for wasm).

## Perf round 3: rasterization (2026-10-21)

With asserts off, the stage benches put rasterization at 1.2 of the 3.36 ms solo build (36%); in the whole-build profile it had been spread over inlined `divide_poly`, `push_vertex` and `add_heightfield_span` frames. Inside it, clipping (`divide_poly` and `push_vertex`) is about 38% and span insertion about 13%.

Change: in the last row of a triangle and the last cell of a row, the clipped-off remainder is discarded, so when every vertex is already on the near side of the boundary the clip only copies the polygon. `all_on_near_side` checks that with the clip's own expression (`(cell + cell_size) - v >= 0`) and the rasterizer swaps buffers instead of clipping. Bit-identical (heightfield parity unchanged). Triangles inside one cell skip both clips; larger ones skip one per row.

| bench (min ms, alternating runs) | before | after |
|---|---:|---:|
| rasterize triangles | 1.187 | 1.066 |
| generate solo nav mesh | 3.287 | 3.195 |

## Perf round 4: large levels (2026-10-21)

nav-test is a 1,612-triangle, 3 ms build, too small to show scaling problems. `tools/navcat-bench/glb_to_fixture.py` flattens a `.glb` into navcat's bench fixture format (scene node transforms, positions merged like `getPositionsAndIndices`; it reproduces navcat's nav-test fixture to within its JSON rounding). Fixtures go in `build/bench-fixtures/`, never committed. Both `roam_bench --fixture` and `bench.mjs --fixture` take them.

```sh
python3 -I tools/navcat-bench/glb_to_fixture.py demos/assets/game-level.glb build/bench-fixtures/game-level.json
node tools/navcat-bench/bench.mjs --navcat ../navcat --roam build/release/roam_bench --fixture build/bench-fixtures/game-level.json
```

Starting point (median ms): roam stays 3-5x faster than navcat, but two stages blow up at scale, in navcat too.

| level | triangles | navcat solo | roam solo | roam contours | roam poly mesh |
|---|---:|---:|---:|---:|---:|
| game-level | 71,399 | 180.5 | 52.3 | 17.2 | 2.6 |
| lowpoly FPS map | 58,758 | 303.1 | 66.8 | 2.4 | 22.7 |

**Contours: walks stuck in a cycle.** On game-level, 124 of 512 contour walks ran to the 40,000-iteration cap (5M steps, all in region 14, which also logs "Multiple outlines": the watershed overlap navcat's comments warn about in narrow spiral corridors). The walk emits its points, then circles cells whose flags are already cleared and never returns to its start. Flags only change on emission, so a repeated (span, direction) state with no emission in between is a cycle that can never emit or terminate. `walk_contour` detects it with Brent's algorithm (a checkpoint reset on every emission, doubling interval) and stops there, producing the same points as running to the cap. game-level contours: 17.2 to 1.8 ms. The same change for navcat is `llm/navcat-contour-walk-cycle.patch` (navcat game-level contours: 63 to 7 ms).

**Poly mesh: cubic merging.** `find_best_merge` re-evaluated every polygon pair after every merge: O(p^3) per contour, and one FPS map contour triangulates into 159 triangles (merging took 27.4 of the build's poly mesh time). A merge value depends only on its two polygons, and a merge only changes slot `pa` (merged) and `pb` (receives the last polygon), so `merge_polys_with_cache` keeps every pair's value and recomputes only pairs touching those two slots, scanning in the original order with the original strict `>` so it picks the same merge. Contours over 512 triangles fall back to the old search, since the cache is quadratic. FPS map poly mesh: 22.7 to 4.2 ms.

Both are verified bit-identical beyond the parity tests: a hash of every contour, poly mesh and detail mesh on game-level, the FPS map and nav-test matches the previous build exactly.

| level | solo before | solo after | tiled |
|---|---:|---:|---:|
| game-level | 52.3 | 34.0 | 91.3 |
| lowpoly FPS map | 66.8 | 46.7 | 93.1 |
| nav-test | 3.36 | 3.40 | 5.37 |

Tiled builds are now about 2-2.7x the solo build on the big levels and are the next thing to look at.

**Tiled: chunky tri mesh sort.** On game-level, building the chunky tri mesh was 23% of the tiled build: every subdivision level stable-sorted its whole range of 24-byte items with merge sort (O(n log^2 n) overall, with a lot of copying). The comparator is a float `<` on one coordinate, so `sort_by_min` uses a stable 4-pass LSD radix sort on order-preserving integer keys (with `-0` mapped to `+0`, since `<` treats them as equal) over compact (key, index) pairs, then permutes the items once. Verified identical by hashing chunky meshes (64 and 256 triangles per chunk) on all three levels.

| level | tiled before | tiled after |
|---|---:|---:|
| game-level | 91.3 | 68.0 |
| lowpoly FPS map | 93.1 | 81.1 |
| nav-test | 5.37 | 5.31 |

What's left in tiled builds is mostly rasterization (about 45% on game-level): large triangles are clipped once per tile they overlap, plus the tile border. That's inherent to tiling, and per-tile work is what callers can spread across threads.

## Against recast/detour (2026-10-21)

`tools/recast-bench` is a standalone CMake project that compiles recastnavigation's Recast, Detour and DetourCrowd sources (plus RecastDemo's chunky tri mesh) from `RECAST_DIR` (default `../recastnavigation`) into `recast_bench`. roam never links it. It runs the same workloads, names, settings (`-O3 -DNDEBUG`, same voxel and region parameters, detail sampling, 32-voxel tiles with a `walkableRadius + 3` border like `Sample_TileMesh`) and sampling rules as `roam_bench`. Queries use roam's query points (`roam_bench --dump-points`), since recast's poly order differs. Walkable polys get flag 1 so Detour's default filter passes them.

```sh
cmake -S tools/recast-bench -B build/recast-bench -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/recast-bench
node tools/navcat-bench/bench.mjs --navcat ../navcat --roam build/release/roam_bench --recast build/recast-bench/recast_bench [--fixture build/bench-fixtures/<level>.json]
```

The meshes are close but not identical, since navcat (and roam) carry fixes recast doesn't: solo poly counts are 132 vs 134 (nav-test), 1,058 vs 1,102 (game-level) and 1,414 vs 1,416 (FPS map).

Median ms, M1 Pro:

| bench | nav-test recast | nav-test roam | game-level recast | game-level roam | FPS map recast | FPS map roam |
|---|---:|---:|---:|---:|---:|---:|
| rasterize triangles | 1.44 | 1.09 | 20.13 | 14.99 | 24.32 | 18.96 |
| build contours | 0.17 | 0.18 | 8.48 | 1.76 | 1.69 | 1.98 |
| build poly mesh | 0.21 | 0.16 | 2.17 | 1.54 | 22.90 | 3.87 |
| generate solo nav mesh | 3.60 | 3.39 | 46.23 | 34.04 | 71.27 | 46.45 |
| generate tiled nav mesh | 5.94 | 5.82 | 89.97 | 68.10 | 92.16 | 79.67 |
| find path (100 pairs) | 0.45 | 0.67 | 1.65 | 2.05 | 0.61 | 0.86 |
| raycast (100 rays) | 0.0077 | 0.0175 | 0.0080 | 0.0170 | 0.0091 | 0.0176 |
| find nearest poly (200 points) | 0.15 | 0.21 | 0.12 | 0.16 | 0.08 | 0.10 |
| crowd (100 agents, 60 updates) | 31.2 | 41.4 | 27.1 | 26.3 | 25.3 | 24.2 |

- Generation: on par with recast on nav-test (1.02-1.06x), 1.3-1.5x faster on the big levels. Rasterization is about 1.3x faster everywhere; the rest of the gap is recast having the same stuck contour walks (game-level contours 8.5 vs 1.8 ms) and cubic poly merging (FPS map 22.9 vs 3.9 ms) that roam fixed. Distance field and regions are 5-30% slower than recast.
- Queries are where recast wins: find path 0.68-0.81x, nearest poly 0.72-0.81x, raycast about 0.45-0.5x (twice as slow). Detour writes into caller buffers and uses a fixed node pool; roam allocates result arrays and walks nodes through `get_node_by_ref`/links. These are the next targets.
- Crowd: on par on the big levels (1.03-1.05x), slower on nav-test (0.75x). Detour caps neighbours at 6 by default; roam's default (navcat's) is unlimited.

### Queries after the caller-buffer API (2026-10-21)

Same machine and rig, median ms. Queries now write into caller buffers through a reused `SearchWorkspace`, so they don't allocate.

| bench | nav-test recast | nav-test roam | game-level recast | game-level roam | FPS map recast | FPS map roam |
|---|---:|---:|---:|---:|---:|---:|
| find path (100 pairs) | 0.478 | 0.573 | 1.719 | 1.863 | 0.617 | 0.865 |
| raycast (100 rays) | 0.0078 | 0.0069 | 0.0080 | 0.0077 | 0.0092 | 0.0090 |
| find nearest poly (200 points) | 0.162 | 0.181 | 0.120 | 0.163 | 0.085 | 0.097 |

Raycast's whole gap was allocation. Find path and nearest poly still trail by 8-30%, and that is compute inside the query: the next round should profile A* (node lookups, `get_tile_and_poly_by_ref` per expansion) and nearest poly's detail-edge distance.
