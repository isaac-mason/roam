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
