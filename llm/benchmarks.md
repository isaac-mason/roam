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
