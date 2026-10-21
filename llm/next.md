# Next: performance and API

A working checklist. Tick items as they land, and record findings and numbers in `worklog.md` and `benchmarks.md`.

## API cleanup

- [x] **One way to hand an `Array` to a query.** A writable `Span` from an `Array` now covers its capacity and a read-only one its length, so `reserve` and pass the array. Before, a reserved but empty array silently became a zero-capacity span, and 16 call sites spelled out `{data, capacity}`.
- [x] **Grow-and-retry without running the query twice.** Checked: workspace buffers start at 256 entries and a query only reruns when one grows, so the cost amortizes to nothing. Each site is now four plain lines, so no helper.
- [x] **Old-style signatures.** `get_portal_points` and `get_edge_mid_point` write `PortalPoints` and `EdgeMidPoint` out-first. `get_tile_and_poly_by_ref` takes the nav mesh first. Sliced init and update take the query first, and init no longer returns the status it already stores in the query.
- [x] **Search node count without reaching into the workspace:** `nav_mesh_search::get_search_node_count`.
- [ ] **Maybe: a `links_of` helper** for walking a node's links (`first_link` / `next`).

## Measure what isn't measured

- [x] **Crowd phase timing.** Obstacle avoidance is 63-73% of the update, corners 10-24% (see `benchmarks.md`). The crowd is the biggest number in the bench (25-32 ms for 100 agents x 60 updates, about 5 us per agent update). Time each phase: neighbours, local boundary, corridor and topology, corners, steering, obstacle avoidance, integration, collisions, and moving along the surface.
- [x] **Tile add and remove**, in roam_bench and recast_bench: 0.56-0.64x of Detour. Rebuilding tiles at runtime is a main reason to use tiled meshes, and linking now walks each node's list to append.
- [x] **Standalone queries** (move along surface 0.6x, wall segments 0.7x, query polygons and local neighbourhood about 0.93x, straight paths 1.2x): `move_along_surface`, `query_polygons`, `find_local_neighbourhood`, `find_straight_path`, `find_smooth_path`, `find_random_point_around_circle`.

## Performance

- [ ] **The crowd's hottest phases.** Obstacle avoidance: three exact scalar rewrites measured and dropped; the remaining lever is SIMD over obstacles (deferred). Next: corners (`find_corners`).
- [x] **Tile add and remove**: 0.6x → 1.10-1.37x of Detour by indexing border edges per tile.
- [x] **Wall segments** (0.7x → 0.87-1.0x), **local neighbourhood** (→ 1.01-1.07x), **query polygons** (→ 0.94-1.02x): inline node lookups and a leaner wall segment loop.
- [ ] **Move along surface** (0.73x with equal work): about 900 more instructions per move than Detour; the link walk and height snap are ruled out. Needs a profile.
- [ ] **Parallel tiled builds.** Tiles are independent. Expose per-tile generation as a thread-safe public function, and show a threaded build in the bench and a demo. roam itself stays single-threaded.
- [ ] **Regions (0.97x of Detour): a closer look.** Beyond the level stack allocation, profile flood, expand and merge at the instruction level, and compare the data layouts and loop structure with Detour.
- [ ] **Mark walkable (0.95x of Detour): a closer look.** Check the generated code against Detour's loop: bounds checks, index widths, aliasing, and whether the area write blocks vectorization.
- [ ] Deferred: SIMD (obstacle avoidance, detail sampling, triangle setup), a generation workspace, Detour's 6-vertex poly cap.
