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
- [ ] **Move along surface** (0.75-0.79x): profiled. Faster `point_in_poly` and a z-range skip landed. Remaining: a small pool for tiny searches (Detour uses a 64-node pool), and the cost of navcat's exact point-in-poly tie rules.
- [ ] **Parallel tiled builds.** Tiles are independent. Expose per-tile generation as a thread-safe public function, and show a threaded build in the bench and a demo. roam itself stays single-threaded.
- [x] **Regions: a closer look.** Now 1.03-1.21x ahead of Detour: 16-bit scratch, sorting only unassigned spans, and expansion drops resolved entries.
- [x] **Mark walkable: a closer look.** Bound by indexed vertex loads; vectorizing the arithmetic didn't change the time.
- [ ] Deferred: SIMD (obstacle avoidance, detail sampling, triangle setup), a generation workspace, Detour's 6-vertex poly cap.

## Structure and API review (2026-10-21)

Generation:

- [ ] **One way to pass a triangle mesh.** `mark_walkable_triangles`, `rasterize_triangles`, `calculate_mesh_bounds` take `(vertices, indices, index_count)`; `rasterize_convex` takes `(vertices, vertex_float_count, triangles, triangle_index_count)`; the blocks have three identical structs (`SoloNavMeshInput`, `TiledNavMeshInput`, `MeshView`). Use spans (`Span<const float> positions, Span<const uint32_t> indices`) or one shared view struct.
- [ ] **Output buffers as spans.** `mark_walkable_triangles(..., uint8_t *out_tri_area_ids, ...)` writes through a raw pointer with the output last. Make it a `Span<uint8_t>` like the query outputs.
- [x] **The mutated object first.** Done: erode, the mark area functions and `compact_heightfield::build(context, heightfield, ...)`; a `BuildContext`, when taken, stays first. Was: `erode_walkable_area(radius, chf)`, `mark_box_area(bounds, area, chf)`, `median_filter_walkable_area(chf)` and friends put the compact heightfield last (navcat's order), while `filter_ledge_spans(hf, ...)` and `build_distance_field(chf)` put it first.
- [ ] **Counts that duplicate array lengths.** `CompactHeightfield::span_count`, `Contour::vertex_count` / `raw_vertex_count`, `PolyMesh::vertex_count` / `poly_count`, `PolyMeshDetail::mesh_count` / `vertex_count` / `triangle_count` sit next to arrays that already know their length. Two sources of truth (the nav mesh structs dropped theirs).
- [ ] **Borrowed arrays that look owned.** `NavMeshTileParams` holds `Array<T>` fields that it doesn't own (`build_tile` copies them). Make them `Span<const T>`. Also add one helper that builds tile params from tile polys and detail mesh; the solo and tiled generators and the bench each write the 15-field copy by hand.
- [ ] **Internal constants in `roam::`.** `common.h` exports `AXIS_X`, `BORDER_VERTEX`, `AREA_BORDER`, `MULTIPLE_REGS`, `CONTOUR_REG_MASK`, `MAX_LAYERS` and the direction helpers next to the public `NULL_AREA`, `WALKABLE_AREA`, `MESH_NULL_IDX`.
- [ ] **Later phase: typed records** for the flat int arrays (`PolyMesh::vertices` stride 3, `polys` stride 2 x nvp, detail `meshes` and `triangles` stride 4). Bigger change; parity tests compare these arrays.
- [ ] Small: `flag_merge_threshold` defaults to 1 on `rasterize_triangles` but is required on the shape rasterizers.

Query:

- [x] **`release_nav_mesh`** → `release(NavMesh &)`. Was the only `release_<thing>` left; everything else is a `release(T &)` overload.
- [x] **Result naming.** Now `GetClosestPointOnPolyBoundaryResult`, `GetClosestPointOnDetailEdgesResult` (out-first), `GetPortalPointsResult`, `GetEdgeMidPointResult`, all with `position`. Was: `ClosestPointOnPolyBoundary` and `ClosestPointOnDetailEdges` lack the `Result` suffix that `GetClosestPointOnPolyResult` has, and use `point` where the others use `position`. `get_closest_point_on_detail_edges` returns by value while its siblings write `out`.
- [ ] **Later: `success` and a `SUCCESS` flag** both live in `FindNodePathResult`, `FindPathResult`, `FindStraightPathResult`, `FindSmoothPathResult`. Two sources of truth; navcat has both, so decide deliberately.
- [ ] Small: `StraightPathPoint::type` repeats the type bits in `node_ref` (40 → 32 bytes without it; navcat has the field).
- [ ] **Later, design: blocks scratch.** Blocks (crowd, path corridor) use `SearchWorkspace` arrays (`node_path`, `visited`, `raycast_path`) as their own scratch, coupling them to the workspace's internals. They may want their own scratch.
