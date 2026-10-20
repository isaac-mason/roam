# roam worklog

Dated decisions and notable findings. The current design lives in `plan.md`; this file records how it got there.

## 2026-10-20

- **No clock in the library.** `BuildContext` keeps logs only. Reading time in wasm means a call out to JS, so callers time stages themselves.
- **Detail mesh ties stay recast-aligned.** Detail heights are multiples of `cell_height`, so the "error greater than max error" checks often compare equal values, and `triangulate_hull` often compares equal ear perimeters. roam resolves these in `float` like recast, navcat in JS doubles. The detail parity test allows a small budget of tied meshes.
- **Flat tile-wide poly arrays.** `NavMeshPoly` keeps `vertex_base`/`vertex_count` into tile-wide `poly_vertices`/`poly_neis`, instead of navcat's per-poly arrays or recast's fixed `verts[6]`. Node links are a chain through the world link pool (`first_link`, `last_link`, `next`) that appends at the tail, keeping navcat's link order.
- **Ref 0 is a valid ref; navcat sometimes treats it as "none".** Poly node 0 with sequence 0 serializes to `0`, which is falsy in JS. Affected places in navcat:
  - `findNearestPoly` failure leaves `nodeRef = 0`.
  - `findRandomPoint` and `findRandomPointAroundCircle` treat a selected ref of 0 as no selection. On nav-test this makes navcat fail a random-point call, which also skips two random draws, so every later call reads a shifted random sequence.
  - `findLocalNeighbourhood` and `getPolyWallSegments` skip links and portal intervals whose ref is 0.
  - `raycast` and any-angle search pass `0` as "no previous ref".
  roam uses `INVALID_NODE_REF` everywhere. The navcat fix is `llm/navcat-ref-zero-truthiness.patch`, covering `nav-mesh-api.ts`, `nav-mesh-search.ts` and `local-neighbourhood.ts`.
- **BV tree bounds are quantized recast-style, with snapping.**
  - Problem: navcat stored leaf bounds as unrounded `(v - tile_min) * quant_factor`. Grid-aligned polys produce near-integers, and the query snaps its box to integers (`& 0xfffe`, `| 1`), so membership near tile-grid boundaries depended on float noise. On nav-test, roam (float) and navcat (double) disagreed on 26 of 200 polygon queries and 3 of 200 nearest-poly results. An internal node also had equal x/z extents, so float and double split on different axes.
  - Recast (checked `createBVTree`): truncates min and max to `unsigned short`, in float. Correct, since the query's even/odd snapping covers the truncation, but it has the same float-versus-double split at integers.
  - Decision: recast's integer layout (`uint16_t bmin[3]`, `bmax[3]`, truncation, clamp to `0..0xffff`), plus snapping. Any quantized value within `1e-3` of an integer is snapped to it before truncating. Bounds are integers, the longest-axis choice is exact, and grid-aligned values agree in float and double.
  - navcat needs the same change for parity: `llm/navcat-bv-tree-quantization.patch` (applies to `src/query/bv-tree.ts`). With it applied to a scratch copy of navcat, all 16 parity tests pass exactly: BV tree, 200 nearest-poly queries, 200 polygon queries and off-mesh connection links. Until navcat has the patch, `nav_mesh_matches_navcat_on_nav_test` fails.
- **Floating point contraction is off** (`-ffp-contract=off` on roam's targets). clang on arm64 fuses `a * b - c * d` into FMA by default, so `tri_area_2d(a, b, b)` came out as a tiny nonzero instead of exactly 0. That flipped funnel decisions at the end of straight paths: 44 of 180 straight, crossing and smooth paths attached a different ref to their end point than navcat. Contraction also differs between compilers and platforms, so turning it off is a determinism fix on its own. MSVC's default `/fp:precise` doesn't contract.
- **Query parity on nav-test** (60 path pairs, with both navcat patches applied in a scratch copy):
  - Exact: `find_path` results and node paths, raycast, `move_along_surface`, local neighbourhood, wall segments, random points.
  - Straight path 2/60, crossings 2/60, any-angle and sliced 2/60: precision ties at exact mesh vertices and edges. Examples: a crossing point that lands exactly on the end vertex in only one precision, and an any-angle raycast that ends exactly on a portal edge midpoint.
  - Smooth path 14/60: the same vertex ties, compounded over many 0.5-unit steps, plus the navcat zero-step bug (see 2026-10-21).
  - Test inputs only use points that are directly over a poly. Nearest-poly positions projected onto poly boundaries put rays and moves exactly on edges, which gave spurious tie mismatches.

## 2026-10-21

- **navcat bug, fixed in both: `findSmoothPath` with a zero-length step.** If the steer target equals the current position (for example, standing exactly on an off-mesh connection start), `Math.min(stepSize, 0) / 0` is NaN. `moveAlongSurface` rejects the NaN target and the smooth path stops after its first point. roam steps zero distance when the length is 0, so the off-mesh connection is taken; `find_smooth_path_takes_off_mesh_connection_from_its_start` covers it. The navcat fix is `llm/navcat-smooth-path-zero-step.patch`.
