# roam worklog

Dated decisions and notable findings. The current design lives in `plan.md`; this file records how it got there.

## 2026-10-20

- **No clock in the library.** `BuildContext` keeps logs only. Reading time in wasm means a call out to JS, so callers time stages themselves.
- **Detail mesh ties stay recast-aligned.** Detail heights are multiples of `cell_height`, so the "error greater than max error" checks often compare equal values, and `triangulate_hull` often compares equal ear perimeters. roam resolves these in `float` like recast, navcat in JS doubles. The detail parity test allows a small budget of tied meshes.
- **Flat tile-wide poly arrays.** `NavMeshPoly` keeps `vertex_base`/`vertex_count` into tile-wide `poly_vertices`/`poly_neis`, instead of navcat's per-poly arrays or recast's fixed `verts[6]`. Node links are a chain through the world link pool (`first_link`, `last_link`, `next`) that appends at the tail, keeping navcat's link order.
- **`find_nearest_poly` failure ref.** navcat leaves `nodeRef = 0` on failure, which is a valid ref (poly node 0, sequence 0). roam returns `INVALID_NODE_REF`. Worth fixing in navcat too.
- **BV tree bounds are quantized recast-style, with snapping.**
  - Problem: navcat stored leaf bounds as unrounded `(v - tile_min) * quant_factor`. Grid-aligned polys produce near-integers, and the query snaps its box to integers (`& 0xfffe`, `| 1`), so membership near tile-grid boundaries depended on float noise. On nav-test, roam (float) and navcat (double) disagreed on 26 of 200 polygon queries and 3 of 200 nearest-poly results. An internal node also had equal x/z extents, so float and double split on different axes.
  - Recast (checked `createBVTree`): truncates min and max to `unsigned short`, in float. Correct, since the query's even/odd snapping covers the truncation, but it has the same float-versus-double split at integers.
  - Decision: recast's integer layout (`uint16_t bmin[3]`, `bmax[3]`, truncation, clamp to `0..0xffff`), plus snapping. Any quantized value within `1e-3` of an integer is snapped to it before truncating. Bounds are integers, the longest-axis choice is exact, and grid-aligned values agree in float and double.
  - navcat needs the same change for parity: `llm/navcat-bv-tree-quantization.patch` (applies to `src/query/bv-tree.ts`). With it applied to a scratch copy of navcat, all 16 parity tests pass exactly: BV tree, 200 nearest-poly queries, 200 polygon queries and off-mesh connection links. Until navcat has the patch, `nav_mesh_matches_navcat_on_nav_test` fails.
