# Saving and loading nav meshes

Design, 2026-10-21. Built in `roam/query/nav_mesh_io.{h,cpp}`, tested in `tests/query/nav_mesh_io.test.cpp`.

## Goals

- Bake a nav mesh offline and load it at runtime, or send it over a network.
- Save and load single tiles, for streaming and for caching rebuilt tiles.
- A saved nav mesh loads back as the same nav mesh, as with navcat's `JSON.stringify` / `JSON.parse` round trip: every ref that was valid is valid again, queries give the same results (link order and tile column order decide exact-cost ties, so they are kept), and the refs and ids handed out next are the same ones the original would have handed out.
- Malformed or hostile bytes fail with an error. They never crash, assert or read out of bounds.
- No file I/O in roam: everything is byte buffers.

Not goals: a format navcat can read, or Detour binary compatibility. The format is roam's own, shaped like Detour's.

## How Detour does it

- A tile is one blob from `dtCreateNavMeshData`: a header (magic, version, tile x, y and layer, counts, walkable settings, bounds, BV quantization factor), then sections for vertices, polys, links, detail meshes, detail vertices, detail triangles, the BV tree and off-mesh connections, each 4-byte aligned. Links are reserved space, filled in by `addTile`.
- The sample's mesh file (`MSET`) is a header (magic, version, tile count, `dtNavMeshParams`) followed by each tile's ref, data size and blob. Loading calls `addTile(data, size, flags, tileRef)`, and passing the old tile ref keeps poly refs the same.

roam follows the same split for tiles: a self-contained tile blob that `add_tile` links up. The nav mesh file goes further than Detour's and is a full snapshot (nodes, links, tiles, off-mesh connections, column order, sequence counters, index pools), so a reload is indistinguishable from the original.

## Encoding rules

- Little-endian, written and read field by field with explicit widths. Structs are never `memcpy`'d, so padding, alignment and host ABI don't matter, and the encoder is the same on every platform.
- Floats are written as their IEEE 754 bits, so a round trip is bit-exact.
- Every blob starts with a 4-byte magic and a `uint32` version. Readers reject unknown versions instead of guessing.
- No padding depends on where a blob sits: every tile section is a whole number of 4-byte words from the start of the tile (polys are 12 bytes with two zero bytes after the area), so a tile embedded in a nav mesh file encodes exactly like a standalone one. Nothing is read in place.

## Tile blob

Header:

| field | type |
|---|---|
| magic `RMTL` | 4 bytes |
| version | u32 |
| tile x, y, layer | i32 x 3 |
| vertex count, poly count, poly vertex count | u32 x 3 |
| detail mesh count, detail vertex count, detail triangle count | u32 x 3 |
| BV node count | u32 |
| bounds min, max | f32 x 6 |
| cell size, cell height | f32 x 2 |
| walkable height, radius, climb | f32 x 3 |
| BV quantization factor | f32 |

Sections, in order:

| section | per element |
|---|---|
| vertices | f32 x 3 |
| polys | vertex base u32, vertex count u8, area u8, zero u16, flags u32 |
| poly vertices | u32 |
| poly neighbours | i32 |
| detail meshes | vertex base, triangle base, vertex count, triangle count: u32 x 4 |
| detail vertices | f32 x 3 |
| detail triangles | u8 x 4 |
| BV nodes | bmin u16 x 3, bmax u16 x 3, index i32 |

Not saved: links and `poly_nodes`, which `add_tile` rebuilds, and the portal index, which `add_tile` builds. Saving the BV tree trades file size for load time, as Detour does.

## Nav mesh file

Magic `RMNM`, version, then the nav mesh state, each array in index order with an `allocated` byte per slot and the fields only for allocated slots:

| part | contents |
|---|---|
| settings | origin (f32 x 3), tile width, tile height |
| nodes | count; per allocated node: type, area, ref, first link, flags, tile id, poly index, off-mesh connection id |
| links | count; per link: to node ref, next, edge, side, bmin, bmax (free links included, so link indices stay put) |
| tiles | count; per allocated tile: sequence, blob size, tile blob, node index per poly |
| off-mesh connections | count; per allocated connection: sequence, start, end, radius, direction, flags, area |
| attachments | count; per attached one: off-mesh node, start poly node, end poly node |
| tile columns | count; per column (sorted by key): key, tile ids in order |
| sequence counters | count; (tile position key, counter) pairs sorted by key |
| off-mesh sequence counter | u32 |
| index pools | node, tile, off-mesh connection and link pools: counter, then the free list in order |

Map contents are written sorted by key, so saving a loaded nav mesh gives the same bytes. Derived data is rebuilt on load: the tile position map and each tile's portal index.

## API sketch

In `roam/query/nav_mesh_io.h` (namespace `nav_mesh_io`):

```cpp
struct SaveResult {
    bool success;
    uint32_t byte_count;
    uint32_t byte_total;
    bool truncated;
};

enum LoadStatus : uint32_t {
    LOAD_STATUS_OK,
    LOAD_STATUS_BAD_MAGIC,
    LOAD_STATUS_UNSUPPORTED_VERSION,
    LOAD_STATUS_TRUNCATED,
    LOAD_STATUS_INVALID_DATA,
};

void save_tile(SaveResult &out, const NavMeshTile &tile, Span<uint8_t> bytes);
void save_nav_mesh(SaveResult &out, const NavMesh &nav_mesh, Span<uint8_t> bytes);

LoadStatus load_tile(NavMeshTile &tile, Span<const uint8_t> bytes);
LoadStatus load_nav_mesh(NavMesh &nav_mesh, Span<const uint8_t> bytes);
```

A loaded tile that never gets added is freed with `nav_mesh_api::release(NavMeshTile &)`, which also covers `build_tile`'s result. Following Detour, the status is the return value and the data goes out through a parameter.

- Saving follows the query buffer pattern: call with an empty span to get `byte_total`, size a buffer, call again.
- `load_tile` fills a tile ready for `add_tile`, like `build_tile`'s result. `load_nav_mesh` fills an empty nav mesh. Both own what they allocate, and on failure leave nothing allocated.

## Validation on load

Reads never go past the end, and every count is checked against the remaining bytes before anything is allocated. A tile is rejected unless every poly has 3 to `MAX_POLY_VERTICES` vertices within the poly vertex array, every poly vertex index is in range, neighbour entries are 0, an in-tile poly or an external flag with a direction below 8, detail meshes are absent or one per poly with ranges in bounds and triangle corners below the poly's vertex count plus its detail vertex count, and BV leaves point at polys.

A nav mesh is rejected unless every allocated node's ref matches its index, type and owner's sequence, and its owner points back at it (the tile's poly node, or the off-mesh connection's attachment); every link list is in bounds, acyclic and unshared, each link points at an allocated node with a matching ref, and poly-to-poly links name an edge the poly has; attachments point at matching nodes; every allocated tile is in exactly one column, the right one, and positions are unique; and each index pool's free list is exactly the unused slots, with counters matching the array lengths. On any failure the partially read nav mesh or tile is released and nothing leaks.

Fuzzing in the tests (every truncation, thousands of random bit flips) checks that loading never crashes and that anything accepted is safe to query and edit.

## Decisions

- Status return with an out parameter for loading, as Detour does.
- No per-tile user field for now.
- Compression is left to the user.
