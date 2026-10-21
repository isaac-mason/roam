# Saving and loading nav meshes

Draft design, 2026-10-21. Nothing here is built yet.

## Goals

- Bake a nav mesh offline and load it at runtime, or send it over a network.
- Save and load single tiles, for streaming and for caching rebuilt tiles.
- After loading a saved nav mesh, every node ref that was valid before saving is valid again and points at the same poly or off-mesh connection, as with navcat's `JSON.stringify` / `JSON.parse` round trip.
- Malformed or hostile bytes fail with an error. They never crash, assert or read out of bounds.
- No file I/O in roam: everything is byte buffers.

Not goals: a format navcat can read, or Detour binary compatibility. The format is roam's own, shaped like Detour's.

## How Detour does it

- A tile is one blob from `dtCreateNavMeshData`: a header (magic, version, tile x, y and layer, counts, walkable settings, bounds, BV quantization factor), then sections for vertices, polys, links, detail meshes, detail vertices, detail triangles, the BV tree and off-mesh connections, each 4-byte aligned. Links are reserved space, filled in by `addTile`.
- The sample's mesh file (`MSET`) is a header (magic, version, tile count, `dtNavMeshParams`) followed by each tile's ref, data size and blob. Loading calls `addTile(data, size, flags, tileRef)`, and passing the old tile ref keeps poly refs the same.

roam follows the same split: a self-contained tile blob, and a mesh file that is the mesh's settings plus tile blobs plus what's needed to give everything its old refs back.

## Encoding rules

- Little-endian, written and read field by field with explicit widths. Structs are never `memcpy`'d, so padding, alignment and host ABI don't matter, and the encoder is the same on every platform.
- Floats are written as their IEEE 754 bits, so a round trip is bit-exact.
- Every blob starts with a 4-byte magic and a `uint32` version. Readers reject unknown versions instead of guessing.
- Sections are 4-byte aligned, so a reader on a little-endian host could later read sections in place.

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
| polys | vertex base u32, vertex count u8, area u8, flags u32 |
| poly vertices | u32 |
| poly neighbours | i32 |
| detail meshes | vertex base, triangle base, vertex count, triangle count: u32 x 4 |
| detail vertices | f32 x 3 |
| detail triangles | u8 x 4 |
| BV nodes | bmin u16 x 3, bmax u16 x 3, index i32 |

Not saved: links and `poly_nodes`, which `add_tile` rebuilds, and the portal index, which `add_tile` builds. Saving the BV tree trades file size for load time, as Detour does.

## Nav mesh file

| part | contents |
|---|---|
| header | magic `RMNM`, version, origin (f32 x 3), tile width, tile height, tile count, off-mesh connection count |
| per tile | tile id u32, tile sequence u32, node index per poly (u32 x poly count), blob size u32, tile blob |
| per off-mesh connection | id u32, sequence u32, start, end, radius, direction, flags, area, and its off-mesh node index if it was attached |
| sequence counters | count, then (tile position key u64, counter u32) pairs |
| off-mesh sequence counter | u32 |

A node ref is type, node index and sequence. Restoring each tile's id, sequence and node indices, each off-mesh connection's id, sequence and node, and the sequence counters makes every saved ref valid again. Refs that were already stale before saving stay stale, because the counters carry on from where they were.

Loading creates an empty nav mesh, marks the saved tile ids, node indices and off-mesh ids as used in their index pools (everything else goes on the free lists in ascending order), adds each tile at its saved id and node indices, which rebuilds the links, then reattaches the off-mesh connections. Refs handed out after loading can differ from what the original mesh would have handed out next, because the free lists aren't saved in their original order. Nothing a user held before saving depends on that.

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

- Saving follows the query buffer pattern: call with an empty span to get `byte_total`, size a buffer, call again.
- `load_tile` fills a tile ready for `add_tile`, like `build_tile`'s result. `load_nav_mesh` fills an empty nav mesh. Both own what they allocate, and on failure leave nothing allocated.

## Validation on load

Every count is checked against the remaining bytes before anything is allocated, and every index is checked against what it indexes: poly vertex indices against the vertex count, poly vertex ranges against the poly vertex count, neighbour indices against the poly count, detail ranges against the detail counts, detail triangle vertices against their mesh's vertex count, BV node poly indices against the poly count, tile ids, node indices and off-mesh ids against duplicates and the format's limits. Vertex counts per poly are checked against `MAX_POLY_VERTICES`. Floats are not required to be finite, since the mesh may legitimately contain any value the generator produced.

## Open questions

1. `load_tile(NavMeshTile &, bytes)` returning a status, or a `LoadTileResult {status, tile}` returned by value like `build_tile`? The first keeps results from owning memory; the second reads more like `build_tile`.
2. Should tile blobs carry a user field (Detour's `userId`), for games that tag tiles?
3. Compression is left to the user (the blob is plain bytes); fine?
