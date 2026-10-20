import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { parseArgs } from 'node:util';

const { values: args } = parseArgs({
    options: {
        navcat: { type: 'string' },
        out: { type: 'string' },
    },
});

if (!args.navcat || !args.out) {
    console.error('usage: node dump.mjs --navcat <navcat checkout> --out <fixtures dir>');
    process.exit(1);
}

const navcatDir = resolve(args.navcat);
const outDir = resolve(args.out);

const newestModifiedTime = (dir) => {
    let newest = 0;
    for (const entry of readdirSync(dir, { withFileTypes: true })) {
        const path = join(dir, entry.name);
        newest = Math.max(newest, entry.isDirectory() ? newestModifiedTime(path) : statSync(path).mtimeMs);
    }
    return newest;
};

const distBuiltAt = statSync(join(navcatDir, 'dist/index.js')).mtimeMs;
const sourceChangedAt = Math.max(newestModifiedTime(join(navcatDir, 'src')), newestModifiedTime(join(navcatDir, 'blocks')));
if (sourceChangedAt > distBuiltAt) {
    console.error(`navcat dist is older than its sources, run \`pnpm build\` in ${navcatDir}`);
    process.exit(1);
}

const navcat = await import(join(navcatDir, 'dist/index.js'));

const writeFixture = (name, entries) => {
    const chunks = [];
    const header = Buffer.alloc(4);
    header.writeUInt32LE(entries.length, 0);
    chunks.push(header);

    for (const { name: entryName, type, values } of entries) {
        const nameBytes = Buffer.from(entryName, 'utf8');
        const entryHeader = Buffer.alloc(4 + nameBytes.length + 8);
        entryHeader.writeUInt32LE(nameBytes.length, 0);
        nameBytes.copy(entryHeader, 4);
        entryHeader.writeUInt32LE(type === 'f64' ? 0 : 1, 4 + nameBytes.length);
        entryHeader.writeUInt32LE(values.length, 8 + nameBytes.length);
        chunks.push(entryHeader);

        const data = type === 'f64' ? Float64Array.from(values) : Int32Array.from(values);
        chunks.push(Buffer.from(data.buffer, data.byteOffset, data.byteLength));
    }

    mkdirSync(outDir, { recursive: true });
    writeFileSync(join(outDir, `${name}.bin`), Buffer.concat(chunks));
};

const f64 = (name, values) => ({ name, type: 'f64', values: typeof values === 'number' ? [values] : Array.from(values) });
const i32 = (name, values) => ({ name, type: 'i32', values: typeof values === 'number' ? [values] : Array.from(values) });

const heightfieldEntries = (prefix, heightfield) => {
    const columnSpanCounts = [];
    const spanMin = [];
    const spanMax = [];
    const spanArea = [];
    for (const first of heightfield.spans) {
        let count = 0;
        for (let span = first; span; span = span.next) {
            spanMin.push(span.min);
            spanMax.push(span.max);
            spanArea.push(span.area);
            count++;
        }
        columnSpanCounts.push(count);
    }
    return [
        i32(`${prefix}.width`, heightfield.width),
        i32(`${prefix}.height`, heightfield.height),
        f64(`${prefix}.bounds`, heightfield.bounds),
        f64(`${prefix}.cell_size`, heightfield.cellSize),
        f64(`${prefix}.cell_height`, heightfield.cellHeight),
        i32(`${prefix}.column_span_counts`, columnSpanCounts),
        i32(`${prefix}.span_min`, spanMin),
        i32(`${prefix}.span_max`, spanMax),
        i32(`${prefix}.span_area`, spanArea),
    ];
};

const compactHeightfieldEntries = (prefix, chf) => [
    i32(`${prefix}.span_count`, chf.spanCount),
    f64(`${prefix}.bounds`, chf.bounds),
    i32(`${prefix}.cell_index`, chf.cells.map((cell) => cell.index)),
    i32(`${prefix}.cell_count`, chf.cells.map((cell) => cell.count)),
    i32(`${prefix}.span_y`, chf.spans.map((span) => span.y)),
    i32(`${prefix}.span_h`, chf.spans.map((span) => span.h)),
    i32(`${prefix}.span_con`, chf.spans.map((span) => span.con)),
    i32(`${prefix}.areas`, chf.areas),
];

const regionEntries = (prefix, chf) => [
    i32(`${prefix}.span_region`, chf.spans.map((span) => span.region)),
    i32(`${prefix}.max_regions`, chf.maxRegions),
];

const contourSetEntries = (prefix, contourSet) => [
    f64(`${prefix}.bounds`, contourSet.bounds),
    i32(`${prefix}.width`, contourSet.width),
    i32(`${prefix}.height`, contourSet.height),
    i32(`${prefix}.vertex_counts`, contourSet.contours.map((contour) => contour.nVertices)),
    i32(`${prefix}.vertices`, contourSet.contours.flatMap((contour) => contour.vertices.slice(0, contour.nVertices * 4))),
    i32(`${prefix}.raw_vertex_counts`, contourSet.contours.map((contour) => contour.nRawVertices)),
    i32(`${prefix}.raw_vertices`, contourSet.contours.flatMap((contour) => contour.rawVertices.slice(0, contour.nRawVertices * 4))),
    i32(`${prefix}.regions`, contourSet.contours.map((contour) => contour.reg)),
    i32(`${prefix}.areas`, contourSet.contours.map((contour) => contour.area)),
];

const polyMeshEntries = (prefix, polyMesh) => [
    i32(`${prefix}.vertex_count`, polyMesh.nVertices),
    i32(`${prefix}.poly_count`, polyMesh.nPolys),
    i32(`${prefix}.vertices`, polyMesh.vertices.slice(0, polyMesh.nVertices * 3)),
    i32(`${prefix}.polys`, polyMesh.polys.slice(0, polyMesh.nPolys * polyMesh.maxVerticesPerPoly)),
    i32(`${prefix}.regions`, polyMesh.regions.slice(0, polyMesh.nPolys)),
    i32(`${prefix}.areas`, polyMesh.areas.slice(0, polyMesh.nPolys)),
    f64(`${prefix}.bounds`, polyMesh.bounds),
];

const polyMeshDetailEntries = (prefix, detail) => [
    i32(`${prefix}.mesh_count`, detail.nMeshes),
    i32(`${prefix}.meshes`, detail.meshes),
    f64(`${prefix}.vertices`, detail.vertices),
    i32(`${prefix}.triangles`, detail.triangles),
];

const tilePolysEntries = (prefix, tilePolys) => [
    f64(`${prefix}.vertices`, tilePolys.vertices),
    i32(`${prefix}.poly_vertex_counts`, tilePolys.polys.map((poly) => poly.vertices.length)),
    i32(`${prefix}.poly_vertices`, tilePolys.polys.flatMap((poly) => poly.vertices)),
    i32(`${prefix}.poly_neis`, tilePolys.polys.flatMap((poly) => poly.neis)),
    i32(`${prefix}.poly_flags`, tilePolys.polys.map((poly) => poly.flags)),
    i32(`${prefix}.poly_areas`, tilePolys.polys.map((poly) => poly.area)),
];

const tileDetailEntries = (prefix, tileDetail) => [
    i32(
        `${prefix}.detail_meshes`,
        tileDetail.detailMeshes.flatMap((mesh) => [mesh.verticesBase, mesh.trianglesBase, mesh.verticesCount, mesh.trianglesCount]),
    ),
    f64(`${prefix}.detail_vertices`, tileDetail.detailVertices),
    i32(`${prefix}.detail_triangles`, tileDetail.detailTriangles),
];

const navMeshEntries = (prefix, navMesh) => {
    const nodeRefs = [];
    const nodeAllocated = [];
    const linkCounts = [];
    const linkToRefs = [];
    const linkFields = [];
    for (const node of navMesh.nodes) {
        nodeRefs.push(node.ref);
        nodeAllocated.push(node.allocated ? 1 : 0);
        linkCounts.push(node.links.length);
        for (const linkIndex of node.links) {
            const link = navMesh.links[linkIndex];
            linkToRefs.push(link.toNodeRef);
            linkFields.push(link.edge, link.side, link.bmin, link.bmax);
        }
    }
    return [
        f64(`${prefix}.node_refs`, nodeRefs),
        i32(`${prefix}.node_allocated`, nodeAllocated),
        i32(`${prefix}.link_counts`, linkCounts),
        f64(`${prefix}.link_to_refs`, linkToRefs),
        i32(`${prefix}.link_fields`, linkFields),
    ];
};

const lcg = (seed) => {
    let state = seed >>> 0;
    return () => {
        state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
        return state / 4294967296;
    };
};

const config = {
    cellSize: 0.15,
    cellHeight: 0.15,
    walkableRadiusWorld: 0.1,
    walkableClimbWorld: 0.5,
    walkableHeightWorld: 0.25,
    walkableSlopeAngleDegrees: 45,
    borderSize: 0,
    minRegionArea: 8,
    mergeRegionArea: 20,
    maxSimplificationError: 1.3,
    maxEdgeLength: 12,
    maxVerticesPerPoly: 5,
    detailSampleDistance: 6,
    detailSampleMaxError: 1,
};

const options = {
    ...config,
    walkableRadiusVoxels: Math.ceil(config.walkableRadiusWorld / config.cellSize),
    walkableClimbVoxels: Math.ceil(config.walkableClimbWorld / config.cellHeight),
    walkableHeightVoxels: Math.ceil(config.walkableHeightWorld / config.cellHeight),
    detailSampleDistanceWorld: config.detailSampleDistance < 0.9 ? 0 : config.cellSize * config.detailSampleDistance,
    detailSampleMaxErrorWorld: config.cellHeight * config.detailSampleMaxError,
};
options.walkableRadiusWorld = config.walkableRadiusWorld;
options.walkableClimbWorld = config.walkableClimbWorld;
options.walkableHeightWorld = config.walkableHeightWorld;

const dumpSoloGeneration = (name, positions, indices) => {
    const entries = [
        f64('input.positions', positions),
        i32('input.indices', indices),
        f64('options.cell_size', options.cellSize),
        f64('options.cell_height', options.cellHeight),
        f64('options.walkable_slope_angle_degrees', options.walkableSlopeAngleDegrees),
        i32('options.walkable_radius_voxels', options.walkableRadiusVoxels),
        i32('options.walkable_climb_voxels', options.walkableClimbVoxels),
        i32('options.walkable_height_voxels', options.walkableHeightVoxels),
        i32('options.border_size', options.borderSize),
        i32('options.min_region_area', options.minRegionArea),
        i32('options.merge_region_area', options.mergeRegionArea),
        f64('options.max_simplification_error', options.maxSimplificationError),
        i32('options.max_edge_length', options.maxEdgeLength),
        i32('options.max_vertices_per_poly', options.maxVerticesPerPoly),
        f64('options.detail_sample_distance', options.detailSampleDistanceWorld),
        f64('options.detail_sample_max_error', options.detailSampleMaxErrorWorld),
    ];

    const ctx = navcat.BuildContext.create();

    const triAreaIds = new Uint8Array(indices.length / 3);
    navcat.markWalkableTriangles(positions, indices, triAreaIds, options.walkableSlopeAngleDegrees);
    entries.push(i32('tri_area_ids', triAreaIds));

    const bounds = navcat.calculateMeshBounds([0, 0, 0, 0, 0, 0], positions, indices);
    const [width, height] = navcat.calculateGridSize([0, 0], bounds, options.cellSize);
    entries.push(f64('mesh_bounds', bounds), i32('grid_size', [width, height]));

    const heightfield = navcat.createHeightfield(width, height, bounds, options.cellSize, options.cellHeight);
    navcat.rasterizeTriangles(ctx, heightfield, positions, indices, triAreaIds, options.walkableClimbVoxels);
    entries.push(...heightfieldEntries('heightfield.rasterized', heightfield));

    navcat.filterLowHangingWalkableObstacles(heightfield, options.walkableClimbVoxels);
    entries.push(...heightfieldEntries('heightfield.filtered_low_hanging', heightfield));

    navcat.filterLedgeSpans(heightfield, options.walkableHeightVoxels, options.walkableClimbVoxels);
    entries.push(...heightfieldEntries('heightfield.filtered_ledges', heightfield));

    navcat.filterWalkableLowHeightSpans(heightfield, options.walkableHeightVoxels);
    entries.push(...heightfieldEntries('heightfield.filtered_low_height', heightfield));

    const buildCompact = () => navcat.buildCompactHeightfield(ctx, options.walkableHeightVoxels, options.walkableClimbVoxels, heightfield);

    const chf = buildCompact();
    entries.push(...compactHeightfieldEntries('compact_heightfield.built', chf));

    navcat.erodeWalkableArea(options.walkableRadiusVoxels, chf);
    entries.push(i32('compact_heightfield.eroded.areas', chf.areas));

    navcat.buildDistanceField(chf);
    entries.push(i32('compact_heightfield.distances', chf.distances), i32('compact_heightfield.max_distance', chf.maxDistance));

    navcat.buildRegions(ctx, chf, options.borderSize, options.minRegionArea, options.mergeRegionArea);
    entries.push(...regionEntries('regions.watershed', chf));

    const contourSet = navcat.buildContours(ctx, chf, options.maxSimplificationError, options.maxEdgeLength, navcat.ContourBuildFlags.CONTOUR_TESS_WALL_EDGES);
    entries.push(...contourSetEntries('contours.watershed', contourSet));

    const polyMesh = navcat.buildPolyMesh(ctx, contourSet, options.maxVerticesPerPoly);
    entries.push(...polyMeshEntries('poly_mesh.watershed', polyMesh));

    const detail = navcat.buildPolyMeshDetail(ctx, polyMesh, chf, options.detailSampleDistanceWorld, options.detailSampleMaxErrorWorld);
    entries.push(...polyMeshDetailEntries('poly_mesh_detail.watershed', detail));

    const tilePolys = navcat.polyMeshToTilePolys(polyMesh);
    entries.push(...tilePolysEntries('tile.watershed', tilePolys));
    entries.push(...tileDetailEntries('tile.watershed', navcat.polyMeshDetailToTileDetailMesh(tilePolys.polys, detail)));
    entries.push(...tileDetailEntries('tile.watershed.fan', navcat.polysToTileDetailMesh(tilePolys.polys)));

    for (const poly of tilePolys.polys) {
        if (poly.area === navcat.WALKABLE_AREA) poly.area = 0;
        if (poly.area === 0) poly.flags = 1;
    }
    const tileDetail = navcat.polyMeshDetailToTileDetailMesh(tilePolys.polys, detail);
    const tile = navcat.buildTile({
        bounds: polyMesh.bounds,
        vertices: tilePolys.vertices,
        polys: tilePolys.polys,
        detailMeshes: tileDetail.detailMeshes,
        detailVertices: tileDetail.detailVertices,
        detailTriangles: tileDetail.detailTriangles,
        tileX: 0,
        tileY: 0,
        tileLayer: 0,
        cellSize: options.cellSize,
        cellHeight: options.cellHeight,
        walkableHeight: options.walkableHeightWorld,
        walkableRadius: options.walkableRadiusWorld,
        walkableClimb: options.walkableClimbWorld,
    });
    const navMesh = navcat.createNavMesh();
    navMesh.tileWidth = polyMesh.bounds[3] - polyMesh.bounds[0];
    navMesh.tileHeight = polyMesh.bounds[5] - polyMesh.bounds[2];
    navMesh.origin = [polyMesh.bounds[0], polyMesh.bounds[1], polyMesh.bounds[2]];
    navcat.addTile(navMesh, tile);

    entries.push(
        f64('nav_mesh.walkable', [options.walkableHeightWorld, options.walkableRadiusWorld, options.walkableClimbWorld]),
        i32('nav_mesh.bv_bounds', tile.bvTree.nodes.flatMap((node) => node.bounds)),
        i32('nav_mesh.bv_i', tile.bvTree.nodes.map((node) => node.i)),
        f64('nav_mesh.bv_quant_factor', tile.bvTree.quantFactor),
    );
    entries.push(...navMeshEntries('nav_mesh.built', navMesh));

    const random = lcg(1234);
    const meshBounds = polyMesh.bounds;
    const queryPoints = [];
    const nearestSuccess = [];
    const nearestRefs = [];
    const nearestPositions = [];
    const queryCounts = [];
    const queryRefs = [];
    const halfExtents = [1, 2, 1];
    for (let i = 0; i < 200; i++) {
        const point = [
            meshBounds[0] + random() * (meshBounds[3] - meshBounds[0]),
            meshBounds[1] + random() * (meshBounds[4] - meshBounds[1]),
            meshBounds[2] + random() * (meshBounds[5] - meshBounds[2]),
        ];
        queryPoints.push(...point);
        const nearest = navcat.findNearestPoly(navcat.createFindNearestPolyResult(), navMesh, point, halfExtents, navcat.DEFAULT_QUERY_FILTER);
        nearestSuccess.push(nearest.success ? 1 : 0);
        nearestRefs.push(nearest.success ? nearest.nodeRef : -1);
        nearestPositions.push(...nearest.position);
        const box = [point[0] - halfExtents[0], point[1] - halfExtents[1], point[2] - halfExtents[2], point[0] + halfExtents[0], point[1] + halfExtents[1], point[2] + halfExtents[2]];
        const refs = navcat.queryPolygons(navMesh, box, navcat.DEFAULT_QUERY_FILTER);
        queryCounts.push(refs.length);
        queryRefs.push(...refs);
    }
    entries.push(
        f64('queries.points', queryPoints),
        f64('queries.half_extents', halfExtents),
        i32('queries.nearest_success', nearestSuccess),
        f64('queries.nearest_refs', nearestRefs),
        f64('queries.nearest_positions', nearestPositions),
        i32('queries.polygon_counts', queryCounts),
        f64('queries.polygon_refs', queryRefs),
    );

    const successes = [];
    for (let i = 0; i < nearestSuccess.length; i++) if (nearestSuccess[i]) successes.push(i);
    const startIndex = successes[0];
    const endIndex = successes[successes.length - 1];
    const offMeshStart = nearestPositions.slice(startIndex * 3, startIndex * 3 + 3);
    const offMeshEnd = nearestPositions.slice(endIndex * 3, endIndex * 3 + 3);
    const offMeshId = navcat.addOffMeshConnection(navMesh, {
        start: offMeshStart,
        end: offMeshEnd,
        radius: 0.5,
        direction: navcat.OffMeshConnectionDirection.BIDIRECTIONAL,
        flags: 1,
        area: 0,
    });
    const attachment = navMesh.offMeshConnectionAttachments[offMeshId];
    entries.push(
        f64('off_mesh.start', offMeshStart),
        f64('off_mesh.end', offMeshEnd),
        i32('off_mesh.connected', navcat.isOffMeshConnectionConnected(navMesh, offMeshId) ? 1 : 0),
        f64('off_mesh.attachment', [attachment.offMeshNode, attachment.startPolyNode, attachment.endPolyNode]),
    );
    entries.push(...navMeshEntries('nav_mesh.with_off_mesh', navMesh));

    const monotone = buildCompact();
    navcat.erodeWalkableArea(options.walkableRadiusVoxels, monotone);
    navcat.buildDistanceField(monotone);
    navcat.buildRegionsMonotone(ctx, monotone, options.borderSize, options.minRegionArea, options.mergeRegionArea);
    entries.push(...regionEntries('regions.monotone', monotone));

    const layers = buildCompact();
    navcat.erodeWalkableArea(options.walkableRadiusVoxels, layers);
    navcat.buildDistanceField(layers);
    navcat.buildLayerRegions(ctx, layers, options.borderSize, options.minRegionArea);
    entries.push(...regionEntries('regions.layers', layers));

    const tessellateAll = navcat.ContourBuildFlags.CONTOUR_TESS_WALL_EDGES | navcat.ContourBuildFlags.CONTOUR_TESS_AREA_EDGES;
    const monotoneContours = navcat.buildContours(ctx, monotone, options.maxSimplificationError, options.maxEdgeLength, tessellateAll);
    const layerContours = navcat.buildContours(ctx, layers, options.maxSimplificationError, options.maxEdgeLength, tessellateAll);
    entries.push(...contourSetEntries('contours.monotone', monotoneContours));
    entries.push(...contourSetEntries('contours.layers', layerContours));
    entries.push(...polyMeshEntries('poly_mesh.monotone', navcat.buildPolyMesh(ctx, monotoneContours, options.maxVerticesPerPoly)));
    entries.push(...polyMeshEntries('poly_mesh.layers', navcat.buildPolyMesh(ctx, layerContours, options.maxVerticesPerPoly)));
    entries.push(...polyMeshEntries('poly_mesh.triangles', navcat.buildPolyMesh(ctx, contourSet, 3)));

    writeFixture(name, entries);
    console.log(`wrote ${name}.bin (${entries.length} entries)`);
};

const navTest = JSON.parse(readFileSync(join(navcatDir, 'bench/fixtures/nav-test.json'), 'utf8'));
dumpSoloGeneration('nav_test_solo', new Float32Array(navTest.positions), new Uint32Array(navTest.indices));
