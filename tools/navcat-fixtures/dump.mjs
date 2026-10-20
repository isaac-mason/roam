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
const blocks = await import(join(navcatDir, 'dist/blocks.js'));

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

const refValue = (ref) => (ref === null || ref === undefined || ref === navcat.INVALID_NODE_REF ? -1 : ref);

const pointsEntries = (prefix, points, counts) => [
    i32(`${prefix}.counts`, counts),
    f64(`${prefix}.positions`, points.flatMap((point) => point.position)),
    i32(`${prefix}.types`, points.map((point) => point.type)),
    f64(`${prefix}.refs`, points.map((point) => refValue(point.nodeRef))),
    i32(`${prefix}.flags`, points.map((point) => point.flags)),
];

const queryEntries = (navMesh, positions, halfExtents) => {
    const filter = navcat.DEFAULT_QUERY_FILTER;
    const pairs = [];
    for (let i = 0; i < 60; i++) pairs.push([positions[i % positions.length], positions[(i * 37 + 11) % positions.length]]);

    const out = {
        pairStarts: [], pairEnds: [],
        pathSuccess: [], pathFlags: [], nodePathCounts: [], nodePathRefs: [], straight: [], straightCounts: [], crossingCounts: [], smoothCounts: [],
        anyAngleNodePathCounts: [], anyAngleNodePathRefs: [],
        crossings: [],
        smooth: [],
        rayT: [], rayEdge: [], rayNormal: [], rayCounts: [], rayRefs: [],
        moveSuccess: [], movePositions: [], moveRefs: [], moveCounts: [], moveVisited: [],
        neighbourhoodCounts: [], neighbourhoodRefs: [],
        wallCounts: [], wallVertices: [], wallRefs: [],
        slicedStatus: [], slicedCounts: [], slicedRefs: [],
    };

    for (const [start, end] of pairs) {
        out.pairStarts.push(...start);
        out.pairEnds.push(...end);

        const path = navcat.findPath(navMesh, start, end, halfExtents, filter);
        out.pathSuccess.push(path.success ? 1 : 0);
        out.pathFlags.push(path.flags);
        const nodePath = path.nodePath ? path.nodePath.path : [];
        out.nodePathCounts.push(nodePath.length);
        out.nodePathRefs.push(...nodePath);
        out.straight.push(...path.path);
        out.straightCounts.push(path.path.length);

        const anyAngle = navcat.findPath(navMesh, start, end, halfExtents, filter, { raycastDistance: 10 });
        const anyAnglePath = anyAngle.nodePath ? anyAngle.nodePath.path : [];
        out.anyAngleNodePathCounts.push(anyAnglePath.length);
        out.anyAngleNodePathRefs.push(...anyAnglePath);

        if (path.success) {
            const crossings = navcat.findStraightPath(navMesh, path.startPosition, path.endPosition, nodePath, null, navcat.FindStraightPathOptions.ALL_CROSSINGS);
            out.crossings.push(...crossings.path);
            out.crossingCounts.push(crossings.path.length);
        } else {
            out.crossingCounts.push(0);
        }

        const smooth = navcat.findSmoothPath(navMesh, start, end, halfExtents, filter, { stepSize: 0.5, slop: 0.01, maxPoints: 256 });
        out.smooth.push(...smooth.path);
        out.smoothCounts.push(smooth.path.length);

        const startRef = path.startNodeRef;
        const ray = navcat.raycast(navMesh, startRef, path.startPosition, end, filter);
        out.rayT.push(ray.t);
        out.rayEdge.push(ray.hitEdgeIndex);
        out.rayNormal.push(...ray.hitNormal);
        out.rayCounts.push(ray.path.length);
        out.rayRefs.push(...ray.path);

        const move = navcat.moveAlongSurface(navMesh, startRef, path.startPosition, end, filter);
        out.moveSuccess.push(move.success ? 1 : 0);
        out.movePositions.push(...move.position);
        out.moveRefs.push(refValue(move.nodeRef));
        out.moveCounts.push(move.visited.length);
        out.moveVisited.push(...move.visited);

        const neighbourhood = navcat.findLocalNeighbourhood(navMesh, startRef, path.startPosition, 2, filter);
        out.neighbourhoodCounts.push(neighbourhood.nodeRefs.length);
        out.neighbourhoodRefs.push(...neighbourhood.nodeRefs);

        const walls = navcat.getPolyWallSegments(navMesh, startRef, filter, true);
        out.wallCounts.push(walls.segmentRefs.length);
        out.wallVertices.push(...walls.segmentVerts);
        out.wallRefs.push(...walls.segmentRefs.map(refValue));

        const query = navcat.createSlicedNodePathQuery();
        navcat.initSlicedFindNodePath(navMesh, query, startRef, path.endNodeRef, path.startPosition, path.endPosition, filter, navcat.SlicedFindNodePathInitFlags.ANY_ANGLE);
        while (query.status & navcat.SlicedFindNodePathStatusFlags.IN_PROGRESS) navcat.updateSlicedFindNodePath(navMesh, query, 7);
        const sliced = navcat.finalizeSlicedFindNodePath(navMesh, query);
        out.slicedStatus.push(sliced.status);
        out.slicedCounts.push(sliced.path.length);
        out.slicedRefs.push(...sliced.path);
    }

    const random = lcg(99);
    const randomPoints = [];
    for (let i = 0; i < 20; i++) {
        const point = navcat.findRandomPoint(navMesh, filter, random);
        randomPoints.push(refValue(point.success ? point.nodeRef : -1), ...point.position);
    }
    const circleRandom = lcg(7);
    const circlePoints = [];
    for (let i = 0; i < 20; i++) {
        const start = positions[i % positions.length];
        const nearest = navcat.findNearestPoly(navcat.createFindNearestPolyResult(), navMesh, start, halfExtents, filter);
        const point = navcat.findRandomPointAroundCircle(navMesh, nearest.nodeRef, start, 3, filter, circleRandom);
        circlePoints.push(refValue(point.success ? point.nodeRef : -1), ...point.position);
    }

    return [
        f64('paths.starts', out.pairStarts),
        f64('paths.ends', out.pairEnds),
        i32('paths.success', out.pathSuccess),
        i32('paths.flags', out.pathFlags),
        i32('paths.node_path_counts', out.nodePathCounts),
        f64('paths.node_path_refs', out.nodePathRefs),
        ...pointsEntries('paths.straight', out.straight, out.straightCounts),
        i32('paths.any_angle_node_path_counts', out.anyAngleNodePathCounts),
        f64('paths.any_angle_node_path_refs', out.anyAngleNodePathRefs),
        ...pointsEntries('paths.crossings', out.crossings, out.crossingCounts),
        ...pointsEntries('paths.smooth', out.smooth, out.smoothCounts),
        f64('paths.ray_t', out.rayT),
        i32('paths.ray_edge', out.rayEdge),
        f64('paths.ray_normal', out.rayNormal),
        i32('paths.ray_counts', out.rayCounts),
        f64('paths.ray_refs', out.rayRefs),
        i32('paths.move_success', out.moveSuccess),
        f64('paths.move_positions', out.movePositions),
        f64('paths.move_refs', out.moveRefs),
        i32('paths.move_counts', out.moveCounts),
        f64('paths.move_visited', out.moveVisited),
        i32('paths.neighbourhood_counts', out.neighbourhoodCounts),
        f64('paths.neighbourhood_refs', out.neighbourhoodRefs),
        i32('paths.wall_counts', out.wallCounts),
        f64('paths.wall_vertices', out.wallVertices),
        f64('paths.wall_refs', out.wallRefs),
        i32('paths.sliced_status', out.slicedStatus),
        i32('paths.sliced_counts', out.slicedCounts),
        f64('paths.sliced_refs', out.slicedRefs),
        f64('random.points', randomPoints),
        f64('random.circle_points', circlePoints),
    ];
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

    const interior = successes.filter(
        (index) => nearestPositions[index * 3] === queryPoints[index * 3] && nearestPositions[index * 3 + 2] === queryPoints[index * 3 + 2],
    );
    entries.push(...queryEntries(navMesh, interior.map((index) => nearestPositions.slice(index * 3, index * 3 + 3)), halfExtents));

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

const navMeshTilesEntries = (prefix, navMesh) => {
    const tiles = Object.values(navMesh.tiles);
    const entries = [
        i32(`${prefix}.tile_count`, tiles.length),
        i32(`${prefix}.tile_positions`, tiles.flatMap((tile) => [tile.tileX, tile.tileY, tile.tileLayer])),
        f64(`${prefix}.tile_bounds`, tiles.flatMap((tile) => tile.bounds)),
        i32(`${prefix}.tile_vertex_counts`, tiles.map((tile) => tile.vertices.length)),
        i32(`${prefix}.tile_poly_counts`, tiles.map((tile) => tile.polys.length)),
        i32(`${prefix}.tile_detail_vertex_counts`, tiles.map((tile) => tile.detailVertices.length)),
        i32(`${prefix}.tile_detail_triangle_counts`, tiles.map((tile) => tile.detailTriangles.length)),
        f64(`${prefix}.vertices`, tiles.flatMap((tile) => Array.from(tile.vertices))),
        i32(`${prefix}.poly_vertex_counts`, tiles.flatMap((tile) => tile.polys.map((poly) => poly.vertices.length))),
        i32(`${prefix}.poly_vertices`, tiles.flatMap((tile) => tile.polys.flatMap((poly) => poly.vertices))),
        i32(`${prefix}.poly_neis`, tiles.flatMap((tile) => tile.polys.flatMap((poly) => poly.neis))),
        i32(`${prefix}.poly_flags`, tiles.flatMap((tile) => tile.polys.map((poly) => poly.flags))),
        i32(`${prefix}.poly_areas`, tiles.flatMap((tile) => tile.polys.map((poly) => poly.area))),
        i32(`${prefix}.detail_meshes`, tiles.flatMap((tile) => tile.detailMeshes.flatMap((mesh) => [mesh.verticesBase, mesh.verticesCount, mesh.trianglesBase, mesh.trianglesCount]))),
        f64(`${prefix}.detail_vertices`, tiles.flatMap((tile) => Array.from(tile.detailVertices))),
        i32(`${prefix}.detail_triangles`, tiles.flatMap((tile) => Array.from(tile.detailTriangles))),
    ];
    return [...entries, ...navMeshEntries(prefix, navMesh)];
};

const dumpBlocks = (name, positions, indices) => {
    const entries = [];

    const half = indices.length / 2 - ((indices.length / 2) % 3);
    const [mergedPositions, mergedIndices] = blocks.mergePositionsAndIndices([
        { positions, indices: indices.slice(0, half) },
        { positions, indices: indices.slice(half) },
        { positions, indices: indices.slice(0, 30) },
    ]);
    entries.push(i32('merge.split', half), f64('merge.positions', mergedPositions), i32('merge.indices', mergedIndices));

    const chunky = blocks.chunkyTriMesh.create(positions, indices, 64);
    entries.push(
        f64('chunky.bounds', chunky.nodes.flatMap((node) => [...node.bounds[0], ...node.bounds[1]])),
        i32('chunky.index', chunky.nodes.map((node) => node.index)),
        i32('chunky.count', chunky.nodes.map((node) => node.count)),
        i32('chunky.triangles', chunky.triangles),
        i32('chunky.max_tris_per_chunk', chunky.maxTrisPerChunk),
    );

    const meshBounds = navcat.calculateMeshBounds([0, 0, 0, 0, 0, 0], positions, indices);
    const random = lcg(4321);
    const randomXZ = () => [meshBounds[0] + random() * (meshBounds[3] - meshBounds[0]), meshBounds[2] + random() * (meshBounds[5] - meshBounds[2])];
    const rects = [];
    const rectCounts = [];
    const rectChunks = [];
    const rectTriangleCounts = [];
    const segments = [];
    const segmentCounts = [];
    const segmentChunks = [];
    for (let i = 0; i < 40; i++) {
        const a = randomXZ();
        const b = randomXZ();
        const min = [Math.min(a[0], b[0]), Math.min(a[1], b[1])];
        const max = [Math.max(a[0], b[0]), Math.max(a[1], b[1])];
        rects.push(...min, ...max);
        const chunks = blocks.chunkyTriMesh.getChunksOverlappingRect(chunky, min, max);
        rectCounts.push(chunks.length);
        rectChunks.push(...chunks);
        rectTriangleCounts.push(blocks.chunkyTriMesh.getTrianglesInRect(chunky, min, max).length);
        segments.push(...a, ...b);
        const segmentHits = blocks.chunkyTriMesh.getChunksOverlappingSegment(chunky, a, b);
        segmentCounts.push(segmentHits.length);
        segmentChunks.push(...segmentHits);
    }
    entries.push(
        f64('chunky.rects', rects),
        i32('chunky.rect_counts', rectCounts),
        i32('chunky.rect_chunks', rectChunks),
        i32('chunky.rect_triangle_counts', rectTriangleCounts),
        f64('chunky.segments', segments),
        i32('chunky.segment_counts', segmentCounts),
        i32('chunky.segment_chunks', segmentChunks),
    );

    const generatorOptions = {
        cellSize: options.cellSize,
        cellHeight: options.cellHeight,
        walkableRadiusVoxels: options.walkableRadiusVoxels,
        walkableRadiusWorld: options.walkableRadiusWorld,
        walkableClimbVoxels: options.walkableClimbVoxels,
        walkableClimbWorld: options.walkableClimbWorld,
        walkableHeightVoxels: options.walkableHeightVoxels,
        walkableHeightWorld: options.walkableHeightWorld,
        walkableSlopeAngleDegrees: options.walkableSlopeAngleDegrees,
        borderSize: options.borderSize,
        minRegionArea: options.minRegionArea,
        mergeRegionArea: options.mergeRegionArea,
        maxSimplificationError: options.maxSimplificationError,
        maxEdgeLength: options.maxEdgeLength,
        maxVerticesPerPoly: options.maxVerticesPerPoly,
        detailSampleDistance: options.detailSampleDistanceWorld,
        detailSampleMaxError: options.detailSampleMaxErrorWorld,
    };

    const solo = blocks.generateSoloNavMesh({ positions, indices }, generatorOptions);
    entries.push(...navMeshTilesEntries('solo', solo.navMesh));

    const tileSizeVoxels = 32;
    const tiledOptions = {
        ...generatorOptions,
        tileSizeVoxels,
        tileSizeWorld: tileSizeVoxels * options.cellSize,
        borderSize: options.walkableRadiusVoxels + 3,
    };
    entries.push(i32('tiled.tile_size_voxels', tiledOptions.tileSizeVoxels), f64('tiled.tile_size_world', tiledOptions.tileSizeWorld), i32('tiled.border_size', tiledOptions.borderSize));
    const tiled = blocks.generateTiledNavMesh({ positions, indices }, tiledOptions);
    entries.push(...navMeshTilesEntries('tiled', tiled.navMesh));

    const seeds = [];
    for (const node of tiled.navMesh.nodes) {
        if (node.allocated && seeds.length < 1) seeds.push(node.ref);
    }
    const flood = blocks.floodFillNavMesh(tiled.navMesh, seeds);
    entries.push(f64('flood.seeds', seeds), f64('flood.reachable', flood.reachable), f64('flood.unreachable', flood.unreachable));

    const agentRandom = lcg(2024);
    const agentPoints = [];
    for (let i = 0; i < 24; i++) {
        const point = navcat.findRandomPoint(tiled.navMesh, navcat.DEFAULT_QUERY_FILTER, agentRandom);
        if (point.success) agentPoints.push({ ref: point.nodeRef, position: [...point.position] });
    }
    entries.push(f64('agents.points', agentPoints.flatMap((point) => [point.ref, ...point.position])));

    const boundaryCounts = [];
    const boundarySegments = [];
    const boundaryPolyCounts = [];
    const boundaryPolys = [];
    for (const point of agentPoints) {
        const boundary = blocks.localBoundary.create();
        blocks.localBoundary.updateLocalBoundary(boundary, point.ref, point.position, 1.5, tiled.navMesh, navcat.DEFAULT_QUERY_FILTER);
        boundaryCounts.push(boundary.segments.length);
        for (const segment of boundary.segments) boundarySegments.push(...segment.s, segment.d);
        boundaryPolyCounts.push(boundary.polys.length);
        boundaryPolys.push(...boundary.polys);
    }
    entries.push(
        i32('boundary.segment_counts', boundaryCounts),
        f64('boundary.segments', boundarySegments),
        i32('boundary.poly_counts', boundaryPolyCounts),
        f64('boundary.polys', boundaryPolys),
    );

    const avoidanceRandom = lcg(77);
    const avoidanceInputs = [];
    const avoidanceOutputs = [];
    const params = blocks.crowd.DEFAULT_OBSTACLE_AVOIDANCE_PARAMS;
    for (let i = 0; i < 20; i++) {
        const r = () => avoidanceRandom() * 4 - 2;
        const query = blocks.obstacleAvoidance.createObstacleAvoidanceQuery(8, 8);
        const position = [r(), 0, r()];
        const velocity = [r() * 0.5, 0, r() * 0.5];
        const desired = [r(), 0, r()];
        const input = [...position, ...velocity, ...desired];
        for (let c = 0; c < 3; c++) {
            const circle = [r(), 0, r(), 0.3 + avoidanceRandom() * 0.3, r() * 0.5, 0, r() * 0.5, r() * 0.5, 0, r() * 0.5];
            input.push(...circle);
            blocks.obstacleAvoidance.addCircleObstacle(query, circle.slice(0, 3), circle[3], circle.slice(4, 7), circle.slice(7, 10));
        }
        for (let c = 0; c < 2; c++) {
            const segment = [r(), 0, r(), r(), 0, r()];
            input.push(...segment);
            blocks.obstacleAvoidance.addSegmentObstacle(query, segment.slice(0, 3), segment.slice(3, 6));
        }
        avoidanceInputs.push(...input);
        const adaptive = [0, 0, 0];
        const adaptiveSamples = blocks.obstacleAvoidance.sampleVelocityAdaptive(query, position, 0.4, 2, velocity, desired, params, adaptive);
        const grid = blocks.obstacleAvoidance.sampleVelocityGrid(query, position, 0.4, 2, velocity, desired, { ...params, gridSize: 9 });
        avoidanceOutputs.push(...adaptive, adaptiveSamples, ...grid.nvel, grid.samples);
    }
    entries.push(f64('avoidance.inputs', avoidanceInputs), f64('avoidance.outputs', avoidanceOutputs));

    const crowd = blocks.crowd.create(0.6);
    const agentIds = [];
    const agentCount = Math.min(12, agentPoints.length / 2);
    for (let i = 0; i < agentCount; i++) {
        const id = blocks.crowd.addAgent(crowd, tiled.navMesh, [...agentPoints[i].position], {
            radius: 0.3,
            height: 1,
            maxAcceleration: 8,
            maxSpeed: 2,
            collisionQueryRange: 1.8,
            separationWeight: 0.5,
            updateFlags:
                blocks.crowd.CrowdUpdateFlags.ANTICIPATE_TURNS |
                blocks.crowd.CrowdUpdateFlags.OBSTACLE_AVOIDANCE |
                blocks.crowd.CrowdUpdateFlags.SEPARATION |
                blocks.crowd.CrowdUpdateFlags.OPTIMIZE_VIS |
                blocks.crowd.CrowdUpdateFlags.OPTIMIZE_TOPO,
            queryFilter: navcat.DEFAULT_QUERY_FILTER,
        });
        agentIds.push(id);
        const target = agentPoints[agentPoints.length - 1 - i];
        blocks.crowd.requestMoveTarget(crowd, id, target.ref, target.position);
    }
    const crowdFrames = [];
    const crowdStates = [];
    for (let frame = 1; frame <= 120; frame++) {
        blocks.crowd.update(crowd, tiled.navMesh, 1 / 60);
        {
            for (const id of agentIds) {
                const agent = crowd.agents[id];
                crowdFrames.push(...agent.position, ...agent.velocity);
                crowdStates.push(agent.state, agent.targetState, agent.corridor.path.length);
            }
        }
    }
    entries.push(i32('crowd.agent_count', agentIds.length), f64('crowd.frames', crowdFrames), i32('crowd.states', crowdStates));

    entries.push(...flatCrowdEntries(generatorOptions));

    writeFixture(name, entries);
    console.log(`wrote ${name}.bin (${entries.length} entries)`);
};

const boxMesh = (positions, indices, min, max) => {
    const base = positions.length / 3;
    for (const y of [min[1], max[1]]) {
        positions.push(min[0], y, min[2], max[0], y, min[2], max[0], y, max[2], min[0], y, max[2]);
    }
    const faces = [
        [4, 6, 5], [4, 7, 6],
        [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6],
        [3, 0, 4], [3, 4, 7],
    ];
    for (const face of faces) indices.push(base + face[0], base + face[1], base + face[2]);
};

const flatCrowdEntries = (generatorOptions) => {
    const positions = [-8, 0, -8, 8, 0, -8, 8, 0, 8, -8, 0, 8];
    const indices = [0, 2, 1, 0, 3, 2];
    boxMesh(positions, indices, [-3, 0, -1], [-1, 2, 1]);
    boxMesh(positions, indices, [1, 0, 2], [4, 2, 3]);
    boxMesh(positions, indices, [2, 0, -5], [3, 2, -2]);

    const tileSizeVoxels = 32;
    const options = {
        ...generatorOptions,
        cellSize: 0.2,
        cellHeight: 0.2,
        walkableRadiusVoxels: 2,
        walkableRadiusWorld: 0.4,
        walkableClimbVoxels: 2,
        walkableHeightVoxels: 10,
        walkableHeightWorld: 2,
        tileSizeVoxels,
        tileSizeWorld: tileSizeVoxels * 0.2,
        borderSize: 5,
        detailSampleDistance: 1.2,
        detailSampleMaxError: 0.2,
    };
    const tiled = blocks.generateTiledNavMesh({ positions: new Float32Array(positions), indices: new Uint32Array(indices) }, options);

    const crowd = blocks.crowd.create(0.6);
    const agentIds = [];
    const agentCount = 16;
    const halfExtents = [1, 1, 1];
    for (let i = 0; i < agentCount; i++) {
        const angle = (i / agentCount) * Math.PI * 2;
        const start = [Math.cos(angle) * 6.5, 0, Math.sin(angle) * 6.5];
        const end = [-start[0], 0, -start[2]];
        const id = blocks.crowd.addAgent(crowd, tiled.navMesh, start, {
            radius: 0.4,
            height: 2,
            maxAcceleration: 8,
            maxSpeed: 3,
            collisionQueryRange: 2.4,
            separationWeight: 1,
            updateFlags:
                blocks.crowd.CrowdUpdateFlags.ANTICIPATE_TURNS |
                blocks.crowd.CrowdUpdateFlags.OBSTACLE_AVOIDANCE |
                blocks.crowd.CrowdUpdateFlags.SEPARATION |
                blocks.crowd.CrowdUpdateFlags.OPTIMIZE_VIS |
                blocks.crowd.CrowdUpdateFlags.OPTIMIZE_TOPO,
            queryFilter: navcat.DEFAULT_QUERY_FILTER,
        });
        agentIds.push(id);
        const target = navcat.findNearestPoly(navcat.createFindNearestPolyResult(), tiled.navMesh, end, halfExtents, navcat.DEFAULT_QUERY_FILTER);
        blocks.crowd.requestMoveTarget(crowd, id, target.nodeRef, target.position);
    }

    const frames = [];
    const states = [];
    for (let frame = 1; frame <= 240; frame++) {
        blocks.crowd.update(crowd, tiled.navMesh, 1 / 60);
        for (const id of agentIds) {
            const agent = crowd.agents[id];
            frames.push(...agent.position, ...agent.velocity);
            states.push(agent.state, agent.targetState, agent.corridor.path.length);
        }
    }

    return [
        f64('flat.positions', positions),
        i32('flat.indices', indices),
        i32('flat.agent_count', agentCount),
        f64('flat.frames', frames),
        i32('flat.states', states),
    ];
};

dumpBlocks('nav_test_blocks', new Float32Array(navTest.positions), new Uint32Array(navTest.indices));
