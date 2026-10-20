import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { parseArgs } from 'node:util';

const { values: args } = parseArgs({
    options: {
        navcat: { type: 'string' },
        roam: { type: 'string' },
        filter: { type: 'string' },
    },
});

if (!args.navcat) {
    console.error('usage: node bench.mjs --navcat <navcat checkout> [--roam <roam_bench binary>] [--filter <name>]');
    process.exit(1);
}

const navcatDir = resolve(args.navcat);
const fixturePath = join(navcatDir, 'bench/fixtures/nav-test.json');
const navcat = await import(join(navcatDir, 'dist/index.js'));
const blocks = await import(join(navcatDir, 'dist/blocks.js'));

const fixture = JSON.parse(readFileSync(fixturePath, 'utf8'));
const input = { positions: new Float32Array(fixture.positions), indices: new Uint32Array(fixture.indices) };

const cellSize = 0.15;
const cellHeight = 0.15;
const options = {
    cellSize,
    cellHeight,
    walkableRadiusWorld: 0.1,
    walkableRadiusVoxels: Math.ceil(0.1 / cellSize),
    walkableClimbWorld: 0.5,
    walkableClimbVoxels: Math.ceil(0.5 / cellHeight),
    walkableHeightWorld: 0.25,
    walkableHeightVoxels: Math.ceil(0.25 / cellHeight),
    walkableSlopeAngleDegrees: 45,
    borderSize: 0,
    minRegionArea: 8,
    mergeRegionArea: 20,
    maxSimplificationError: 1.3,
    maxEdgeLength: 12,
    maxVerticesPerPoly: 5,
    detailSampleDistance: cellSize * 6,
    detailSampleMaxError: cellHeight * 1,
};
const tiledOptions = {
    ...options,
    tileSizeVoxels: 32,
    tileSizeWorld: 32 * cellSize,
    borderSize: options.walkableRadiusVoxels + 3,
};

const STAGES = [
    'mark walkable triangles',
    'rasterize triangles',
    'filter heightfield',
    'build compact heightfield',
    'erode walkable area',
    'build distance field',
    'build regions',
    'build contours',
    'build poly mesh',
    'build poly mesh detail',
];

const advance = (pipeline, stage) => {
    const { positions, indices } = input;
    switch (stage) {
        case 0:
            pipeline.triAreaIds = new Uint8Array(indices.length / 3);
            navcat.markWalkableTriangles(positions, indices, pipeline.triAreaIds, options.walkableSlopeAngleDegrees);
            break;
        case 1: {
            const bounds = navcat.calculateMeshBounds([0, 0, 0, 0, 0, 0], positions, indices);
            const [width, height] = navcat.calculateGridSize([0, 0], bounds, cellSize);
            pipeline.heightfield = navcat.createHeightfield(width, height, bounds, cellSize, cellHeight);
            navcat.rasterizeTriangles(pipeline.ctx, pipeline.heightfield, positions, indices, pipeline.triAreaIds, options.walkableClimbVoxels);
            break;
        }
        case 2:
            navcat.filterLowHangingWalkableObstacles(pipeline.heightfield, options.walkableClimbVoxels);
            navcat.filterLedgeSpans(pipeline.heightfield, options.walkableHeightVoxels, options.walkableClimbVoxels);
            navcat.filterWalkableLowHeightSpans(pipeline.heightfield, options.walkableHeightVoxels);
            break;
        case 3:
            pipeline.compactHeightfield = navcat.buildCompactHeightfield(pipeline.ctx, options.walkableHeightVoxels, options.walkableClimbVoxels, pipeline.heightfield);
            break;
        case 4:
            navcat.erodeWalkableArea(options.walkableRadiusVoxels, pipeline.compactHeightfield);
            break;
        case 5:
            navcat.buildDistanceField(pipeline.compactHeightfield);
            break;
        case 6:
            navcat.buildRegions(pipeline.ctx, pipeline.compactHeightfield, options.borderSize, options.minRegionArea, options.mergeRegionArea);
            break;
        case 7:
            pipeline.contourSet = navcat.buildContours(
                pipeline.ctx,
                pipeline.compactHeightfield,
                options.maxSimplificationError,
                options.maxEdgeLength,
                navcat.ContourBuildFlags.CONTOUR_TESS_WALL_EDGES,
            );
            break;
        case 8:
            pipeline.polyMesh = navcat.buildPolyMesh(pipeline.ctx, pipeline.contourSet, options.maxVerticesPerPoly);
            break;
        case 9:
            pipeline.polyMeshDetail = navcat.buildPolyMeshDetail(
                pipeline.ctx,
                pipeline.polyMesh,
                pipeline.compactHeightfield,
                options.detailSampleDistance,
                options.detailSampleMaxError,
            );
            break;
    }
};

const navMesh = blocks.generateSoloNavMesh(input, options).navMesh;
const tile = Object.values(navMesh.tiles)[0];
const centers = [];
const centerRefs = [];
for (const poly of tile.polys) {
    const center = [0, 0, 0];
    for (const vertex of poly.vertices) {
        center[0] += tile.vertices[vertex * 3];
        center[1] += tile.vertices[vertex * 3 + 1];
        center[2] += tile.vertices[vertex * 3 + 2];
    }
    for (let i = 0; i < 3; i++) center[i] = Math.fround(center[i] / poly.vertices.length);
    const nearest = navcat.findNearestPoly(navcat.createFindNearestPolyResult(), navMesh, center, [1, 1, 1], navcat.DEFAULT_QUERY_FILTER);
    if (!nearest.success) continue;
    centers.push([...nearest.position]);
    centerRefs.push(nearest.nodeRef);
}

const QUERY_COUNT = 100;
const pairEnd = (i) => (i * 37 + 11) % centers.length;

const cases = [];
for (let stage = 0; stage < STAGES.length; stage++) {
    let pipeline = null;
    cases.push({
        name: STAGES[stage],
        prepare: () => {
            pipeline = { ctx: navcat.BuildContext.create() };
            for (let i = 0; i < stage; i++) advance(pipeline, i);
        },
        run: () => advance(pipeline, stage),
    });
}
cases.push({ name: 'generate solo nav mesh', run: () => blocks.generateSoloNavMesh(input, options) });
cases.push({ name: 'generate tiled nav mesh', run: () => blocks.generateTiledNavMesh(input, tiledOptions) });
cases.push({
    name: 'find path (100 pairs)',
    run: () => {
        for (let i = 0; i < QUERY_COUNT; i++) {
            navcat.findPath(navMesh, centers[i % centers.length], centers[pairEnd(i)], [1, 1, 1], navcat.DEFAULT_QUERY_FILTER);
        }
    },
});
cases.push({
    name: 'raycast (100 rays)',
    run: () => {
        for (let i = 0; i < QUERY_COUNT; i++) {
            const a = i % centers.length;
            navcat.raycast(navMesh, centerRefs[a], centers[a], centers[pairEnd(i)], navcat.DEFAULT_QUERY_FILTER);
        }
    },
});
const nearestResult = navcat.createFindNearestPolyResult();
cases.push({
    name: 'find nearest poly (200 points)',
    run: () => {
        for (let i = 0; i < 2 * QUERY_COUNT; i++) {
            const center = centers[i % centers.length];
            const point = [Math.fround(center[0] + 0.37), Math.fround(center[1] + 0.5), Math.fround(center[2] - 0.21)];
            navcat.findNearestPoly(nearestResult, navMesh, point, [1, 2, 1], navcat.DEFAULT_QUERY_FILTER);
        }
    },
});
let crowd = null;
cases.push({
    name: 'crowd (100 agents, 60 updates)',
    prepare: () => {
        crowd = blocks.crowd.create(0.6);
        const flags = blocks.crowd.CrowdUpdateFlags;
        const params = {
            radius: 0.3,
            height: 0.6,
            maxAcceleration: 15,
            maxSpeed: 3.5,
            collisionQueryRange: 2,
            separationWeight: 0.5,
            updateFlags: flags.ANTICIPATE_TURNS | flags.SEPARATION | flags.OBSTACLE_AVOIDANCE | flags.OPTIMIZE_TOPO | flags.OPTIMIZE_VIS,
            queryFilter: navcat.DEFAULT_QUERY_FILTER,
        };
        for (let i = 0; i < 100; i++) {
            const id = blocks.crowd.addAgent(crowd, navMesh, [...centers[i % centers.length]], params);
            const target = (i * 7 + 3) % centers.length;
            blocks.crowd.requestMoveTarget(crowd, id, centerRefs[target], centers[target]);
        }
    },
    run: () => {
        for (let frame = 0; frame < 60; frame++) blocks.crowd.update(crowd, navMesh, 1 / 60);
    },
});

const sample = (benchCase) => {
    if (benchCase.prepare) benchCase.prepare();
    const start = performance.now();
    benchCase.run();
    return performance.now() - start;
};

const runCase = (benchCase) => {
    for (let i = 0; i < 3; i++) sample(benchCase);
    const times = [];
    let total = 0;
    while (times.length < 2000 && (times.length < 10 || total < 500)) {
        const elapsed = sample(benchCase);
        times.push(elapsed);
        total += elapsed;
    }
    times.sort((a, b) => a - b);
    return { median: times[Math.floor(times.length / 2)], min: times[0], samples: times.length };
};

const parseRoam = (output) => {
    const results = new Map();
    for (const line of output.split('\n').slice(1)) {
        const match = line.match(/^(.*?)\s+([\d.]+)\s+([\d.]+)\s+(\d+)\s*$/);
        if (match) results.set(match[1].trim(), { median: Number(match[2]), min: Number(match[3]), samples: Number(match[4]) });
    }
    return results;
};

let roamResults = null;
if (args.roam) {
    const roamArgs = ['--fixture', fixturePath];
    if (args.filter) roamArgs.push(args.filter);
    roamResults = parseRoam(execFileSync(resolve(args.roam), roamArgs, { encoding: 'utf8' }));
}

const rows = [];
for (const benchCase of cases) {
    if (args.filter && !benchCase.name.includes(args.filter)) continue;
    const result = runCase(benchCase);
    const roam = roamResults?.get(benchCase.name);
    rows.push({ name: benchCase.name, navcat: result, roam });
    if (!roamResults) console.log(`${benchCase.name.padEnd(34)} ${result.median.toFixed(4).padStart(12)} ${result.min.toFixed(4).padStart(12)} ${String(result.samples).padStart(8)}`);
}

if (roamResults) {
    console.log('| bench | navcat median ms | roam median ms | speedup |');
    console.log('|---|---:|---:|---:|');
    for (const row of rows) {
        const roam = row.roam ? row.roam.median.toFixed(4) : '-';
        const speedup = row.roam ? `${(row.navcat.median / row.roam.median).toFixed(1)}x` : '-';
        console.log(`| ${row.name} | ${row.navcat.median.toFixed(4)} | ${roam} | ${speedup} |`);
    }
}
