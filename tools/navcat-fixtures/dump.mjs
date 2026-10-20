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

const config = {
    cellSize: 0.15,
    cellHeight: 0.15,
    walkableRadiusWorld: 0.1,
    walkableClimbWorld: 0.5,
    walkableHeightWorld: 0.25,
    walkableSlopeAngleDegrees: 45,
};

const options = {
    ...config,
    walkableRadiusVoxels: Math.ceil(config.walkableRadiusWorld / config.cellSize),
    walkableClimbVoxels: Math.ceil(config.walkableClimbWorld / config.cellHeight),
    walkableHeightVoxels: Math.ceil(config.walkableHeightWorld / config.cellHeight),
};

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

    writeFixture(name, entries);
    console.log(`wrote ${name}.bin (${entries.length} entries)`);
};

const navTest = JSON.parse(readFileSync(join(navcatDir, 'bench/fixtures/nav-test.json'), 'utf8'));
dumpSoloGeneration('nav_test_solo', new Float32Array(navTest.positions), new Uint32Array(navTest.indices));
