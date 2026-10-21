import json
import math
import struct
import sys

COMPONENT_FORMATS = {5120: 'b', 5121: 'B', 5122: 'h', 5123: 'H', 5125: 'I', 5126: 'f'}
TYPE_COUNTS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def read_glb(path):
    data = open(path, 'rb').read()
    magic, _, _ = struct.unpack_from('<III', data, 0)
    if magic != 0x46546C67:
        raise SystemExit(f'{path} is not a .glb')
    offset = 12
    gltf = None
    binary = b''
    while offset < len(data):
        length, kind = struct.unpack_from('<II', data, offset)
        chunk = data[offset + 8 : offset + 8 + length]
        if kind == 0x4E4F534A:
            gltf = json.loads(chunk)
        elif kind == 0x004E4942:
            binary = chunk
        offset += 8 + length
    return gltf, binary


def read_accessor(gltf, binary, index):
    accessor = gltf['accessors'][index]
    view = gltf['bufferViews'][accessor['bufferView']]
    fmt = COMPONENT_FORMATS[accessor['componentType']]
    components = TYPE_COUNTS[accessor['type']]
    size = struct.calcsize(fmt)
    stride = view.get('byteStride', size * components)
    start = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
    values = []
    for i in range(accessor['count']):
        values.append(struct.unpack_from('<' + fmt * components, binary, start + i * stride))
    return values


def multiply(a, b):
    return [sum(a[k * 4 + row] * b[column * 4 + k] for k in range(4)) for column in range(4) for row in range(4)]


def local_matrix(node):
    if 'matrix' in node:
        return list(node['matrix'])
    tx, ty, tz = node.get('translation', [0, 0, 0])
    qx, qy, qz, qw = node.get('rotation', [0, 0, 0, 1])
    sx, sy, sz = node.get('scale', [1, 1, 1])
    return [
        (1 - 2 * (qy * qy + qz * qz)) * sx, (2 * (qx * qy + qz * qw)) * sx, (2 * (qx * qz - qy * qw)) * sx, 0,
        (2 * (qx * qy - qz * qw)) * sy, (1 - 2 * (qx * qx + qz * qz)) * sy, (2 * (qy * qz + qx * qw)) * sy, 0,
        (2 * (qx * qz + qy * qw)) * sz, (2 * (qy * qz - qx * qw)) * sz, (1 - 2 * (qx * qx + qy * qy)) * sz, 0,
        tx, ty, tz, 1,
    ]


def float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def main():
    if len(sys.argv) != 3:
        raise SystemExit('usage: python3 glb_to_fixture.py <model.glb> <fixture.json>')
    gltf, binary = read_glb(sys.argv[1])

    positions = []
    indices = []
    index_of = {}

    def append_node(node_index, parent):
        node = gltf['nodes'][node_index]
        world = multiply(parent, local_matrix(node))
        if 'mesh' in node:
            for primitive in gltf['meshes'][node['mesh']]['primitives']:
                if primitive.get('mode', 4) != 4 or 'POSITION' not in primitive['attributes']:
                    continue
                local = read_accessor(gltf, binary, primitive['attributes']['POSITION'])
                vertex_indices = [v[0] for v in read_accessor(gltf, binary, primitive['indices'])] if 'indices' in primitive else range(len(local))
                for vertex in vertex_indices:
                    x, y, z = local[vertex]
                    point = tuple(
                        float32(world[row] * x + world[4 + row] * y + world[8 + row] * z + world[12 + row]) + 0.0 for row in range(3)
                    )
                    if point not in index_of:
                        index_of[point] = len(positions) // 3
                        positions.extend(point)
                    indices.append(index_of[point])
        for child in node.get('children', []):
            append_node(child, world)

    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    scene = gltf['scenes'][gltf.get('scene', 0)]
    for node_index in scene['nodes']:
        append_node(node_index, identity)

    if any(not math.isfinite(value) for value in positions):
        raise SystemExit('non-finite positions')

    with open(sys.argv[2], 'w') as out:
        json.dump({'source': sys.argv[1], 'vertexCount': len(positions) // 3, 'triangleCount': len(indices) // 3, 'positions': positions, 'indices': indices}, out)
    print(f'{sys.argv[2]}: {len(positions) // 3} vertices, {len(indices) // 3} triangles')


main()
