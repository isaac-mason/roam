#include "roam/query/bv_tree.h"
#include "tests/harness/test.h"

using namespace roam;

static bool overlaps(const int *bmin, const int *bmax, const NavMeshBvNode &node) {
    return bmin[0] <= node.bmax[0] && bmax[0] >= node.bmin[0] && bmin[1] <= node.bmax[1] && bmax[1] >= node.bmin[1] && bmin[2] <= node.bmax[2] &&
           bmax[2] >= node.bmin[2];
}

TEST_CASE("bv tree traversal matches brute force over leaves") {
    NavMeshTileParams params = {};
    Array<float> vertices = {};
    Array<NavMeshPoly> polys = {};
    Array<uint32_t> poly_vertices = {};
    params.cell_size = 1;
    params.bounds = {{0, 0, 0}, {64, 4, 64}};
    uint32_t state = 7;
    auto next = [&]() {
        state = state * 1664525u + 1013904223u;
        return (float)(state >> 8) / 16777216.0f;
    };
    for (uint32_t i = 0; i < 300; i++) {
        float x = next() * 60;
        float z = next() * 60;
        uint32_t base = vertices.length / 3;
        vertices.push(x);
        vertices.push(next() * 3);
        vertices.push(z);
        vertices.push(x + 1 + next() * 3);
        vertices.push(next() * 3);
        vertices.push(z);
        vertices.push(x);
        vertices.push(next() * 3);
        vertices.push(z + 1 + next() * 3);
        polys.push({poly_vertices.length, 3, 1, 0});
        for (uint32_t k = 0; k < 3; k++) poly_vertices.push(base + k);
    }

    params.vertices = vertices;
    params.polys = polys;
    params.poly_vertices = poly_vertices;
    NavMeshTileBvTree tree = bv_tree::build_nav_mesh_bv_tree(params);

    uint32_t mismatches = 0;
    for (int query = 0; query < 200; query++) {
        int x = (int)(next() * 60);
        int z = (int)(next() * 60);
        int bmin[3] = {x, 0, z};
        int bmax[3] = {x + (int)(next() * 8), 4, z + (int)(next() * 8)};

        int found = 0;
        uint32_t node_index = 0;
        while (node_index < tree.nodes.length) {
            const NavMeshBvNode &node = tree.nodes[node_index];
            bool overlap = overlaps(bmin, bmax, node);
            bool leaf = node.i >= 0;
            if (leaf && overlap) found++;
            if (overlap || leaf) {
                node_index++;
            } else {
                node_index += (uint32_t)(-node.i);
            }
        }

        int expected = 0;
        for (const NavMeshBvNode &node : tree.nodes) {
            if (node.i >= 0 && overlaps(bmin, bmax, node)) expected++;
        }
        if (found != expected) mismatches++;
    }
    CHECK(mismatches == 0);

    for (const NavMeshBvNode &node : tree.nodes) {
        for (int axis = 0; axis < 3; axis++) CHECK(node.bmin[axis] <= node.bmax[axis]);
    }

    tree.nodes.release();
    vertices.release();
    polys.release();
    poly_vertices.release();
}

TEST_CASE("bv tree snaps near integers before truncating") {
    NavMeshTileParams params = {};
    Array<NavMeshPoly> polys = {};
    Array<uint32_t> poly_vertices = {};
    params.cell_size = 0.1f;
    params.bounds = {{0, 0, 0}, {10, 10, 10}};
    const float vertices[] = {0.3f, 0, 0.3f, 2.4f, 0, 0.3f, 2.4f, 0, 2.39999f};
    polys.push({0, 3, 1, 0});
    for (uint32_t k = 0; k < 3; k++) poly_vertices.push(k);

    params.vertices = vertices;
    params.polys = polys;
    params.poly_vertices = poly_vertices;
    NavMeshTileBvTree tree = bv_tree::build_nav_mesh_bv_tree(params);
    REQUIRE(tree.nodes.length == 1);
    CHECK(tree.nodes[0].bmin[0] == 3);
    CHECK(tree.nodes[0].bmax[0] == 24);
    CHECK(tree.nodes[0].bmax[2] == 24);

    tree.nodes.release();
    polys.release();
    poly_vertices.release();
}
