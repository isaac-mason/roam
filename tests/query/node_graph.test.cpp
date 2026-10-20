#include "roam/query/nav_mesh_api.h"
#include "tests/harness/test.h"
#include "tests/query/test_nav_meshes.h"

using namespace roam;

static uint32_t allocated_node_count(const NavMesh &nav_mesh, int type) {
    uint32_t count = 0;
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated && (type < 0 || (int)node.type == type)) count++;
    }
    return count;
}

static uint32_t link_count(const NavMesh &nav_mesh, const NavMeshNode &node) {
    uint32_t count = 0;
    for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) count++;
    return count;
}

static uint32_t links_to(const NavMesh &nav_mesh, const NavMeshNode &node, NodeRef to) {
    uint32_t count = 0;
    for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
        if (nav_mesh.links[(uint32_t)link].to_node_ref == to) count++;
    }
    return count;
}

static uint32_t links_to_off_mesh_nodes(const NavMesh &nav_mesh, const NavMeshNode &node) {
    uint32_t count = 0;
    for (int32_t link = node.first_link; link != NULL_LINK; link = nav_mesh.links[(uint32_t)link].next) {
        if (node::get_node_ref_type(nav_mesh.links[(uint32_t)link].to_node_ref) == NODE_TYPE_OFFMESH) count++;
    }
    return count;
}

TEST(node_graph_tile_polys) {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3};

    NavMesh nav_mesh = nav_mesh_api::create_nav_mesh();
    CHECK(nav_mesh.nodes.length == 0);
    CHECK(nav_mesh.links.length == 0);
    nav_mesh_api::release_nav_mesh(nav_mesh);

    nav_mesh = create_nav_mesh_from_triangles(positions, 4, indices, 6);

    REQUIRE(allocated_node_count(nav_mesh, -1) == 2);

    NodeRef refs[2];
    uint32_t ref_count = 0;
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (!node.allocated) continue;
        refs[ref_count++] = node.ref;
        CHECK(nav_mesh_api::is_valid_node_ref(nav_mesh, node.ref));
        CHECK(link_count(nav_mesh, node) == 1);
        const NavMeshLink &link = nav_mesh.links[(uint32_t)node.first_link];
        CHECK(node::get_node_ref_type(link.to_node_ref) == NODE_TYPE_POLY);
        CHECK(nav_mesh_api::is_valid_node_ref(nav_mesh, link.to_node_ref));
    }

    nav_mesh_api::remove_tile(nav_mesh, 0, 0, 0);

    CHECK(allocated_node_count(nav_mesh, -1) == 0);
    for (uint32_t i = 0; i < ref_count; i++) CHECK(!nav_mesh_api::is_valid_node_ref(nav_mesh, refs[i]));
    CHECK(nav_mesh.nodes.length == 2);

    uint32_t allocated_links = 0;
    for (const NavMeshLink &link : nav_mesh.links) allocated_links += link.allocated ? 1 : 0;
    CHECK(allocated_links == 0);
    CHECK(nav_mesh.links.length == 2);

    nav_mesh_api::release_nav_mesh(nav_mesh);
}

static NavMesh create_off_mesh_test_nav_mesh() {
    const float positions[] = {0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2, 7, 0, 0, 9, 0, 0, 9, 0, 2, 7, 0, 2};
    const uint32_t indices[] = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    return create_nav_mesh_from_triangles(positions, 8, indices, 12);
}

static const NavMeshNode *find_node(const NavMesh &nav_mesh, NodeRef ref) {
    for (const NavMeshNode &node : nav_mesh.nodes) {
        if (node.allocated && node.ref == ref) return &node;
    }
    return nullptr;
}

TEST(node_graph_bidirectional_off_mesh_connection) {
    NavMesh nav_mesh = create_off_mesh_test_nav_mesh();
    CHECK(allocated_node_count(nav_mesh, -1) == 4);
    CHECK(nav_mesh.off_mesh_connections.length == 0);

    uint32_t id = nav_mesh_api::add_off_mesh_connection(
        nav_mesh, {{1, 0, 1}, {8, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_BIDIRECTIONAL, 1, 0});

    REQUIRE(nav_mesh.off_mesh_connections[id].allocated);
    REQUIRE(nav_mesh.off_mesh_connection_attachments[id].attached);
    OffMeshConnectionAttachment attachment = nav_mesh.off_mesh_connection_attachments[id];

    CHECK(allocated_node_count(nav_mesh, -1) == 5);
    REQUIRE(allocated_node_count(nav_mesh, NODE_TYPE_OFFMESH) == 1);

    const NavMeshNode *off_mesh_node = find_node(nav_mesh, attachment.off_mesh_node);
    REQUIRE(off_mesh_node != nullptr);
    CHECK(off_mesh_node->off_mesh_connection_id == (int32_t)id);
    NodeRef off_mesh_node_ref = off_mesh_node->ref;

    const NavMeshNode *start_poly_node = find_node(nav_mesh, attachment.start_poly_node);
    REQUIRE(start_poly_node != nullptr);
    CHECK(links_to(nav_mesh, *start_poly_node, off_mesh_node_ref) == 1);

    CHECK(link_count(nav_mesh, *off_mesh_node) == 2);
    CHECK(links_to(nav_mesh, *off_mesh_node, attachment.end_poly_node) == 1);
    CHECK(links_to(nav_mesh, *off_mesh_node, attachment.start_poly_node) == 1);

    const NavMeshNode *end_poly_node = find_node(nav_mesh, attachment.end_poly_node);
    REQUIRE(end_poly_node != nullptr);
    CHECK(links_to(nav_mesh, *end_poly_node, off_mesh_node_ref) == 1);

    nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);

    CHECK(!nav_mesh.off_mesh_connections[id].allocated);
    CHECK(allocated_node_count(nav_mesh, -1) == 4);
    CHECK(allocated_node_count(nav_mesh, NODE_TYPE_OFFMESH) == 0);
    CHECK(!nav_mesh_api::is_valid_node_ref(nav_mesh, off_mesh_node_ref));

    nav_mesh_api::release_nav_mesh(nav_mesh);
}

TEST(node_graph_one_way_off_mesh_connection) {
    NavMesh nav_mesh = create_off_mesh_test_nav_mesh();
    CHECK(allocated_node_count(nav_mesh, -1) == 4);

    uint32_t id =
        nav_mesh_api::add_off_mesh_connection(nav_mesh, {{1, 0, 1}, {8, 0, 1}, 0.5f, OFF_MESH_CONNECTION_DIRECTION_START_TO_END, 1, 0});

    REQUIRE(nav_mesh.off_mesh_connections[id].allocated);
    REQUIRE(nav_mesh.off_mesh_connection_attachments[id].attached);
    OffMeshConnectionAttachment attachment = nav_mesh.off_mesh_connection_attachments[id];

    CHECK(allocated_node_count(nav_mesh, -1) == 5);
    REQUIRE(allocated_node_count(nav_mesh, NODE_TYPE_OFFMESH) == 1);

    const NavMeshNode *off_mesh_node = find_node(nav_mesh, attachment.off_mesh_node);
    REQUIRE(off_mesh_node != nullptr);
    NodeRef off_mesh_node_ref = off_mesh_node->ref;

    const NavMeshNode *start_poly_node = find_node(nav_mesh, attachment.start_poly_node);
    REQUIRE(start_poly_node != nullptr);
    CHECK(links_to(nav_mesh, *start_poly_node, off_mesh_node_ref) == 1);

    REQUIRE(link_count(nav_mesh, *off_mesh_node) == 1);
    CHECK(nav_mesh.links[(uint32_t)off_mesh_node->first_link].to_node_ref == attachment.end_poly_node);

    const NavMeshNode *end_poly_node = find_node(nav_mesh, attachment.end_poly_node);
    REQUIRE(end_poly_node != nullptr);
    CHECK(links_to_off_mesh_nodes(nav_mesh, *end_poly_node) == 0);

    nav_mesh_api::remove_off_mesh_connection(nav_mesh, id);

    CHECK(!nav_mesh.off_mesh_connections[id].allocated);
    CHECK(allocated_node_count(nav_mesh, -1) == 4);
    CHECK(allocated_node_count(nav_mesh, NODE_TYPE_OFFMESH) == 0);
    CHECK(!nav_mesh_api::is_valid_node_ref(nav_mesh, off_mesh_node_ref));

    nav_mesh_api::release_nav_mesh(nav_mesh);
}
