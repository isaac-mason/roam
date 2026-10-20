#include "roam/query/node.h"
#include "tests/harness/test.h"

using namespace roam;

static void check_roundtrip(NodeType type, uint32_t node_index, uint32_t sequence) {
    NodeRef ref = node::ser_node_ref(type, node_index, sequence);
    CHECK(node::get_node_ref_type(ref) == type);
    CHECK(node::get_node_ref_index(ref) == node_index);
    CHECK(node::get_node_ref_sequence(ref) == sequence);
}

TEST(node_ref_roundtrips_basic_values) { check_roundtrip(NODE_TYPE_POLY, 100, 42); }

TEST(node_ref_roundtrips_max_values) { check_roundtrip(NODE_TYPE_OFFMESH, node::MAX_NODE_INDEX, node::MAX_SEQUENCE); }

TEST(node_ref_roundtrips_various_poly_values) {
    check_roundtrip(NODE_TYPE_POLY, 0, 0);
    check_roundtrip(NODE_TYPE_POLY, 1, 1);
    check_roundtrip(NODE_TYPE_POLY, 12345, 50);
    check_roundtrip(NODE_TYPE_POLY, 1000000, 100000);
    check_roundtrip(NODE_TYPE_POLY, node::MAX_NODE_INDEX, node::MAX_SEQUENCE);
    check_roundtrip(NODE_TYPE_POLY, node::MAX_NODE_INDEX >> 1, node::MAX_SEQUENCE >> 1);
}

TEST(node_ref_roundtrips_various_offmesh_values) {
    check_roundtrip(NODE_TYPE_OFFMESH, 0, 0);
    check_roundtrip(NODE_TYPE_OFFMESH, 1, 1);
    check_roundtrip(NODE_TYPE_OFFMESH, 12345, 50);
    check_roundtrip(NODE_TYPE_OFFMESH, 1000000, 100000);
}

TEST(node_ref_type_bit_is_isolated) {
    NodeRef poly = node::ser_node_ref(NODE_TYPE_POLY, 12345, 678);
    NodeRef offmesh = node::ser_node_ref(NODE_TYPE_OFFMESH, 12345, 678);
    CHECK(node::get_node_ref_index(poly) == 12345);
    CHECK(node::get_node_ref_index(offmesh) == 12345);
    CHECK(node::get_node_ref_sequence(poly) == 678);
    CHECK(node::get_node_ref_sequence(offmesh) == 678);
    CHECK(node::get_node_ref_type(poly) == NODE_TYPE_POLY);
    CHECK(node::get_node_ref_type(offmesh) == NODE_TYPE_OFFMESH);
}

TEST(node_ref_invalid_is_distinct_from_valid_refs) {
    CHECK(node::ser_node_ref(NODE_TYPE_POLY, 0, 0) != INVALID_NODE_REF);
    CHECK(node::ser_node_ref(NODE_TYPE_POLY, 1, 1) != INVALID_NODE_REF);
    CHECK(node::ser_node_ref(NODE_TYPE_POLY, node::MAX_NODE_INDEX, node::MAX_SEQUENCE) != INVALID_NODE_REF);
    CHECK(node::ser_node_ref(NODE_TYPE_OFFMESH, 0, 0) != INVALID_NODE_REF);
    CHECK(node::ser_node_ref(NODE_TYPE_OFFMESH, node::MAX_NODE_INDEX, node::MAX_SEQUENCE) != INVALID_NODE_REF);
}

TEST(node_ref_invalid_deserializes_to_max_fields) {
    CHECK(node::get_node_ref_type(INVALID_NODE_REF) == NODE_TYPE_OFFMESH);
    CHECK(node::get_node_ref_index(INVALID_NODE_REF) == 2147483647u);
    CHECK(node::get_node_ref_sequence(INVALID_NODE_REF) == 1048575u);
}
