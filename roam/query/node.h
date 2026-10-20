#pragma once

#include <stdint.h>

namespace roam {

enum NodeType : uint32_t {
    NODE_TYPE_POLY = 0,
    NODE_TYPE_OFFMESH = 1,
};

using NodeRef = uint64_t;

constexpr NodeRef INVALID_NODE_REF = ~0ull;

namespace node {

constexpr uint32_t TYPE_BITS = 1;
constexpr uint32_t NODE_INDEX_BITS = 31;
constexpr uint32_t SEQUENCE_BITS = 20;

constexpr uint64_t TYPE_MASK = 0x1;
constexpr uint64_t NODE_INDEX_MASK = 0x7fffffff;
constexpr uint64_t SEQUENCE_MASK = (1ull << SEQUENCE_BITS) - 1;

constexpr uint32_t NODE_INDEX_SHIFT = TYPE_BITS;
constexpr uint32_t SEQUENCE_SHIFT = TYPE_BITS + NODE_INDEX_BITS;

constexpr uint32_t MAX_NODE_INDEX = (uint32_t)NODE_INDEX_MASK;
constexpr uint32_t MAX_SEQUENCE = (uint32_t)SEQUENCE_MASK;

inline NodeRef ser_node_ref(NodeType type, uint32_t node_index, uint32_t sequence) {
    return ((uint64_t)type & TYPE_MASK) | (((uint64_t)node_index & NODE_INDEX_MASK) << NODE_INDEX_SHIFT) |
           (((uint64_t)sequence & SEQUENCE_MASK) << SEQUENCE_SHIFT);
}

inline NodeType get_node_ref_type(NodeRef ref) { return (NodeType)(ref & TYPE_MASK); }

inline uint32_t get_node_ref_index(NodeRef ref) { return (uint32_t)((ref >> NODE_INDEX_SHIFT) & NODE_INDEX_MASK); }

inline uint32_t get_node_ref_sequence(NodeRef ref) { return (uint32_t)((ref >> SEQUENCE_SHIFT) & SEQUENCE_MASK); }

}

}
