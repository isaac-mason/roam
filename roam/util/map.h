#pragma once

#include <stdint.h>
#include <string.h>

#include "roam/util/assert.h"
#include "roam/util/memory.h"

namespace roam {

enum MapSlot : uint8_t {
    MAP_SLOT_EMPTY = 0,
    MAP_SLOT_OCCUPIED = 1,
    MAP_SLOT_REMOVED = 2,
};

inline uint64_t hash_key(uint64_t key) {
    key ^= key >> 30;
    key *= 0xbf58476d1ce4e5b9ull;
    key ^= key >> 27;
    key *= 0x94d049bb133111ebull;
    key ^= key >> 31;
    return key;
}

template <typename K, typename V>
struct Map {
    static_assert(__is_trivially_copyable(V));

    K *keys;
    V *values;
    uint8_t *slots;
    uint32_t length;
    uint32_t removed;
    uint32_t capacity;

    uint32_t find_slot(K key) const {
        uint32_t mask = capacity - 1;
        uint32_t index = (uint32_t)hash_key((uint64_t)key) & mask;
        while (true) {
            if (slots[index] == MAP_SLOT_EMPTY) return UINT32_MAX;
            if (slots[index] == MAP_SLOT_OCCUPIED && keys[index] == key) return index;
            index = (index + 1) & mask;
        }
    }

    V *get(K key) {
        if (length == 0) return nullptr;
        uint32_t index = find_slot(key);
        return index == UINT32_MAX ? nullptr : &values[index];
    }

    const V *get(K key) const {
        if (length == 0) return nullptr;
        uint32_t index = find_slot(key);
        return index == UINT32_MAX ? nullptr : &values[index];
    }

    bool has(K key) const { return get(key) != nullptr; }

    void rehash(uint32_t new_capacity) {
        K *old_keys = keys;
        V *old_values = values;
        uint8_t *old_slots = slots;
        uint32_t old_capacity = capacity;

        keys = memory::allocate_array<K>(new_capacity);
        values = memory::allocate_array<V>(new_capacity);
        slots = memory::allocate_array<uint8_t>(new_capacity);
        memset(slots, MAP_SLOT_EMPTY, new_capacity);
        capacity = new_capacity;
        length = 0;
        removed = 0;

        for (uint32_t i = 0; i < old_capacity; i++) {
            if (old_slots[i] == MAP_SLOT_OCCUPIED) set(old_keys[i], old_values[i]);
        }

        memory::free_array(old_keys, old_capacity);
        memory::free_array(old_values, old_capacity);
        memory::free_array(old_slots, old_capacity);
    }

    V &set(K key, const V &value) {
        if ((length + removed + 1) * 4 > capacity * 3) {
            uint32_t grown = capacity == 0 ? 16 : (length + 1) * 4 > capacity * 2 ? capacity * 2 : capacity;
            V copy = value;
            rehash(grown);
            return set(key, copy);
        }

        uint32_t mask = capacity - 1;
        uint32_t index = (uint32_t)hash_key((uint64_t)key) & mask;
        uint32_t first_removed = UINT32_MAX;
        while (slots[index] != MAP_SLOT_EMPTY) {
            if (slots[index] == MAP_SLOT_OCCUPIED && keys[index] == key) {
                values[index] = value;
                return values[index];
            }
            if (slots[index] == MAP_SLOT_REMOVED && first_removed == UINT32_MAX) first_removed = index;
            index = (index + 1) & mask;
        }

        if (first_removed != UINT32_MAX) {
            index = first_removed;
            removed--;
        }

        slots[index] = MAP_SLOT_OCCUPIED;
        keys[index] = key;
        values[index] = value;
        length++;
        return values[index];
    }

    bool remove(K key) {
        if (length == 0) return false;
        uint32_t index = find_slot(key);
        if (index == UINT32_MAX) return false;
        slots[index] = MAP_SLOT_REMOVED;
        length--;
        removed++;
        return true;
    }

    void clear() {
        if (capacity > 0) memset(slots, MAP_SLOT_EMPTY, capacity);
        length = 0;
        removed = 0;
    }

    void release() {
        memory::free_array(keys, capacity);
        memory::free_array(values, capacity);
        memory::free_array(slots, capacity);
        keys = nullptr;
        values = nullptr;
        slots = nullptr;
        length = 0;
        removed = 0;
        capacity = 0;
    }
};

}
