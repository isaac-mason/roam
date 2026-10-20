#include "tests/harness/fixture.h"

#include <stdio.h>
#include <string.h>

#include "tests/harness/test.h"

namespace fixture {

static bool read_u32(FILE *file, uint32_t &value) { return fread(&value, sizeof(value), 1, file) == 1; }

static uint32_t value_size(ValueType type) { return type == VALUE_TYPE_F64 ? 8 : 4; }

Fixture load(const char *name) {
    Fixture fixture = {};

    char path[1024];
    snprintf(path, sizeof(path), "%s/%s.bin", ROAM_FIXTURES_DIR, name);

    FILE *file = fopen(path, "rb");
    if (file == nullptr) {
        char message[1100];
        snprintf(message, sizeof(message), "missing fixture %s, build the navcat_fixtures target", path);
        test::fail(__FILE__, __LINE__, message);
        return fixture;
    }

    uint32_t entry_count = 0;
    bool ok = read_u32(file, entry_count);

    for (uint32_t i = 0; ok && i < entry_count; i++) {
        Entry entry = {};
        uint32_t name_length = 0;
        uint32_t type = 0;
        ok = read_u32(file, name_length) && name_length < sizeof(entry.name) &&
             fread(entry.name, 1, name_length, file) == name_length && read_u32(file, type) && read_u32(file, entry.count);
        if (!ok) break;

        entry.type = (ValueType)type;
        size_t bytes = (size_t)entry.count * value_size(entry.type);
        entry.data = roam::memory::allocate(bytes);
        ok = bytes == 0 || fread(entry.data, 1, bytes, file) == bytes;
        fixture.entries.push(entry);
    }

    fclose(file);

    if (!ok) {
        test::fail(__FILE__, __LINE__, "corrupt fixture file");
        release(fixture);
        return fixture;
    }

    fixture.loaded = true;
    return fixture;
}

void release(Fixture &fixture) {
    for (Entry &entry : fixture.entries) roam::memory::free(entry.data, (size_t)entry.count * value_size(entry.type));
    fixture.entries.release();
    fixture.loaded = false;
}

static const Entry *find(const Fixture &fixture, const char *name, ValueType type) {
    for (const Entry &entry : fixture.entries) {
        if (strcmp(entry.name, name) == 0 && entry.type == type) return &entry;
    }
    char message[256];
    snprintf(message, sizeof(message), "fixture has no %s entry named %s", type == VALUE_TYPE_F64 ? "f64" : "i32", name);
    test::fail(__FILE__, __LINE__, message);
    return nullptr;
}

F64 f64(const Fixture &fixture, const char *name) {
    const Entry *entry = find(fixture, name, VALUE_TYPE_F64);
    if (entry == nullptr) return {nullptr, 0};
    return {(const double *)entry->data, entry->count};
}

I32 i32(const Fixture &fixture, const char *name) {
    const Entry *entry = find(fixture, name, VALUE_TYPE_I32);
    if (entry == nullptr) return {nullptr, 0};
    return {(const int32_t *)entry->data, entry->count};
}

double number(const Fixture &fixture, const char *name) {
    F64 values = f64(fixture, name);
    return values.count == 1 ? values.data[0] : 0.0;
}

}  // namespace fixture
