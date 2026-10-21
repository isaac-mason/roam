#include "tests/harness/fixture.h"
#include "tests/harness/test.h"

TEST_CASE("nav test solo fixture loads") {
    fixture::Fixture nav_test = fixture::load("nav_test_solo");
    REQUIRE(nav_test.loaded);

    fixture::F64 positions = fixture::f64(nav_test, "input.positions");
    fixture::I32 indices = fixture::i32(nav_test, "input.indices");
    CHECK(positions.count == 884 * 3);
    CHECK(indices.count == 1612 * 3);
    CHECK(fixture::i32(nav_test, "tri_area_ids").count == 1612);

    fixture::I32 grid_size = fixture::i32(nav_test, "grid_size");
    REQUIRE(grid_size.count == 2);
    CHECK(fixture::i32(nav_test, "heightfield.rasterized.column_span_counts").count == (uint32_t)(grid_size.data[0] * grid_size.data[1]));

    fixture::release(nav_test);
}
