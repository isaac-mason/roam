#include "roam/util/index_pool.h"
#include "tests/harness/test.h"

using namespace roam;

TEST_CASE("index pool counts up then reuses released") {
    IndexPool pool = {};
    CHECK(index_pool::request(pool) == 0);
    CHECK(index_pool::request(pool) == 1);
    CHECK(index_pool::request(pool) == 2);
    index_pool::release_index(pool, 1);
    CHECK(index_pool::request(pool) == 1);
    CHECK(index_pool::request(pool) == 3);
    index_pool::release(pool);
}

TEST_CASE("index pool reuses most recently released first") {
    IndexPool pool = {};
    for (int i = 0; i < 4; i++) index_pool::request(pool);
    index_pool::release_index(pool, 0);
    index_pool::release_index(pool, 2);
    CHECK(index_pool::request(pool) == 2);
    CHECK(index_pool::request(pool) == 0);
    index_pool::release(pool);
}
