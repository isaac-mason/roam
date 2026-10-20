#include "roam/util/index_pool.h"
#include "tests/harness/test.h"

using namespace roam;

TEST(index_pool_counts_up_then_reuses_released) {
    IndexPool pool = {};
    CHECK(index_pool::request(pool) == 0);
    CHECK(index_pool::request(pool) == 1);
    CHECK(index_pool::request(pool) == 2);
    index_pool::release_index(pool, 1);
    CHECK(index_pool::request(pool) == 1);
    CHECK(index_pool::request(pool) == 3);
    index_pool::release(pool);
}

TEST(index_pool_reuses_most_recently_released_first) {
    IndexPool pool = {};
    for (int i = 0; i < 4; i++) index_pool::request(pool);
    index_pool::release_index(pool, 0);
    index_pool::release_index(pool, 2);
    CHECK(index_pool::request(pool) == 2);
    CHECK(index_pool::request(pool) == 0);
    index_pool::release(pool);
}
