#include "roam/geometry/geometry.h"
#include "tests/harness/test.h"

using namespace roam;

TEST(point_in_poly_point_on_edge_is_inside) {
    const float vertices[] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    CHECK(geometry::point_in_poly({0.5f, 0, 0}, vertices, 4));
}

TEST(point_in_poly_point_outside_is_outside) {
    const float vertices[] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    CHECK(!geometry::point_in_poly({1.5f, 0, 0.5f}, vertices, 4));
}

TEST(point_in_poly_point_on_vertical_edge_is_inside) {
    const float vertices[] = {-13, 0, -10, -15, 0, -10, -14, 0, -8.5f, -13, 0, -8.5f};
    CHECK(geometry::point_in_poly({-13, 0, -9}, vertices, 4));
}

TEST(overlap_poly_poly_2d_detects_overlap_and_separation) {
    const float square[] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    const float overlapping[] = {0.5f, 0, 0.5f, 1.5f, 0, 0.5f, 1.5f, 0, 1.5f, 0.5f, 0, 1.5f};
    const float separate[] = {2, 0, 0, 3, 0, 0, 3, 0, 1, 2, 0, 1};
    CHECK(geometry::overlap_poly_poly_2d(square, 4, overlapping, 4));
    CHECK(!geometry::overlap_poly_poly_2d(square, 4, separate, 4));
}

TEST(intersect_segment_poly_2d_clips_to_the_polygon) {
    const float square[] = {0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0};
    geometry::IntersectSegmentPoly2dResult result = geometry::intersect_segment_poly_2d({-1, 0, 0.5f}, {2, 0, 0.5f}, 4, square);
    CHECK(result.intersects);
    CHECK_NEAR(result.tmin, 1.0 / 3.0, 1e-6);
    CHECK_NEAR(result.tmax, 2.0 / 3.0, 1e-6);
}

TEST(closest_height_point_triangle_interpolates_or_misses) {
    float height = geometry::closest_height_point_triangle({0.25f, 0, 0.25f}, {0, 0, 0}, {0, 1, 1}, {1, 1, 0});
    CHECK_NEAR(height, 0.5, 1e-6);
    CHECK(isnan(geometry::closest_height_point_triangle({2, 0, 2}, {0, 0, 0}, {0, 1, 1}, {1, 1, 0})));
}
