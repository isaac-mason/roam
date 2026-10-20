#pragma once

#include "roam/generate/build_context.h"
#include "roam/generate/compact_heightfield.h"

namespace roam {

namespace compact_heightfield_regions {

void build_distance_field(CompactHeightfield &compact_heightfield);

bool build_regions(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area,
                   int merge_region_area);

bool build_regions_monotone(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area,
                            int merge_region_area);

bool build_layer_regions(BuildContext &context, CompactHeightfield &compact_heightfield, int border_size, int min_region_area);

}

}
