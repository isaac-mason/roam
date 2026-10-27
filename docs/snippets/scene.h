#pragma once

#include "level.h"
#include "roam/blocks/generators/generate_solo_nav_mesh.h"
#include "roam/query/nav_mesh_api.h"
#include "roam/query/nav_mesh_search.h"

namespace roam {

struct Scene {
    Level level;
    NavMesh nav_mesh;
    QueryWorkspace workspace;
    QueryFilter filter;
};

SoloNavMeshOptions scene_options();
Scene create_scene();
void release(Scene &scene);

}
