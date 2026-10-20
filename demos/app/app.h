#pragma once

#include "app/camera.h"
#include "app/model.h"
#include "app/renderer.h"
#include "roam/util/array.h"

struct Click {
    bool clicked;
    bool hit;
    roam::Vec3 point;
    bool secondary;
    bool shift;
};

struct App {
    Renderer renderer;
    Camera camera;
    Model level;
    bool show_level;
    Click click;
    roam::Array<StaticDrawing> statics;
};

struct Example {
    const char *name;
    void (*init)(App &app);
    void (*update)(App &app, float delta_time);
    void (*ui)(App &app);
    void (*shutdown)(App &app);
};

namespace app {

void load_level(App &app, const char *name);
void draw_static(App &app, const StaticDrawing &drawing);
DrawList &frame(App &app);
double now_milliseconds();
void lift(roam::DebugPrimitives &primitives, float height);

}
