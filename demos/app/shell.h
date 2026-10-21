#pragma once

#include "app/app.h"
#include "sokol_gfx.h"

struct FrameInput {
    float width;
    float height;
    float delta_time;
    float dpi_scale;
    float pointer_x;
    float pointer_y;
    sg_swapchain swapchain;
};

namespace shell {

App &app();
void setup(const sg_environment &environment, int initial_example);
void shutdown();

void select_example(int index);
int current_example();
int find_example(const char *name);

void frame(const FrameInput &input);
void click(float x, float y, bool secondary, bool shift);
void read_dropped_file(const char *path);

}
