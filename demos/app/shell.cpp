#include "app/shell.h"

#include <stdio.h>
#include <string.h>

#include "app/examples.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_imgui.h"
#include "sokol_log.h"
#include "sokol_time.h"

using namespace roam;

static App state;
static int active_example = -1;

namespace app {

void load_level(App &app, const char *name, ModelLoadOptions options) { set_level(app, model::load(name, options)); }

void set_level(App &app, Model level) {
    model::release(app.level);
    if (level.nodes.length == 0 && level.indices.length > 0) {
        ModelNode root = {};
        root.parent = -1;
        root.index_count = level.indices.length;
        root.bounds = level.bounds;
        level.nodes.push(root);
    }
    app.level = level;
    renderer::set_level(app.renderer, app.level);
    if (app.level.loaded) camera::frame_bounds(app.camera, app.level.bounds);
}

void draw_static(App &app, const StaticDrawing &drawing) { app.statics.push(drawing); }

DrawList &frame(App &app) { return app.renderer.frame; }

double now_milliseconds() { return stm_ms(stm_now()); }

void lift(DebugPrimitives &primitives, float height) {
    for (DebugPrimitive &primitive : primitives) {
        for (uint32_t i = 1; i < primitive.positions.length; i += 3) primitive.positions[i] += height;
    }
}

}

namespace shell {

App &app() { return state; }

void select_example(int index) {
    if (index == active_example) return;
    if (active_example >= 0 && EXAMPLES[active_example]->shutdown) EXAMPLES[active_example]->shutdown(state);
    active_example = index;
    state.click = {};
    state.want_hover = false;
    EXAMPLES[active_example]->init(state);
}

int current_example() { return active_example; }

int find_example(const char *name) {
    for (int i = 0; i < EXAMPLE_COUNT; i++) {
        if (strcmp(EXAMPLES[i]->name, name) == 0) return i;
    }
    return -1;
}

void setup(const sg_environment &environment, int initial_example) {
    sg_desc desc = {};
    desc.environment = environment;
    desc.logger.func = slog_func;
    sg_setup(&desc);

    simgui_desc_t imgui = {};
    imgui.logger.func = slog_func;
    simgui_setup(&imgui);

    state.renderer = renderer::create();
    state.camera = camera::create();
    state.show_level = true;

    select_example(initial_example);
}

void shutdown() {
    if (active_example >= 0 && EXAMPLES[active_example]->shutdown) EXAMPLES[active_example]->shutdown(state);
    active_example = -1;
    model::release(state.level);
    state.dropped.data.release();
    state.statics.release();
    renderer::destroy(state.renderer);
    simgui_shutdown();
    sg_shutdown();
    state = {};
}

void click(float x, float y, bool secondary, bool shift) {
    Ray ray = camera::screen_ray(state.camera, x, y);
    RayHit hit = model::raycast(state.level, ray);
    state.click = {true, hit.hit, hit.point, secondary, shift, ray};
}

void read_dropped_file(const char *path) {
    DroppedFile &dropped = state.dropped;
    snprintf(dropped.path, sizeof(dropped.path), "%s", path);
    FILE *file = fopen(dropped.path, "rb");
    if (!file) {
        dropped.failed = true;
        return;
    }
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    dropped.data.clear();
    dropped.data.resize((uint32_t)size);
    size_t read = fread(dropped.data.data, 1, (size_t)size, file);
    fclose(file);
    dropped.data.resize((uint32_t)read);
    dropped.ready = true;
}

void frame(const FrameInput &input) {
    simgui_frame_desc_t imgui_frame = {};
    imgui_frame.width = (int)input.width;
    imgui_frame.height = (int)input.height;
    imgui_frame.delta_time = input.delta_time;
    imgui_frame.dpi_scale = input.dpi_scale;
    simgui_new_frame(&imgui_frame);

    camera::update(state.camera, input.width, input.height);
    renderer::clear(state.renderer.frame);
    state.statics.clear();

    state.hover = {};
    if (state.want_hover && !ImGui::GetIO().WantCaptureMouse) {
        Ray ray = camera::screen_ray(state.camera, input.pointer_x, input.pointer_y);
        RayHit hit = model::raycast(state.level, ray);
        state.hover = {hit.hit, hit.point, ray};
    }

    const Example &example = *EXAMPLES[active_example];
    if (example.update) example.update(state, input.delta_time);
    state.click = {};
    if (state.dropped.ready || state.dropped.failed) {
        state.dropped.ready = false;
        state.dropped.failed = false;
        state.dropped.data.clear();
    }

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("roam");
    if (ImGui::BeginCombo("example", example.name)) {
        for (int i = 0; i < EXAMPLE_COUNT; i++) {
            if (ImGui::Selectable(EXAMPLES[i]->name, i == active_example)) select_example(i);
        }
        ImGui::EndCombo();
    }
    ImGui::Checkbox("show level", &state.show_level);
    ImGui::Text("%.1f ms", input.delta_time * 1000.0f);
    ImGui::Separator();
    if (EXAMPLES[active_example]->ui) EXAMPLES[active_example]->ui(state);
    ImGui::End();

    sg_pass pass = {};
    pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
    pass.action.colors[0].clear_value = {0.125f, 0.125f, 0.125f, 1.0f};
    pass.swapchain = input.swapchain;
    sg_begin_pass(&pass);
    renderer::render(state.renderer, state.camera, state.statics.data, state.statics.length, state.show_level);
    simgui_render();
    sg_end_pass();
    sg_commit();
}

}
