#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#elif defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#elif !defined(__EMSCRIPTEN__)
#include <GL/gl.h>
#endif

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_imgui.h"
#include "sokol_log.h"
#include "sokol_time.h"

using namespace roam;

struct Input {
    bool down;
    bool dragging;
    sapp_mousebutton button;
    float down_x;
    float down_y;
    float x;
    float y;
};

static App state;
static Input input;
static int current_example = -1;
static int initial_example = 0;
static const char *screenshot_path = nullptr;
static int frame_count = 0;

namespace app {

void load_level(App &app, const char *name, bool skip_unwalkable_nodes) { set_level(app, model::load(name, skip_unwalkable_nodes)); }

void set_level(App &app, Model level) {
    model::release(app.level);
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

static void select_example(int index) {
    if (index == current_example) return;
    if (current_example >= 0 && EXAMPLES[current_example]->shutdown) EXAMPLES[current_example]->shutdown(state);
    current_example = index;
    state.click = {};
    state.want_hover = false;
    EXAMPLES[current_example]->init(state);
}

static void init() {
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(&desc);

    simgui_desc_t imgui = {};
    imgui.logger.func = slog_func;
    simgui_setup(&imgui);

    stm_setup();

    state.renderer = renderer::create();
    state.camera = camera::create();
    state.show_level = true;

    select_example(initial_example);
}

static void save_screenshot() {
#if !defined(__EMSCRIPTEN__)
    int width = sapp_width();
    int height = sapp_height();
    Array<uint8_t> pixels = {};
    pixels.resize((uint32_t)(width * height * 3));
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data);

    FILE *file = fopen(screenshot_path, "wb");
    if (file) {
        fprintf(file, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; y--) fwrite(&pixels[(uint32_t)(y * width * 3)], 1, (size_t)width * 3, file);
        fclose(file);
    }
    pixels.release();
#endif
    sapp_request_quit();
}

static void frame() {
    float width = sapp_widthf();
    float height = sapp_heightf();
    float delta_time = (float)sapp_frame_duration();

    simgui_frame_desc_t imgui_frame = {};
    imgui_frame.width = sapp_width();
    imgui_frame.height = sapp_height();
    imgui_frame.delta_time = sapp_frame_duration();
    imgui_frame.dpi_scale = sapp_dpi_scale();
    simgui_new_frame(&imgui_frame);

    camera::update(state.camera, width, height);
    renderer::clear(state.renderer.frame);
    state.statics.clear();

    state.hover = {};
    if (state.want_hover && !ImGui::GetIO().WantCaptureMouse) {
        RayHit hit = model::raycast(state.level, camera::screen_ray(state.camera, input.x, input.y));
        state.hover = {hit.hit, hit.point};
    }

    const Example &example = *EXAMPLES[current_example];
    if (example.update) example.update(state, delta_time);
    state.click = {};

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("roam");
    if (ImGui::BeginCombo("example", example.name)) {
        for (int i = 0; i < EXAMPLE_COUNT; i++) {
            if (ImGui::Selectable(EXAMPLES[i]->name, i == current_example)) select_example(i);
        }
        ImGui::EndCombo();
    }
    ImGui::Checkbox("show level", &state.show_level);
    ImGui::Text("%.1f ms", delta_time * 1000.0f);
    ImGui::Separator();
    if (EXAMPLES[current_example]->ui) EXAMPLES[current_example]->ui(state);
    ImGui::End();

    sg_pass pass = {};
    pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
    pass.action.colors[0].clear_value = {0.125f, 0.125f, 0.125f, 1.0f};
    pass.swapchain = sglue_swapchain();
    sg_begin_pass(&pass);
    renderer::render(state.renderer, state.camera, state.statics.data, state.statics.length, state.show_level);
    simgui_render();
    sg_end_pass();
    sg_commit();

    frame_count++;
    if (screenshot_path && frame_count == 30) save_screenshot();
}

static void cleanup() {
    if (current_example >= 0 && EXAMPLES[current_example]->shutdown) EXAMPLES[current_example]->shutdown(state);
    model::release(state.level);
    state.statics.release();
    renderer::destroy(state.renderer);
    simgui_shutdown();
    sg_shutdown();
}

static void click(float x, float y, sapp_mousebutton button, uint32_t modifiers) {
    Ray ray = camera::screen_ray(state.camera, x, y);
    RayHit hit = model::raycast(state.level, ray);
    state.click = {true, hit.hit, hit.point, button != SAPP_MOUSEBUTTON_LEFT, (modifiers & SAPP_MODIFIER_SHIFT) != 0, ray};
}

static void event(const sapp_event *e) {
    if (simgui_handle_event(e) && ImGui::GetIO().WantCaptureMouse) {
        input.down = false;
        return;
    }

    switch (e->type) {
        case SAPP_EVENTTYPE_MOUSE_DOWN:
            input = {true, false, e->mouse_button, e->mouse_x, e->mouse_y, e->mouse_x, e->mouse_y};
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE: {
            float dx = e->mouse_x - input.x;
            float dy = e->mouse_y - input.y;
            input.x = e->mouse_x;
            input.y = e->mouse_y;
            if (!input.down) break;
            float moved_x = e->mouse_x - input.down_x;
            float moved_y = e->mouse_y - input.down_y;
            if (moved_x * moved_x + moved_y * moved_y > 16) input.dragging = true;
            if (!input.dragging) break;
            if (input.button == SAPP_MOUSEBUTTON_LEFT && (e->modifiers & SAPP_MODIFIER_SHIFT) == 0) {
                camera::orbit(state.camera, dx, dy);
            } else {
                camera::pan(state.camera, dx, dy);
            }
            break;
        }
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (input.down && !input.dragging) click(e->mouse_x, e->mouse_y, e->mouse_button, e->modifiers);
            input.down = false;
            break;
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (!ImGui::GetIO().WantCaptureKeyboard && e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = true;
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if (e->key_code >= 0 && e->key_code < 512) state.keys[e->key_code] = false;
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            camera::zoom(state.camera, e->scroll_y);
            break;
        default:
            break;
    }
}

sapp_desc sokol_main(int argc, char **argv) {
    for (int arg = 1; arg < argc; arg++) {
        if (strcmp(argv[arg], "--screenshot") == 0 && arg + 1 < argc) {
            screenshot_path = argv[++arg];
            continue;
        }
        for (int i = 0; i < EXAMPLE_COUNT; i++) {
            if (strcmp(EXAMPLES[i]->name, argv[arg]) == 0) initial_example = i;
        }
    }

    sapp_desc desc = {};
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = event;
    desc.width = 1280;
    desc.height = 800;
    desc.sample_count = 4;
    desc.high_dpi = true;
    desc.window_title = "roam";
    desc.logger.func = slog_func;
    return desc;
}
