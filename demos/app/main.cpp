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

#include "app/shell.h"
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

static Input input;
static int initial_example = 0;
static const char *screenshot_path = nullptr;
static const char *drop_path = nullptr;
static int frame_count = 0;

static void init() {
    stm_setup();
    shell::setup(sglue_environment(), initial_example);
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
    if (drop_path && frame_count == 1) shell::read_dropped_file(drop_path);

    FrameInput frame_input = {sapp_widthf(), sapp_heightf(), (float)sapp_frame_duration(), sapp_dpi_scale(), input.x, input.y, sglue_swapchain(), false};
    shell::frame(frame_input);

    frame_count++;
    if (screenshot_path && frame_count == 30) save_screenshot();
}

static void cleanup() { shell::shutdown(); }

#if defined(__EMSCRIPTEN__)
static void dropped_file_fetched(const sapp_html5_fetch_response *response) {
    shell::app().dropped.pending = false;
    if (response->succeeded) {
        shell::app().dropped.data.resize((uint32_t)response->data.size);
        shell::app().dropped.ready = true;
    } else {
        shell::app().dropped.failed = true;
    }
}
#endif

static void receive_dropped_file() {
    DroppedFile &dropped = shell::app().dropped;
    dropped.ready = false;
    dropped.failed = false;
    if (sapp_get_num_dropped_files() < 1) return;
    snprintf(dropped.path, sizeof(dropped.path), "%s", sapp_get_dropped_file_path(0));

#if defined(__EMSCRIPTEN__)
    uint32_t size = sapp_html5_get_dropped_file_size(0);
    dropped.data.clear();
    dropped.data.resize(size);
    sapp_html5_fetch_request request = {};
    request.dropped_file_index = 0;
    request.callback = dropped_file_fetched;
    request.buffer = {dropped.data.data, size};
    dropped.pending = true;
    sapp_html5_fetch_dropped_file(&request);
#else
    shell::read_dropped_file(sapp_get_dropped_file_path(0));
#endif
}

static void click(float x, float y, sapp_mousebutton button, uint32_t modifiers) {
    shell::click(x, y, button != SAPP_MOUSEBUTTON_LEFT, (modifiers & SAPP_MODIFIER_SHIFT) != 0);
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
            float threshold = 5 * sapp_dpi_scale();
            if (moved_x * moved_x + moved_y * moved_y > threshold * threshold) input.dragging = true;
            if (!input.dragging) break;
            if (input.button == SAPP_MOUSEBUTTON_LEFT && (e->modifiers & SAPP_MODIFIER_SHIFT) == 0) {
                camera::orbit(shell::app().camera, dx, dy);
            } else {
                camera::pan(shell::app().camera, dx, dy);
            }
            break;
        }
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (input.down && !input.dragging) click(e->mouse_x, e->mouse_y, e->mouse_button, e->modifiers);
            input.down = false;
            break;
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (!ImGui::GetIO().WantCaptureKeyboard && e->key_code >= 0 && e->key_code < 512) shell::app().keys[e->key_code] = true;
            break;
        case SAPP_EVENTTYPE_KEY_UP:
            if (e->key_code >= 0 && e->key_code < 512) shell::app().keys[e->key_code] = false;
            break;
        case SAPP_EVENTTYPE_FILES_DROPPED:
            receive_dropped_file();
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            camera::zoom(shell::app().camera, e->scroll_y);
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
        if (strcmp(argv[arg], "--drop") == 0 && arg + 1 < argc) {
            drop_path = argv[++arg];
            continue;
        }
        int found = shell::find_example(argv[arg]);
        if (found >= 0) initial_example = found;
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
    desc.enable_dragndrop = true;
    desc.max_dropped_file_path_length = 4096;
    desc.logger.func = slog_func;
    return desc;
}
