#include <OpenGL/OpenGL.h>
#include <OpenGL/gl3.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app/examples.h"
#include "app/shell.h"
#include "roam/util/array.h"
#include "roam/util/memory.h"
#include "sokol_time.h"
#include "stb_image_write.h"

using namespace roam;

static const int FRAMES = 120;
static const int WIDTH = 600;
static const int HEIGHT = 400;
static const int SAMPLES = 4;
static const char *OUT_DIR = "demos/screenshots";
static const float FIT_MARGIN = 0.8f;
static const float CUSTOM_FRAMING_ZOOM = 0.9f;

struct Target {
    GLuint msaa_framebuffer;
    GLuint msaa_color;
    GLuint msaa_depth;
    GLuint resolve_framebuffer;
    GLuint resolve_color;
};

static bool create_context() {
    const CGLPixelFormatAttribute attributes[] = {
        kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_GL4_Core,
        kCGLPFAAccelerated,
        kCGLPFAColorSize, (CGLPixelFormatAttribute)24,
        kCGLPFAAlphaSize, (CGLPixelFormatAttribute)8,
        (CGLPixelFormatAttribute)0,
    };
    CGLPixelFormatObj pixel_format = nullptr;
    GLint format_count = 0;
    if (CGLChoosePixelFormat(attributes, &pixel_format, &format_count) != kCGLNoError || !pixel_format) return false;
    CGLContextObj context = nullptr;
    CGLError error = CGLCreateContext(pixel_format, nullptr, &context);
    CGLDestroyPixelFormat(pixel_format);
    if (error != kCGLNoError) return false;
    return CGLSetCurrentContext(context) == kCGLNoError;
}

static Target create_target() {
    Target target = {};
    glGenRenderbuffers(1, &target.msaa_color);
    glBindRenderbuffer(GL_RENDERBUFFER, target.msaa_color);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_RGBA8, WIDTH, HEIGHT);
    glGenRenderbuffers(1, &target.msaa_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, target.msaa_depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, SAMPLES, GL_DEPTH24_STENCIL8, WIDTH, HEIGHT);
    glGenFramebuffers(1, &target.msaa_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, target.msaa_framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target.msaa_color);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, target.msaa_depth);

    glGenRenderbuffers(1, &target.resolve_color);
    glBindRenderbuffer(GL_RENDERBUFFER, target.resolve_color);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, WIDTH, HEIGHT);
    glGenFramebuffers(1, &target.resolve_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, target.resolve_framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target.resolve_color);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return target;
}

static sg_environment offscreen_environment() {
    sg_environment environment = {};
    environment.defaults.color_format = SG_PIXELFORMAT_RGBA8;
    environment.defaults.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    environment.defaults.sample_count = SAMPLES;
    return environment;
}

static sg_swapchain offscreen_swapchain(const Target &target) {
    sg_swapchain swapchain = {};
    swapchain.width = WIDTH;
    swapchain.height = HEIGHT;
    swapchain.sample_count = SAMPLES;
    swapchain.color_format = SG_PIXELFORMAT_RGBA8;
    swapchain.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    swapchain.gl.framebuffer = target.msaa_framebuffer;
    return swapchain;
}

static bool save_png(const Target &target, const char *path) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, target.msaa_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target.resolve_framebuffer);
    glBlitFramebuffer(0, 0, WIDTH, HEIGHT, 0, 0, WIDTH, HEIGHT, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, target.resolve_framebuffer);

    Array<uint8_t> pixels = {};
    pixels.resize((uint32_t)(WIDTH * HEIGHT * 4));
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    Array<uint8_t> image = {};
    image.resize((uint32_t)(WIDTH * HEIGHT * 3));
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            for (int c = 0; c < 3; c++) {
                image[(uint32_t)((y * WIDTH + x) * 3 + c)] = pixels[(uint32_t)(((HEIGHT - 1 - y) * WIDTH + x) * 4 + c)];
            }
        }
    }
    const bool written = stbi_write_png(path, WIDTH, HEIGHT, 3, image.data, WIDTH * 3) != 0;
    image.release();
    pixels.release();
    return written;
}

static void fit_camera(Camera &camera, const Array<float> &positions) {
    const float aspect = (float)WIDTH / (float)HEIGHT;
    for (int iteration = 0; iteration < 32; iteration++) {
        camera::update(camera, (float)WIDTH, (float)HEIGHT);
        float min_x = 1e30f, min_y = 1e30f, max_x = -1e30f, max_y = -1e30f;
        for (uint32_t i = 0; i + 2 < positions.length; i += 3) {
            const Vec3 point = {positions[i], positions[i + 1], positions[i + 2]};
            const float *m = camera.view_projection.m;
            const float x = m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12];
            const float y = m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13];
            const float w = fmaxf(m[3] * point.x + m[7] * point.y + m[11] * point.z + m[15], 1e-4f);
            min_x = fminf(min_x, x / w);
            max_x = fmaxf(max_x, x / w);
            min_y = fminf(min_y, y / w);
            max_y = fmaxf(max_y, y / w);
        }
        const float half_tan = tanf(camera.fov_y * 0.5f) * camera.distance;
        camera.target = vec3::scale_and_add(camera.target, camera.side, (min_x + max_x) * 0.5f * half_tan * aspect);
        camera.target = vec3::scale_and_add(camera.target, camera.up, (min_y + max_y) * 0.5f * half_tan);
        const float extent = fmaxf(max_x - min_x, max_y - min_y) * 0.5f;
        camera.distance *= powf(extent / FIT_MARGIN, 0.5f);
    }
}

static bool run_example(int index, const Target &target) {
    MemoryStats before = memory::outstanding();
    uint64_t start = stm_now();

    char stem[256];
    snprintf(stem, sizeof(stem), "example_%s", EXAMPLES[index]->name);
    for (char *c = stem; *c; c++) {
        if (*c == ' ') *c = '_';
    }
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.png", OUT_DIR, stem);

    shell::setup(offscreen_environment(), index);
    Camera &camera = shell::app().camera;
    const Camera defaults = camera::create();
    const bool default_framing = camera.yaw == defaults.yaw && camera.pitch == defaults.pitch;
    if (default_framing) {
        fit_camera(camera, shell::app().level.positions);
    } else {
        camera.distance *= CUSTOM_FRAMING_ZOOM;
    }
    const bool upload = strcmp(EXAMPLES[index]->name, "upload model") == 0;
    for (int frame = 0; frame < FRAMES; frame++) {
        if (upload && frame == 1) shell::read_dropped_file(ROAM_DEMOS_ASSETS_DIR "/nav-test.glb");
        if (upload && frame == 2) fit_camera(camera, shell::app().level.positions);
        FrameInput input = {(float)WIDTH, (float)HEIGHT, 1.0f / 60.0f, 1.0f, -1.0f, -1.0f, offscreen_swapchain(target), true};
        shell::frame(input);
    }
    glFinish();
    const bool written = save_png(target, path);
    shell::shutdown();

    MemoryStats after = memory::outstanding();
    const bool leaked = after.allocations != before.allocations;
    printf("%-4s %-48s %8.1f ms  %s\n", !written ? "FAIL" : leaked ? "LEAK" : "ok", EXAMPLES[index]->name, stm_ms(stm_since(start)), path);
    fflush(stdout);
    return written && !leaked;
}

int main(int argc, char **argv) {
    stm_setup();
    if (!create_context()) {
        printf("could not create an offscreen OpenGL context\n");
        return 1;
    }
    const Target target = create_target();
    int failures = 0;
    int ran = 0;
    for (int i = 0; i < EXAMPLE_COUNT; i++) {
        bool selected = argc < 2;
        for (int arg = 1; arg < argc; arg++) selected |= strstr(EXAMPLES[i]->name, argv[arg]) != nullptr;
        if (!selected) continue;
        if (!run_example(i, target)) failures++;
        ran++;
    }
    printf("%d examples, %d failed\n", ran, failures);
    return failures == 0 ? 0 : 1;
}
