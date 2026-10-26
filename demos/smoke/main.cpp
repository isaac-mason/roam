#include <stdio.h>
#include <string.h>

#include "app/examples.h"
#include "app/shell.h"
#include "roam/util/memory.h"
#include "sokol_time.h"

using namespace roam;

static const int FRAMES = 30;
static const float WIDTH = 1280;
static const float HEIGHT = 800;

static sg_environment headless_environment() {
    sg_environment environment = {};
    environment.defaults.color_format = SG_PIXELFORMAT_RGBA8;
    environment.defaults.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    environment.defaults.sample_count = 1;
    return environment;
}

static sg_swapchain headless_swapchain() {
    sg_swapchain swapchain = {};
    swapchain.width = (int)WIDTH;
    swapchain.height = (int)HEIGHT;
    swapchain.sample_count = 1;
    swapchain.color_format = SG_PIXELFORMAT_RGBA8;
    swapchain.depth_format = SG_PIXELFORMAT_DEPTH_STENCIL;
    return swapchain;
}

static bool run_example(int index) {
    memory::MemoryStats before = memory::outstanding();
    uint64_t start = stm_now();

    shell::setup(headless_environment(), index);
    int clicks_hit = 0;
    for (int frame = 0; frame < FRAMES; frame++) {
        if (frame == 10) shell::click(WIDTH * 0.5f, HEIGHT * 0.5f, false, false);
        if (frame == 15) shell::click(WIDTH * 0.6f, HEIGHT * 0.45f, false, false);
        if (frame == 20) shell::click(WIDTH * 0.4f, HEIGHT * 0.55f, true, false);
        if (frame == 10 || frame == 15 || frame == 20) clicks_hit += shell::app().click.hit ? 1 : 0;
        FrameInput input = {WIDTH, HEIGHT, 1.0f / 60.0f, 1.0f, WIDTH * 0.5f, HEIGHT * 0.5f, headless_swapchain()};
        shell::frame(input);
    }
    bool has_level = shell::app().level.loaded;
    shell::shutdown();

    memory::MemoryStats after = memory::outstanding();
    bool leaked = after.allocations != before.allocations;
    bool missed = has_level && clicks_hit == 0;
    printf("%-4s %-48s %8.1f ms", leaked ? "LEAK" : missed ? "MISS" : "ok", EXAMPLES[index]->name, stm_ms(stm_since(start)));
    if (leaked) printf("  %lld allocations", (long long)(after.allocations - before.allocations));
    if (missed) printf("  no click hit the level");
    printf("\n");
    fflush(stdout);
    return !leaked && !missed;
}

int main(int argc, char **argv) {
    stm_setup();
    int failures = 0;
    int ran = 0;
    for (int i = 0; i < EXAMPLE_COUNT; i++) {
        bool selected = argc < 2;
        for (int arg = 1; arg < argc; arg++) selected |= strstr(EXAMPLES[i]->name, argv[arg]) != nullptr;
        if (!selected) continue;
        if (!run_example(i)) failures++;
        ran++;
    }
    printf("%d examples, %d failed\n", ran, failures);
    return failures == 0 ? 0 : 1;
}
