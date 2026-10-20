# Porting a navcat example to the roam demo app

Each navcat example in `~/Development/navcat/examples/src/example-<name>.ts` becomes `demos/examples/example_<name>.cpp` (snake_case). Read the navcat example fully first and keep its behaviour: same model, same generation settings, same default points and the same interaction, as close as the app allows. three.js visuals become roam debug drawing.

## The app

- `demos/app/app.h`: `App` (renderer, camera, level model, `click`, `hover`, `keys`), `Example { name, init, update, ui, shutdown }`, and `app::load_level`, `app::draw_static`, `app::frame`, `app::now_milliseconds`, `app::lift`.
- An example file keeps its state in one file-static struct, defines static `init`, `update`, `ui` and `shutdown`, and ends with `const Example EXAMPLE_<NAME> = {"<name in words>", init, update, ui, shutdown};`. Look at `demos/examples/example_find_path.cpp` and `example_crowd_simulation.cpp` for the pattern.
- Register it in `demos/app/examples.h` (an `extern const Example`) and `demos/app/examples.cpp` (in `EXAMPLES`, in the order of navcat's `examples/src/examples.json`).
- `init` loads the level with `app::load_level(app, "<model>.glb")`, generates the nav mesh and builds static drawings. `shutdown` releases everything `init` made. `init` may run again after `shutdown`, so reset the state struct at the start of `init`.
- `update(app, dt)` runs every frame before the UI. Read `app.click` (a click without dragging: `clicked`, `hit`, `point` on the level, `secondary` for right click, `shift`). Set `app.want_hover = true` in `init` to get `app.hover` (`hit`, `point`) under the mouse every frame. `app.keys[SAPP_KEYCODE_W]` and friends are held keys (include `sokol_app.h`).
- `ui(app)` draws ImGui widgets inside the app window. Start with a one-line `ImGui::TextWrapped` of how to interact, then the example's controls. navcat's lil-gui folders become `ImGui::CollapsingHeader`.
- Drawing:
  - `renderer::create_static(DebugPrimitives)` or `create_static(DrawList)` makes GPU buffers once. Pass it to `app::draw_static(app, drawing)` every frame you want it drawn, and `renderer::release` it when it changes or on shutdown.
  - `app::frame(app)` is the per-frame `DrawList`, cleared every frame. Add with `renderer::line`, `triangle`, `point` (an octahedron of a given radius), `box`, `circle`, `cylinder`, `arrow`, or `renderer::add_primitives(list, debug_primitives)`.
  - `Color` is `{r, g, b, a}`; alpha below 1 draws transparent.
  - navcat lifts helpers with `object.position.y += h`; use `app::lift(primitives, h)` before building the drawing.
- Generation settings: `demos/app/nav_mesh_config.h` has navcat's usual config, `solo_options`, `tiled_options` and an ImGui `ui`. Use it when the example's config matches, otherwise build `SoloNavMeshOptions` or `TiledNavMeshOptions` directly with the example's values.
- Random numbers: use a small LCG in the file (see `next_random` in the crowd example) passed to roam's `RandomFunction` APIs, not `rand()`.

## roam

- The library is `roam/` (core) and `roam/blocks/` (generators, agents, chunky tri mesh, flood fill, merge positions). Headers sit next to sources. navcat names map to snake_case: `findNearestPoly` is `nav_mesh_api::find_nearest_poly`, `createNavMeshHelper` is `debug::create_nav_mesh_helper`, `crowd.addAgent` is `crowd::add_agent`.
- Results own `Array`s: release them (`find_path::release`, `nav_mesh_search::release`, `debug::release`, ...).
- If a navcat API the example needs is missing from roam, port it into the library following navcat exactly (and the code style below), add a unit test in `tests/`, and say so in your report.

## Style

- Read `CLAUDE.md`. C in C++20: plain structs, free functions, snake_case functions, PascalCase types, UPPER_SNAKE constants, no STL, no exceptions, no comments, return by value. roam's `Array` and `Map` instead of std containers. Lambdas are fine for local helpers; no new templates.
- Match the existing demo files. Keep each example in one file.

## Building and checking

- `cmake --preset demos && cmake --build --preset demos` (configure fetches sokol, imgui and cgltf). No warnings.
- Check every example you port by rendering it: `build/demos/demos/roam_demos "<name in words>" --screenshot /path/out.ppm` renders 30 frames, saves the framebuffer and quits; convert with `sips -s format png out.ppm --out out.png` and look at it. Compare against navcat's screenshot in `examples/public/screenshots/example-<name>.png`. Fix anything that looks wrong.
- Don't take screenshots of the desktop, only of the app's framebuffer this way.
- Models: copy what the example needs from `navcat/examples/public/models/` into `demos/assets/`, along with its `.md` credit file when one exists. Never copy `.blend` files.

## Not ported yet

- `dynamic-navmesh`, `dynamic-obstacles`, `fps-dynamic-navmesh` use Rapier physics, and `upload-model` needs a file picker. Leave them.
