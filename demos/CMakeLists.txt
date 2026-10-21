include(FetchContent)

FetchContent_Declare(sokol
    GIT_REPOSITORY https://github.com/floooh/sokol.git
    GIT_TAG 1847290135f95e57e6d220b0a41208306aafc0dd)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9)
FetchContent_Declare(cgltf
    GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
    GIT_TAG v1.15)
FetchContent_MakeAvailable(sokol imgui cgltf)

add_library(roam_demos_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp)
target_include_directories(roam_demos_imgui PUBLIC ${imgui_SOURCE_DIR})

file(GLOB ROAM_DEMO_SOURCES CONFIGURE_DEPENDS app/*.cpp examples/*.cpp)
file(GLOB ROAM_DEMO_SHARED_SOURCES CONFIGURE_DEPENDS app/*.cpp examples/*.cpp)
list(REMOVE_ITEM ROAM_DEMO_SHARED_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/app/main.cpp ${CMAKE_CURRENT_SOURCE_DIR}/app/sokol.cpp)
add_executable(roam_demos ${ROAM_DEMO_SOURCES})
target_include_directories(roam_demos PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${sokol_SOURCE_DIR} ${sokol_SOURCE_DIR}/util ${cgltf_SOURCE_DIR})
target_link_libraries(roam_demos PRIVATE roam_blocks roam_demos_imgui)
roam_compile_options(roam_demos)

if(EMSCRIPTEN)
    target_compile_definitions(roam_demos PRIVATE SOKOL_GLES3)
    set_target_properties(roam_demos PROPERTIES SUFFIX ".html")
    target_link_options(roam_demos PRIVATE
        -sUSE_WEBGL2=1
        -sALLOW_MEMORY_GROWTH=1
        -sSTACK_SIZE=1048576
        --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/app/shell.html
        --preload-file ${CMAKE_CURRENT_SOURCE_DIR}/assets@/assets)
    set_property(TARGET roam_demos APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/app/shell.html)
else()
    target_compile_definitions(roam_demos PRIVATE SOKOL_GLCORE ROAM_DEMOS_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
    if(APPLE)
        set_source_files_properties(app/sokol.cpp PROPERTIES COMPILE_OPTIONS "-xobjective-c++;-Wno-deprecated-enum-enum-conversion")
        target_compile_definitions(roam_demos PRIVATE GL_SILENCE_DEPRECATION)
        target_link_libraries(roam_demos PRIVATE "-framework Cocoa" "-framework QuartzCore" "-framework OpenGL")
    elseif(UNIX)
        find_package(Threads REQUIRED)
        target_link_libraries(roam_demos PRIVATE X11 Xi Xcursor GL dl m Threads::Threads)
    endif()
endif()

if(NOT EMSCRIPTEN)
    add_executable(roam_demos_smoke ${ROAM_DEMO_SHARED_SOURCES} smoke/main.cpp smoke/sokol.cpp)
    target_include_directories(roam_demos_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${sokol_SOURCE_DIR} ${sokol_SOURCE_DIR}/util ${cgltf_SOURCE_DIR})
    target_link_libraries(roam_demos_smoke PRIVATE roam_blocks roam_demos_imgui)
    target_compile_definitions(roam_demos_smoke PRIVATE SOKOL_DUMMY_BACKEND ROAM_DEMOS_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
    roam_compile_options(roam_demos_smoke)
    add_test(NAME roam_demos_smoke COMMAND roam_demos_smoke)
endif()
