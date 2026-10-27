cmake_minimum_required(VERSION 3.24)

project(roam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROAM_BUILD_TESTS "build roam's tests" ${PROJECT_IS_TOP_LEVEL})
option(ROAM_BUILD_BENCH "build the benchmarks" ${PROJECT_IS_TOP_LEVEL})
option(ROAM_BUILD_DEMOS "build the demo app, fetching sokol, imgui and cgltf" OFF)
set(ROAM_NAVCAT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../navcat" CACHE PATH "navcat checkout used to generate parity fixtures")

function(roam_compile_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /GR- /EHs-c-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -fno-exceptions -fno-rtti -ffp-contract=off)
    endif()
endfunction()

function(roam_test_compile_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /EHsc)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -ffp-contract=off)
    endif()
endfunction()

file(GLOB_RECURSE ROAM_SOURCES CONFIGURE_DEPENDS roam/*.cpp)
list(FILTER ROAM_SOURCES EXCLUDE REGEX "/roam/blocks/")
add_library(roam ${ROAM_SOURCES})
add_library(roam::roam ALIAS roam)
target_include_directories(roam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(roam PUBLIC cxx_std_20)
roam_compile_options(roam)

file(GLOB_RECURSE ROAM_BLOCKS_SOURCES CONFIGURE_DEPENDS roam/blocks/*.cpp)
if(ROAM_BLOCKS_SOURCES)
    add_library(roam_blocks ${ROAM_BLOCKS_SOURCES})
    add_library(roam::blocks ALIAS roam_blocks)
    target_link_libraries(roam_blocks PUBLIC roam)
    roam_compile_options(roam_blocks)
endif()

if(ROAM_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.11)
    FetchContent_MakeAvailable(doctest)
    include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

    set(ROAM_TEST_HARNESS tests/harness/test.cpp tests/harness/main.cpp)

    file(GLOB_RECURSE ROAM_TEST_SOURCES CONFIGURE_DEPENDS tests/*.test.cpp)
    list(FILTER ROAM_TEST_SOURCES EXCLUDE REGEX "/tests/parity/")
    add_executable(roam_tests ${ROAM_TEST_HARNESS} ${ROAM_TEST_SOURCES})
    target_link_libraries(roam_tests PRIVATE roam_blocks doctest::doctest)
    roam_test_compile_options(roam_tests)
    doctest_discover_tests(roam_tests)

    file(GLOB ROAM_SNIPPET_SOURCES CONFIGURE_DEPENDS docs/snippets/*.cpp)
    list(REMOVE_ITEM ROAM_SNIPPET_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/docs/snippets/quickstart.cpp)
    add_executable(roam_snippets ${ROAM_SNIPPET_SOURCES})
    target_link_libraries(roam_snippets PRIVATE roam_blocks)
    roam_compile_options(roam_snippets)
    add_test(NAME roam_snippets COMMAND roam_snippets)

    add_executable(roam_snippet_quickstart docs/snippets/quickstart.cpp)
    target_link_libraries(roam_snippet_quickstart PRIVATE roam_blocks)
    roam_compile_options(roam_snippet_quickstart)
    add_test(NAME roam_snippet_quickstart COMMAND roam_snippet_quickstart)

    find_program(ROAM_NODE node)
    if(ROAM_NODE)
        add_test(NAME roam_readme COMMAND ${ROAM_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/docs/build.mjs --check)
    endif()
    if(ROAM_NODE AND EXISTS "${ROAM_NAVCAT_DIR}/package.json")
        set(ROAM_FIXTURES_DIR "${CMAKE_BINARY_DIR}/fixtures")
        set(ROAM_FIXTURES_STAMP "${ROAM_FIXTURES_DIR}/.stamp")
        add_custom_command(
            OUTPUT ${ROAM_FIXTURES_STAMP}
            COMMAND ${ROAM_NODE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/navcat-fixtures/dump.mjs --navcat ${ROAM_NAVCAT_DIR} --out ${ROAM_FIXTURES_DIR}
            COMMAND ${CMAKE_COMMAND} -E touch ${ROAM_FIXTURES_STAMP}
            DEPENDS tools/navcat-fixtures/dump.mjs "${ROAM_NAVCAT_DIR}/dist/index.js"
            COMMENT "dumping navcat fixtures"
            VERBATIM)
        add_custom_target(navcat_fixtures DEPENDS ${ROAM_FIXTURES_STAMP})

        file(GLOB_RECURSE ROAM_PARITY_SOURCES CONFIGURE_DEPENDS tests/parity/*.test.cpp)
        add_executable(roam_parity_tests ${ROAM_TEST_HARNESS} tests/harness/fixture.cpp ${ROAM_PARITY_SOURCES})
        target_link_libraries(roam_parity_tests PRIVATE roam_blocks doctest::doctest)
        target_compile_definitions(roam_parity_tests PRIVATE ROAM_FIXTURES_DIR="${ROAM_FIXTURES_DIR}")
        roam_test_compile_options(roam_parity_tests)
        add_dependencies(roam_parity_tests navcat_fixtures)
        doctest_discover_tests(roam_parity_tests)
    else()
        message(STATUS "roam: parity tests skipped, needs node and a navcat checkout at ROAM_NAVCAT_DIR (${ROAM_NAVCAT_DIR})")
    endif()
endif()

if(ROAM_BUILD_BENCH AND NOT EMSCRIPTEN)
    add_executable(roam_bench bench/bench.cpp bench/main.cpp)
    target_link_libraries(roam_bench PRIVATE roam_blocks)
    roam_compile_options(roam_bench)
endif()

if(ROAM_BUILD_DEMOS)
    add_subdirectory(demos)
endif()
