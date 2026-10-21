cmake_minimum_required(VERSION 3.24)

project(recast_bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RECAST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../recastnavigation" CACHE PATH "recastnavigation checkout")

if(NOT EXISTS "${RECAST_DIR}/Recast/Source/Recast.cpp")
    message(FATAL_ERROR "recast_bench: no recastnavigation checkout at RECAST_DIR (${RECAST_DIR})")
endif()

file(GLOB RECAST_SOURCES
    ${RECAST_DIR}/Recast/Source/*.cpp
    ${RECAST_DIR}/Detour/Source/*.cpp
    ${RECAST_DIR}/DetourCrowd/Source/*.cpp)

add_executable(recast_bench main.cpp ${RECAST_SOURCES} ${RECAST_DIR}/RecastDemo/Source/ChunkyTriMesh.cpp)
target_include_directories(recast_bench PRIVATE
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
    ${RECAST_DIR}/DetourCrowd/Include
    ${RECAST_DIR}/RecastDemo/Include)
target_compile_options(recast_bench PRIVATE -O3 -DNDEBUG -w)
