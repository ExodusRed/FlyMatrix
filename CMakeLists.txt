cmake_minimum_required(VERSION 3.21)
project(FlyBrain LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# The simulation inner loop is the whole point; make sure it is optimised even
# when the generator defaults elsewhere.
if(MSVC)
  add_compile_options(/W4 /permissive- /Zc:__cplusplus)
  add_compile_options($<$<CONFIG:Release>:/O2> $<$<CONFIG:Release>:/fp:fast>)
else()
  add_compile_options(-Wall -Wextra)
  add_compile_options($<$<CONFIG:Release>:-O3>)
endif()

add_library(flycore STATIC
  src/core/Connectome.cpp
  src/core/LIFNetwork.cpp
  src/core/DataPath.cpp
  src/core/NeuronNames.cpp
  src/body/FlyBody.cpp
  src/body/FlyPhysics.cpp
  src/body/MotorPools.cpp
  src/body/SensoryOrgans.cpp
  src/body/WalkController.cpp
  src/kin/KinematicFly.cpp
  src/engine/Physics.cpp
)
target_include_directories(flycore PUBLIC src)

# ---------------------------------------------------------------------------
# Browser build (Emscripten): only the body, running live as WebAssembly on
# drive recorded by flyrecord. Configure with
#   emcmake cmake -S . -B build-web && cmake --build build-web
# See web/README.md.
# ---------------------------------------------------------------------------
if(EMSCRIPTEN)
  set(FLYBRAIN_SDL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Library/SDL"
      CACHE PATH "SDL3 source directory")
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory("${FLYBRAIN_SDL_DIR}" "${CMAKE_BINARY_DIR}/SDL" EXCLUDE_FROM_ALL)
  # Our -Wall -Wextra is for our code; SDL's own warnings are not ours to fix.
  target_compile_options(SDL3-static PRIVATE -w)

  # The engine reports errors with exceptions; native WebAssembly exception
  # handling keeps that working at almost no cost.
  target_compile_options(flycore PUBLIC -fwasm-exceptions)
  target_link_options(flycore PUBLIC -fwasm-exceptions)

  set(FLY_WEB_DATA "${CMAKE_CURRENT_SOURCE_DIR}/web/data")
  add_executable(flybody_web
    src/web/body_web.cpp
    src/engine/Mesh.cpp
  )
  target_link_libraries(flybody_web PRIVATE flycore SDL3::SDL3-static)
  # Straight into web/body/, so web/ is a complete site that can be served as-is.
  set_target_properties(flybody_web PROPERTIES OUTPUT_NAME "flybody" SUFFIX ".js"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/web/body")
  target_link_options(flybody_web PRIVATE
    -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web
    -sEXPORTED_FUNCTIONS=_main,_fly_jump,_fly_pause,_fly_speed,_fly_toggle,_fly_status,_fly_selftest,_fly_mode,_fly_walk,_fly_timescale
    -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,UTF8ToString
    "--preload-file=${FLY_WEB_DATA}/drive.bin@/data/drive.bin"
    "--preload-file=${FLY_WEB_DATA}/motor_map.tsv@/data/motor_map.tsv"
  )
  return()
endif()

# Mechanics-only harness. Links flycore but touches no connectome data, so it
# runs in milliseconds and isolates the body from the nervous system.
add_executable(flyphys src/phys_test.cpp)
target_link_libraries(flyphys PRIVATE flycore)
set_target_properties(flyphys PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

add_executable(flysim src/sim_main.cpp)
target_link_libraries(flysim PRIVATE flycore)

# Run from the project root so the default --data path resolves.
set_target_properties(flysim PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Kinematic walker tests (variant B2, no physics).
add_executable(flykin src/kin/kin_main.cpp)
target_link_libraries(flykin PRIVATE flycore)

# Records runs for the web build (see web/README.md). Native only: it needs
# the full connectome, which is exactly what the browser cannot load.
add_executable(flyrecord src/web/record_main.cpp)
target_link_libraries(flyrecord PRIVATE flycore)
set_target_properties(flyrecord PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# 3D renderer (SDL3 + OpenGL 3.3)
# ---------------------------------------------------------------------------
option(FLYBRAIN_VIZ "Build the 3D renderer" ON)

# Point this at an SDL3 checkout. The default picks up the sibling copy in
# MATRIX/Library; build artefacts go into our own build tree, so the SDL
# checkout is never written to.
set(FLYBRAIN_SDL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Library/SDL"
    CACHE PATH "SDL3 source directory")

if(FLYBRAIN_VIZ)
  if(EXISTS "${FLYBRAIN_SDL_DIR}/CMakeLists.txt")
    set(SDL_SHARED ON CACHE BOOL "" FORCE)
    set(SDL_STATIC OFF CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
    add_subdirectory("${FLYBRAIN_SDL_DIR}" "${CMAKE_BINARY_DIR}/SDL" EXCLUDE_FROM_ALL)
    message(STATUS "FlyBrain: building SDL3 from ${FLYBRAIN_SDL_DIR}")
  else()
    find_package(SDL3 CONFIG QUIET)
    if(NOT SDL3_FOUND)
      message(WARNING
        "FlyBrain: no SDL3 at ${FLYBRAIN_SDL_DIR} and none installed; "
        "skipping the renderer. Set -DFLYBRAIN_SDL_DIR=<path> to point at one.")
      set(FLYBRAIN_VIZ OFF)
    endif()
  endif()
endif()

if(FLYBRAIN_VIZ)
  find_package(OpenGL REQUIRED)

  add_executable(flyviz
    src/viz/viz_main.cpp
    src/viz/GL.cpp
  )
  target_link_libraries(flyviz PRIVATE flycore SDL3::SDL3 OpenGL::GL)
  set_target_properties(flyviz PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

  add_executable(flybody
    src/body_main.cpp
    src/engine/Mesh.cpp
    src/viz/GL.cpp
  )
  target_link_libraries(flybody PRIVATE flycore SDL3::SDL3 OpenGL::GL)
  set_target_properties(flybody PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

  # SDL3 is built as a DLL, so it has to sit beside the executable to run.
  add_custom_command(TARGET flyviz POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL3::SDL3> $<TARGET_FILE_DIR:flyviz>
    COMMENT "Copying SDL3 runtime next to flyviz")
endif()
