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
)
target_include_directories(flycore PUBLIC src)

add_executable(flysim src/sim_main.cpp)
target_link_libraries(flysim PRIVATE flycore)

# Run from the project root so the default --data path resolves.
set_target_properties(flysim PROPERTIES
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

  # SDL3 is built as a DLL, so it has to sit beside the executable to run.
  add_custom_command(TARGET flyviz POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL3::SDL3> $<TARGET_FILE_DIR:flyviz>
    COMMENT "Copying SDL3 runtime next to flyviz")
endif()
