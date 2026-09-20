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
)
target_include_directories(flycore PUBLIC src)

add_executable(flysim src/sim_main.cpp)
target_link_libraries(flysim PRIVATE flycore)

# Run from the project root so the default --data path resolves.
set_target_properties(flysim PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
