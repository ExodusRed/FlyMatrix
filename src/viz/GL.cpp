#include "GL.h"

#include <SDL3/SDL.h>

namespace fly::gl {

#define FLY_GL_DEFINE(name, type) type name = nullptr;
FLY_GL_FUNCTIONS(FLY_GL_DEFINE)
#undef FLY_GL_DEFINE

bool load(const char** missing) {
#define FLY_GL_LOAD(name, type)                                            \
    name = reinterpret_cast<type>(SDL_GL_GetProcAddress(#name));           \
    if (!name) {                                                           \
        if (missing) *missing = #name;                                     \
        return false;                                                      \
    }
    FLY_GL_FUNCTIONS(FLY_GL_LOAD)
#undef FLY_GL_LOAD
    return true;
}

}  // namespace fly::gl
