#pragma once

// Minimal OpenGL 3.3 core loader.
//
// Windows only exports OpenGL 1.1 from opengl32; anything newer has to come
// through the driver at runtime. Rather than take on glad or GLEW for the
// couple of dozen entry points this renderer uses, each one is declared here
// and filled in from SDL_GL_GetProcAddress.
//
// GL 1.1 calls (glClear, glDrawArrays, glEnable, ...) are linked directly from
// opengl32 and are not listed here.

#include <SDL3/SDL_opengl.h>

namespace fly::gl {

// name, PFN type -- keep these two in sync; the macro generates the pointer,
// the extern declaration and the load call from this one list.
#define FLY_GL_FUNCTIONS(X)                                        \
    X(glGenVertexArrays,          PFNGLGENVERTEXARRAYSPROC)        \
    X(glBindVertexArray,          PFNGLBINDVERTEXARRAYPROC)        \
    X(glDeleteVertexArrays,       PFNGLDELETEVERTEXARRAYSPROC)     \
    X(glGenBuffers,               PFNGLGENBUFFERSPROC)             \
    X(glBindBuffer,               PFNGLBINDBUFFERPROC)             \
    X(glBufferData,               PFNGLBUFFERDATAPROC)             \
    X(glBufferSubData,            PFNGLBUFFERSUBDATAPROC)          \
    X(glDeleteBuffers,            PFNGLDELETEBUFFERSPROC)          \
    X(glVertexAttribPointer,      PFNGLVERTEXATTRIBPOINTERPROC)    \
    X(glEnableVertexAttribArray,  PFNGLENABLEVERTEXATTRIBARRAYPROC)\
    X(glCreateShader,             PFNGLCREATESHADERPROC)           \
    X(glShaderSource,             PFNGLSHADERSOURCEPROC)           \
    X(glCompileShader,            PFNGLCOMPILESHADERPROC)          \
    X(glGetShaderiv,              PFNGLGETSHADERIVPROC)            \
    X(glGetShaderInfoLog,         PFNGLGETSHADERINFOLOGPROC)       \
    X(glDeleteShader,             PFNGLDELETESHADERPROC)           \
    X(glCreateProgram,            PFNGLCREATEPROGRAMPROC)          \
    X(glAttachShader,             PFNGLATTACHSHADERPROC)           \
    X(glLinkProgram,              PFNGLLINKPROGRAMPROC)            \
    X(glGetProgramiv,             PFNGLGETPROGRAMIVPROC)           \
    X(glGetProgramInfoLog,        PFNGLGETPROGRAMINFOLOGPROC)      \
    X(glUseProgram,               PFNGLUSEPROGRAMPROC)             \
    X(glDeleteProgram,            PFNGLDELETEPROGRAMPROC)          \
    X(glGetUniformLocation,       PFNGLGETUNIFORMLOCATIONPROC)     \
    X(glUniformMatrix4fv,         PFNGLUNIFORMMATRIX4FVPROC)       \
    X(glUniform1f,                PFNGLUNIFORM1FPROC)              \
    X(glUniform3f,                PFNGLUNIFORM3FPROC)

#define FLY_GL_DECLARE(name, type) extern type name;
FLY_GL_FUNCTIONS(FLY_GL_DECLARE)
#undef FLY_GL_DECLARE

// Resolve every entry point against the current GL context. Returns false and
// writes the first missing name to `missing` if the driver does not supply one.
bool load(const char** missing);

}  // namespace fly::gl
