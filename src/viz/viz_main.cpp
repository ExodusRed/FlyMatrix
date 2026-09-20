// flyviz -- interactive 3D view of the simulation.
//
// Draws all 176k neurons as points in their real anatomical positions and
// lights them as they fire. The simulation runs a configurable number of steps
// per frame, so what you see is slow motion: the whole escape response lasts a
// few milliseconds of fly time and would be over in three frames at true speed.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
// windows.h defines min/max as macros, which breaks every std::max in this
// file; NOMINMAX suppresses them.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <SDL3/SDL.h>

#include "core/Connectome.h"
#include "core/DataPath.h"
#include "core/LIFNetwork.h"
#include "core/NeuronNames.h"
#include "viz/Camera.h"
#include "viz/GL.h"

using namespace fly;
using namespace fly::viz;

namespace {

constexpr const char* kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aActivity;

uniform mat4 uViewProj;
uniform vec3 uEye;
uniform float uPointScale;
uniform float uDimAlpha;

out float vActivity;
out float vAlpha;

void main() {
    gl_Position = uViewProj * vec4(aPos, 1.0);
    float dist = length(aPos - uEye);
    // Active neurons are drawn larger so a sparse response stays visible when
    // zoomed out, where a single pixel would otherwise vanish.
    float size = uPointScale * (1.0 + 5.0 * aActivity) / max(dist, 1.0);
    gl_PointSize = clamp(size, 1.0, 64.0);
    vActivity = aActivity;
    vAlpha = mix(uDimAlpha, 1.0, aActivity);
}
)";

constexpr const char* kFragmentShader = R"(#version 330 core
in float vActivity;
in float vAlpha;
out vec4 fragColour;

void main() {
    // Round, soft-edged points. gl_PointCoord runs 0..1 across the sprite.
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) discard;
    float falloff = 1.0 - smoothstep(0.0, 0.25, r2);

    // Cold slate when at rest, through cyan, to hot white at full activity.
    vec3 rest   = vec3(0.22, 0.34, 0.62);
    vec3 mid    = vec3(0.20, 0.85, 0.95);
    vec3 hot    = vec3(1.00, 0.96, 0.80);
    float a = clamp(vActivity, 0.0, 1.0);
    vec3 colour = a < 0.5 ? mix(rest, mid, a * 2.0)
                          : mix(mid, hot, (a - 0.5) * 2.0);

    fragColour = vec4(colour * falloff, vAlpha * falloff);
}
)";

GLuint compile(GLenum type, const char* src, const char* label) {
    const GLuint sh = gl::glCreateShader(type);
    gl::glShaderSource(sh, 1, &src, nullptr);
    gl::glCompileShader(sh);
    GLint ok = 0;
    gl::glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        gl::glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
        std::fprintf(stderr, "%s shader failed to compile:\n%s\n", label, log);
        return 0;
    }
    return sh;
}

GLuint buildProgram() {
    const GLuint vs = compile(GL_VERTEX_SHADER, kVertexShader, "vertex");
    const GLuint fs = compile(GL_FRAGMENT_SHADER, kFragmentShader, "fragment");
    if (!vs || !fs) return 0;

    const GLuint prog = gl::glCreateProgram();
    gl::glAttachShader(prog, vs);
    gl::glAttachShader(prog, fs);
    gl::glLinkProgram(prog);
    GLint ok = 0;
    gl::glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        gl::glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        std::fprintf(stderr, "shader link failed:\n%s\n", log);
        return 0;
    }
    gl::glDeleteShader(vs);
    gl::glDeleteShader(fs);
    return prog;
}

// Read the framebuffer back into a BMP. Used by --screenshot so a render can
// be checked without a human watching the window.
bool saveScreenshot(const std::string& path, int w, int h) {
    std::vector<unsigned char> pixels(static_cast<std::size_t>(w) * h * 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    // OpenGL reads bottom-up; SDL surfaces are top-down.
    const int stride = w * 4;
    std::vector<unsigned char> flipped(pixels.size());
    for (int y = 0; y < h; ++y) {
        std::memcpy(&flipped[static_cast<std::size_t>(y) * stride],
                    &pixels[static_cast<std::size_t>(h - 1 - y) * stride],
                    static_cast<std::size_t>(stride));
    }

    SDL_Surface* surf = SDL_CreateSurfaceFrom(w, h, SDL_PIXELFORMAT_ABGR8888,
                                              flipped.data(), stride);
    if (!surf) {
        std::fprintf(stderr, "SDL_CreateSurfaceFrom failed: %s\n", SDL_GetError());
        return false;
    }
    const bool ok = SDL_SaveBMP(surf, path.c_str());
    if (!ok) std::fprintf(stderr, "SDL_SaveBMP failed: %s\n", SDL_GetError());
    SDL_DestroySurface(surf);
    return ok;
}

void printHelp() {
    std::printf(
        "flyviz -- 3D view of the male-cns:v1.0 simulation\n\n"
        "  --data DIR         directory holding cns.bin (default: data/bin)\n"
        "  --stim-body ID     neuron to pulse (default: 10001, the giant fibre)\n"
        "  --stim-type NAME   pulse every neuron of this cell type instead\n"
        "  --epsp MV          depolarisation per synapse (default: 0.085)\n"
        "  --steps N          simulation steps per frame (default: 8)\n"
        "  --pulse MS         length of each stimulus pulse (default: 15)\n"
        "  --frames N         render N frames then exit (for testing)\n"
        "  --screenshot FILE  save a BMP of the last frame before exiting\n\n"
        "controls\n"
        "  left drag      orbit            scroll     zoom\n"
        "  right drag     pan              space      fire the stimulus again\n"
        "  [ / ]          slower / faster  r          reset the simulation\n"
        "  s              toggle soma-only view        esc  quit\n");
}

}  // namespace

int run(int argc, char** argv) {
    std::string dataDir = "data/bin";
    std::int64_t stimBody = 10001;
    std::string stimType;
    LifParams params;
    int stepsPerFrame = 8;
    float pulseMs = 15.0f;
    long frameLimit = 0;  // 0 = run until closed
    std::string shotPath;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", what);
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { printHelp(); return 0; }
        else if (a == "--data") dataDir = next("--data");
        else if (a == "--stim-body") stimBody = std::atoll(next("--stim-body").c_str());
        else if (a == "--stim-type") stimType = next("--stim-type");
        else if (a == "--epsp") params.epspPerSynapse = std::stof(next("--epsp"));
        else if (a == "--steps") stepsPerFrame = std::atoi(next("--steps").c_str());
        else if (a == "--pulse") pulseMs = std::stof(next("--pulse").c_str());
        else if (a == "--frames") frameLimit = std::atol(next("--frames").c_str());
        else if (a == "--screenshot") shotPath = next("--screenshot");
        else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 1;
        }
    }

    dataDir = findDataDir(dataDir, argv[0]);
    Connectome conn = Connectome::load(dataDir + "/cns.bin");
    std::printf("loaded %u neurons, %llu edges\n", conn.neuronCount(),
                static_cast<unsigned long long>(conn.edgeCount()));

    // Resolve what to stimulate. Types need the name sidecar; a bodyId does not.
    std::vector<std::uint32_t> driven;
    if (!stimType.empty()) {
        bool haveNames = false;
        const auto names = NeuronNames::load(dataDir + "/cns_names.tsv",
                                             conn.neuronCount(), &haveNames);
        if (!haveNames) {
            std::fprintf(stderr, "cns_names.tsv not found, cannot resolve --stim-type\n");
            return 1;
        }
        const auto of = names.ofType(stimType);
        if (of.empty()) {
            std::fprintf(stderr, "no cell type named %s\n", stimType.c_str());
            return 1;
        }
        driven.assign(of.begin(), of.end());
    } else {
        const auto idx = conn.indexOf(stimBody);
        if (idx == UINT32_MAX) {
            std::fprintf(stderr, "no neuron with bodyId %lld\n",
                         static_cast<long long>(stimBody));
            return 1;
        }
        driven.push_back(idx);
    }

    std::printf("stimulus drives %zu neuron(s)\n", driven.size());

    LIFNetwork net(conn, params);

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow("FlyBrain", 1600, 950,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    if (!ctx) {
        std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_GL_SetSwapInterval(1);

    const char* missing = nullptr;
    if (!gl::load(&missing)) {
        std::fprintf(stderr, "this driver does not provide %s -- OpenGL 3.3 required\n",
                     missing);
        SDL_GL_DestroyContext(ctx);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    const GLuint prog = buildProgram();
    if (!prog) {
        SDL_GL_DestroyContext(ctx);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    const std::uint32_t n = conn.neuronCount();

    GLuint vao = 0, posVbo = 0, actVbo = 0;
    gl::glGenVertexArrays(1, &vao);
    gl::glBindVertexArray(vao);

    gl::glGenBuffers(1, &posVbo);
    gl::glBindBuffer(GL_ARRAY_BUFFER, posVbo);
    gl::glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(n * sizeof(Vec3)),
                     conn.positions().data(), GL_STATIC_DRAW);
    gl::glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
    gl::glEnableVertexAttribArray(0);

    std::vector<float> activity(n, 0.0f);
    gl::glGenBuffers(1, &actVbo);
    gl::glBindBuffer(GL_ARRAY_BUFFER, actVbo);
    gl::glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(n * sizeof(float)),
                     activity.data(), GL_STREAM_DRAW);
    gl::glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float), nullptr);
    gl::glEnableVertexAttribArray(1);

    const GLint locViewProj = gl::glGetUniformLocation(prog, "uViewProj");
    const GLint locEye = gl::glGetUniformLocation(prog, "uEye");
    const GLint locPointScale = gl::glGetUniformLocation(prog, "uPointScale");
    const GLint locDimAlpha = gl::glGetUniformLocation(prog, "uDimAlpha");

    OrbitCamera cam;
    const auto lo = conn.bboxMin();
    const auto hi = conn.bboxMax();
    cam.target = {(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f,
                  (lo.z + hi.z) * 0.5f};
    const float extent = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
    cam.distance = extent * 1.3f;

    bool running = true, dragL = false, dragR = false, somaOnly = false;
    bool stimulating = false;
    float stimRemainingMs = 0.0f;
    std::uint64_t frame = 0;
    double lastTitle = 0.0;
    std::uint64_t spikesSinceTitle = 0;

    auto fireStimulus = [&]() {
        for (const auto i : driven) net.setStimulus(i, 200.0f);
        stimulating = true;
        stimRemainingMs = pulseMs;
    };
    fireStimulus();

    const std::uint64_t perfFreq = SDL_GetPerformanceFrequency();
    std::uint64_t lastCounter = SDL_GetPerformanceCounter();

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;
                case SDL_EVENT_KEY_DOWN:
                    if (ev.key.key == SDLK_ESCAPE) running = false;
                    else if (ev.key.key == SDLK_SPACE) fireStimulus();
                    else if (ev.key.key == SDLK_R) {
                        net.reset();
                        std::fill(activity.begin(), activity.end(), 0.0f);
                        fireStimulus();
                    }
                    else if (ev.key.key == SDLK_S) somaOnly = !somaOnly;
                    else if (ev.key.key == SDLK_LEFTBRACKET)
                        stepsPerFrame = std::max(1, stepsPerFrame - 1);
                    else if (ev.key.key == SDLK_RIGHTBRACKET)
                        stepsPerFrame = std::min(200, stepsPerFrame + 1);
                    break;
                case SDL_EVENT_MOUSE_BUTTON_DOWN:
                    if (ev.button.button == SDL_BUTTON_LEFT) dragL = true;
                    if (ev.button.button == SDL_BUTTON_RIGHT) dragR = true;
                    break;
                case SDL_EVENT_MOUSE_BUTTON_UP:
                    if (ev.button.button == SDL_BUTTON_LEFT) dragL = false;
                    if (ev.button.button == SDL_BUTTON_RIGHT) dragR = false;
                    break;
                case SDL_EVENT_MOUSE_MOTION:
                    if (dragL) cam.orbit(-ev.motion.xrel * 0.006f,
                                          ev.motion.yrel * 0.006f);
                    else if (dragR) cam.pan(ev.motion.xrel, ev.motion.yrel);
                    break;
                case SDL_EVENT_MOUSE_WHEEL:
                    cam.zoom(ev.wheel.y > 0 ? 0.88f : 1.136f);
                    break;
                default:
                    break;
            }
        }

        // --- simulate ---
        std::uint32_t spikesThisFrame = 0;
        for (int s = 0; s < stepsPerFrame; ++s) {
            const auto st = net.step();
            spikesThisFrame += st.spikeCount;
            for (const auto j : net.lastSpikes()) activity[j] = 1.0f;

            if (stimulating) {
                stimRemainingMs -= params.dtMs;
                if (stimRemainingMs <= 0.0f) {
                    net.clearStimulus();
                    stimulating = false;
                }
            }
        }
        spikesSinceTitle += spikesThisFrame;

        // Decay the spike trace so a flash fades rather than latching on. Tied
        // to steps simulated, not to frames, so the visual decay rate does not
        // change when you speed the simulation up.
        const float decay = std::pow(0.90f, static_cast<float>(stepsPerFrame));
        const auto gradedOut = net.gradedOutput();
        for (std::uint32_t i = 0; i < n; ++i) activity[i] *= decay;
        // Graded neurons never spike, so show their continuous output instead.
        for (const auto j : net.gradedNeurons()) {
            activity[j] = std::max(activity[j], std::min(1.0f, gradedOut[j] / 80.0f));
        }

        // --- draw ---
        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.02f, 0.025f, 0.04f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Additive blending with depth testing off: the cloud reads as a
        // volume, and dense regions accumulate into a glow rather than the
        // nearest point hiding everything behind it.
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glEnable(GL_PROGRAM_POINT_SIZE);

        gl::glBindBuffer(GL_ARRAY_BUFFER, actVbo);
        // Orphan the buffer so the driver can hand back fresh storage instead
        // of stalling until the previous frame's draw has finished reading it.
        gl::glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(n * sizeof(float)),
                         nullptr, GL_STREAM_DRAW);
        gl::glBufferSubData(GL_ARRAY_BUFFER, 0,
                            static_cast<GLsizeiptr>(n * sizeof(float)),
                            activity.data());

        gl::glUseProgram(prog);
        const float aspect = h > 0 ? static_cast<float>(w) / static_cast<float>(h) : 1.0f;
        const M4 vp = cam.viewProjection(aspect);
        gl::glUniformMatrix4fv(locViewProj, 1, GL_FALSE, vp.m);
        const V3 eye = cam.eye();
        gl::glUniform3f(locEye, eye.x, eye.y, eye.z);
        // Pixels subtended by a 1um object one unit from the eye; the shader
        // divides by distance to get the real size.
        constexpr float kSomaUm = 3.0f;
        const float pxPerUm = static_cast<float>(h) /
                              (2.0f * std::tan(cam.fovy * 0.5f));
        gl::glUniform1f(locPointScale, kSomaUm * pxPerUm);
        gl::glUniform1f(locDimAlpha, somaOnly ? 0.0f : 0.30f);

        gl::glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(n));

        const bool lastFrame =
            frameLimit > 0 && frame + 1 >= static_cast<std::uint64_t>(frameLimit);
        if (lastFrame && !shotPath.empty()) {
            if (saveScreenshot(shotPath, w, h)) {
                std::printf("wrote %s (%dx%d)\n", shotPath.c_str(), w, h);
            }
        }
        SDL_GL_SwapWindow(window);

        // --- status in the title bar ---
        const std::uint64_t now = SDL_GetPerformanceCounter();
        const double dt = static_cast<double>(now - lastCounter) / perfFreq;
        lastCounter = now;
        lastTitle += dt;
        if (lastTitle > 0.25) {
            char title[256];
            std::snprintf(title, sizeof(title),
                          "FlyBrain  |  t = %.1f ms  |  %.0f spikes/s  |  "
                          "%d steps/frame  |  %.0f fps%s",
                          net.timeMs(), spikesSinceTitle / lastTitle, stepsPerFrame,
                          1.0 / std::max(dt, 1e-6), stimulating ? "  |  STIM" : "");
            SDL_SetWindowTitle(window, title);
            lastTitle = 0.0;
            spikesSinceTitle = 0;
        }
        ++frame;
        if (frameLimit > 0 && frame >= static_cast<std::uint64_t>(frameLimit)) {
            running = false;
        }
    }

    gl::glDeleteBuffers(1, &posVbo);
    gl::glDeleteBuffers(1, &actVbo);
    gl::glDeleteVertexArrays(1, &vao);
    gl::glDeleteProgram(prog);
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

namespace {

// True when this process created its own console -- which is what happens when
// a console application is double-clicked, and means the window will vanish
// with the process before anything printed to it can be read. Launched from an
// existing shell the console belongs to that shell, and the text stays put.
bool ownsItsConsole() {
#ifdef _WIN32
    HWND console = GetConsoleWindow();
    if (!console) return false;
    DWORD consolePid = 0;
    GetWindowThreadProcessId(console, &consolePid);
    return consolePid == GetCurrentProcessId();
#else
    return false;
#endif
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        // Only pop a dialog when the message would otherwise be lost, so
        // terminal and scripted runs are not left waiting on a click.
        if (ownsItsConsole()) {
            SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "FlyBrain",
                                     e.what(), nullptr);
        }
        return 1;
    }
}
