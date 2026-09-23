// flybody -- the fly's legs, driven by its own motor neurons.
//
// Milestone one of the sandbox: no physics yet. The connectome simulation runs,
// motor neuron spikes are filtered into muscle activations, and those set the
// joint angles of a kinematic skeleton. Stimulate a motor pool and the matching
// joint on the matching leg moves, because the connectome says which neuron
// drives which muscle on which leg.

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <algorithm>
#include <cmath>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "body/FlyBody.h"
#include "body/FlyPhysics.h"
#include "body/MotorPools.h"
#include "body/SensoryOrgans.h"
#include "core/Connectome.h"
#include "core/DataPath.h"
#include "core/LIFNetwork.h"
#include "core/NeuronNames.h"
#include "engine/Math.h"
#include "engine/Mesh.h"
#include "viz/Camera.h"
#include "viz/GL.h"

using namespace fly;

namespace {

constexpr const char* kVert = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uViewProj;
uniform mat4 uModel;
uniform mat4 uNormalMat;

out vec3 vNormal;
out vec3 vWorld;

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = normalize(mat3(uNormalMat) * aNormal);
    gl_Position = uViewProj * world;
}
)";

constexpr const char* kFrag = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorld;
out vec4 fragColour;

uniform vec3 uColour;
uniform vec3 uEye;
uniform float uGlow;

void main() {
    vec3 n = normalize(vNormal);
    vec3 lightDir = normalize(vec3(0.4, 0.7, 1.0));
    float diffuse = max(dot(n, lightDir), 0.0);
    // A dim fill from below keeps the underside of the body readable rather
    // than letting it go to pure black.
    float fill = 0.28 + 0.22 * max(dot(n, vec3(0.0, 0.0, -1.0)), 0.0);

    vec3 viewDir = normalize(uEye - vWorld);
    float spec = pow(max(dot(reflect(-lightDir, n), viewDir), 0.0), 24.0);

    vec3 base = uColour * (fill + 0.75 * diffuse) + vec3(0.9) * spec * 0.25;
    // Active muscles push their segment toward hot white.
    vec3 hot = vec3(1.0, 0.75, 0.35);
    fragColour = vec4(mix(base, hot, clamp(uGlow, 0.0, 1.0)), 1.0);
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
        throw std::runtime_error(std::string(label) + " shader: " + log);
    }
    return sh;
}

GLuint buildProgram() {
    const GLuint vs = compile(GL_VERTEX_SHADER, kVert, "vertex");
    const GLuint fs = compile(GL_FRAGMENT_SHADER, kFrag, "fragment");
    const GLuint prog = gl::glCreateProgram();
    gl::glAttachShader(prog, vs);
    gl::glAttachShader(prog, fs);
    gl::glLinkProgram(prog);
    GLint ok = 0;
    gl::glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        gl::glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        throw std::runtime_error(std::string("link: ") + log);
    }
    gl::glDeleteShader(vs);
    gl::glDeleteShader(fs);
    return prog;
}

// Rotation taking -Z onto `dir`, so a unit cylinder can be aimed down a leg
// segment.
Quat aimDownZ(const V3& dir) {
    const V3 d = normalise(dir);
    const V3 from{0, 0, -1};
    const float c = dot(from, d);
    if (c > 0.99999f) return {};
    if (c < -0.99999f) return Quat::axisAngle({1, 0, 0}, 3.14159265f);
    const V3 axis = cross(from, d);
    return Quat::axisAngle(axis, std::acos(std::clamp(c, -1.0f, 1.0f)));
}

void printHelp() {
    std::printf(
        "flybody -- the fly's legs driven by its own motor neurons\n\n"
        "  --data DIR         directory holding cns.bin (default: data/bin)\n"
        "  --stim-type NAME   cell type to drive instead of a single neuron\n"
        "  --stim-body ID     neuron to drive (default: 10001, the giant fibre)\n"
        "  --epsp MV          depolarisation per synapse (default: 0.085)\n"
        "  --steps N          simulation steps per frame (default: 8)\n"
        "  --pulse MS         stimulus pulse length, 0 for continuous (default: 0)\n"
        "  --frames N         render N frames then exit (for testing)\n"
        "  --screenshot FILE  save a BMP of the last frame\n"
        "  --dump-pose        print the rest pose and exit\n"
        "  --no-physics       set joint angles directly instead of simulating\n"
        "  --drop MS          run physics headlessly for MS and report\n"
        "  --muscle-torque X  peak torque per muscle pool\n\n"
        "controls\n"
        "  left drag   orbit          scroll  zoom         right drag  pan\n"
        "  space       fire stimulus  r       reset        esc         quit\n");
}

bool ownsItsConsole() {
#ifdef _WIN32
    HWND console = GetConsoleWindow();
    if (!console) return false;
    DWORD pid = 0;
    GetWindowThreadProcessId(console, &pid);
    return pid == GetCurrentProcessId();
#else
    return false;
#endif
}

bool saveScreenshot(const std::string& path, int w, int h) {
    std::vector<unsigned char> px(static_cast<std::size_t>(w) * h * 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    const int stride = w * 4;
    std::vector<unsigned char> flip(px.size());
    for (int y = 0; y < h; ++y) {
        std::copy_n(&px[static_cast<std::size_t>(h - 1 - y) * stride], stride,
                    &flip[static_cast<std::size_t>(y) * stride]);
    }
    SDL_Surface* s = SDL_CreateSurfaceFrom(w, h, SDL_PIXELFORMAT_ABGR8888,
                                           flip.data(), stride);
    if (!s) return false;
    const bool ok = SDL_SaveBMP(s, path.c_str());
    SDL_DestroySurface(s);
    return ok;
}

int run(int argc, char** argv) {
    std::string dataDir = "data/bin";
    std::string stimType;
    std::int64_t stimBody = 10001;  // DNp01, the giant fibre
    LifParams params;
    int stepsPerFrame = 8;
    float pulseMs = 20.0f;
    long frameLimit = 0;
    std::string shotPath;
    bool dumpPose = false;
    bool usePhysics = true;
    float dropMs = 0.0f;
    float muscleTorque = -1.0f;
    int forceJoint = -1;
    float forceDrive = 0.0f;
    float corrVel = -1.0f;
    // Off by default. The loop is implemented and measurable, but it
    // degrades everything it touches: with it on the giant fibre stops
    // jumping and a Kenyon cell starts moving the fly, which inverts the
    // specificity result the rest of the project rests on. Opt in with
    // --sensory to study it; see the README for why it does not work yet.
    bool useSensory = false;
    float extraLoad = 0.0f;
    float sensoryDrive = -1.0f;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + what);
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { printHelp(); return 0; }
        else if (a == "--data") dataDir = next("--data");
        else if (a == "--stim-type") stimType = next("--stim-type");
        else if (a == "--stim-body") { stimBody = std::atoll(next("--stim-body").c_str()); stimType.clear(); }
        else if (a == "--epsp") params.epspPerSynapse = std::stof(next("--epsp"));
        else if (a == "--steps") stepsPerFrame = std::atoi(next("--steps").c_str());
        else if (a == "--pulse") pulseMs = std::stof(next("--pulse"));
        else if (a == "--frames") frameLimit = std::atol(next("--frames").c_str());
        else if (a == "--screenshot") shotPath = next("--screenshot");
        else if (a == "--dump-pose") dumpPose = true;
        else if (a == "--no-physics") usePhysics = false;
        else if (a == "--drop") dropMs = std::stof(next("--drop"));
        else if (a == "--muscle-torque") muscleTorque = std::stof(next("--muscle-torque"));
        else if (a == "--force-joint") {
            const std::string jn = next("--force-joint");
            for (int j = 0; j < kJointCount; ++j) {
                if (jn == jointName(static_cast<Joint>(j))) forceJoint = j;
            }
            if (forceJoint < 0) throw std::runtime_error("unknown joint: " + jn);
        }
        else if (a == "--force-drive") forceDrive = std::stof(next("--force-drive"));
        else if (a == "--corr") corrVel = std::stof(next("--corr"));
        else if (a == "--sensory") useSensory = true;
        else if (a == "--no-sensory") useSensory = false;
        else if (a == "--load") extraLoad = std::stof(next("--load"));
        else if (a == "--sensory-drive") sensoryDrive = std::stof(next("--sensory-drive"));
        else throw std::runtime_error("unknown option: " + a);
    }

    dataDir = findDataDir(dataDir, argv[0]);
    const Connectome conn = Connectome::load(dataDir + "/cns.bin");
    bool haveNames = false;
    const auto names = NeuronNames::load(dataDir + "/cns_names.tsv",
                                         conn.neuronCount(), &haveNames);
    auto pools = MotorPools::load(dataDir + "/motor_map.tsv");

    // The return half of the loop. Optional only so the difference it makes
    // can be measured against its absence.
    SensoryOrgans::Params sensoryParams;
    if (sensoryDrive >= 0.0f) sensoryParams.maxDrive = sensoryDrive;
    SensoryOrgans sensory =
        SensoryOrgans::load(dataDir + "/sensory_map.tsv", sensoryParams);
    std::printf("proprioceptors: %zu chordotonal, %zu campaniform%s\n",
                sensory.chordotonalCount(), sensory.campaniformCount(),
                useSensory ? "" : "  (DISABLED)");
    std::printf("%u neurons, %zu motor neurons mapped to leg joints\n",
                conn.neuronCount(), pools.mappedNeurons());

    std::vector<std::uint32_t> driven;
    if (!stimType.empty()) {
        if (!haveNames) throw std::runtime_error("cns_names.tsv missing, cannot use --stim-type");
        const auto of = names.ofType(stimType);
        if (of.empty()) throw std::runtime_error("no cell type named " + stimType);
        driven.assign(of.begin(), of.end());
    } else {
        const auto idx = conn.indexOf(stimBody);
        if (idx == UINT32_MAX) throw std::runtime_error("no neuron with that bodyId");
        driven.push_back(idx);
    }
    std::printf("driving %zu neuron(s): %s\n", driven.size(),
                stimType.empty() ? "(by bodyId)" : stimType.c_str());

    LIFNetwork net(conn, params);
    FlyBody body;
    FlyPhysics phys;
    if (muscleTorque > 0.0f) phys.params.maxMuscleTorque = muscleTorque;
    phys.params.forceJoint = forceJoint;
    phys.params.forceDrive = forceDrive;
    if (usePhysics) phys.build(body);
    if (corrVel > 0.0f) phys.world.params.maxCorrectionVelocity = corrVel;

    if (dumpPose) {
        // A leg is only plausible if its foot ends up below the body and out
        // to its own side, and the two sides must be exact mirrors. Checking
        // that numerically beats squinting at a render and guessing which
        // joint is wrong.
        std::printf("body at z = %.3f mm\n\n", body.root.position.z);
        std::printf("%-10s %8s %8s %8s   %s\n",
                    "leg", "foot x", "foot y", "foot z", "check");
        bool allOk = true;
        for (int l = 0; l < kLegCount; ++l) {
            const auto id = static_cast<LegId>(l);
            const V3 f = body.footPosition(id);
            const float side = (l % 2 == 0) ? 1.0f : -1.0f;  // even index = left
            const bool below = f.z < body.root.position.z - 0.15f;
            const bool outward = f.y * side > 0.02f;
            allOk = allOk && below && outward;
            std::printf("%-10s %8.3f %8.3f %8.3f   %s%s\n", legName(id),
                        f.x, f.y, f.z,
                        below ? "" : "ABOVE BODY ",
                        outward ? (below ? "ok" : "") : "CROSSES MIDLINE");
        }
        // The mirror check is the one that catches a wrong rotation axis,
        // which "foot is below the body" happily passes.
        for (int l = 0; l < kLegCount; l += 2) {
            const V3 a = body.footPosition(static_cast<LegId>(l));
            const V3 b = body.footPosition(static_cast<LegId>(l + 1));
            const float err = std::fabs(a.x - b.x) + std::fabs(a.y + b.y) +
                              std::fabs(a.z - b.z);
            if (err > 1e-4f) {
                allOk = false;
                std::printf("  %s/%s are not mirrored (error %.4f mm)\n",
                            legName(static_cast<LegId>(l)),
                            legName(static_cast<LegId>(l + 1)), err);
            }
        }
        std::printf("\nrest pose %s\n", allOk ? "OK" : "FAILED");
        return allOk ? 0 : 2;
    }

    if (dropMs > 0.0f) {
        // Headless end-to-end run: nervous system, muscles and body together.
        // The neural model steps at 0.1 ms and the solver at 1 ms, so ten
        // neural steps feed each physics step.
        const float physDt = 0.001f;            // seconds
        const int neuralPerPhys = static_cast<int>(1.0f / params.dtMs);
        const int n = static_cast<int>(dropMs / 1000.0f / physDt);
        float stimLeft = (pulseMs > 0.0f) ? pulseMs : dropMs;
        for (const auto i : driven) net.setStimulus(i, 200.0f);

        std::printf("running %.0f ms: %zu driven neuron(s), physics at %.0f Hz\n",
                    dropMs, driven.size(), 1.0f / physDt);
        std::printf("%8s %10s %10s %9s %9s\n",
                    "t (ms)", "height", "peak", "contacts", "CTr act");
        for (int i = 0; i < n; ++i) {
            for (int k = 0; k < neuralPerPhys; ++k) {
                net.step();
                pools.accumulate(net);
                if (stimLeft > 0.0f) {
                    stimLeft -= params.dtMs;
                    // clearStimulus() wipes every stimulus, sensory included.
                    // sense() restores the proprioceptive drive below, so the
                    // loss lasts one step, but only the driven neurons should
                    // be silenced here.
                    if (stimLeft <= 0.0f) {
                        for (const auto d : driven) net.setStimulus(d, 0.0f);
                    }
                }
            }
            pools.update(1.0f);

            // A steady extra weight on the thorax. This is the perturbation
            // the proprioceptive loop is supposed to resist: with the loop
            // open the fly simply sags under it.
            if (extraLoad != 0.0f) {
                phys.world.bodies[0].force += V3{0, 0, -extraLoad};
            }
            phys.step(physDt, pools);

            // Body state drives the proprioceptors, which drive the network on
            // the next step. This is the only place anything flows backwards.
            if (useSensory) sensory.sense(phys, net, 1.0f);

            if (i % std::max(1, n / 12) == 0 || i == n - 1) {
                // Coxa-trochanter is where the jump muscle pulls.
                float ctr = 0.0f;
                for (int l = 0; l < kLegCount; ++l) {
                    ctr = std::max(ctr,
                        pools.peakActivation(l, static_cast<int>(Joint::CTr)));
                }
                std::printf("%8.0f %10.4f %10.4f %9zu %9.3f\n",
                            i * physDt * 1000.0f, phys.bodyHeight(),
                            phys.peakHeight(), phys.world.contacts.size(), ctr);
            }
            if (!std::isfinite(phys.bodyHeight()) ||
                std::fabs(phys.bodyHeight()) > 1e4f) {
                std::printf("\nSOLVER DIVERGED at %.0f ms\n", i * physDt * 1000.0f);
                return 2;
            }
        }
        std::printf("\nfinal %.4f mm, peak %.4f mm\n",
                    phys.bodyHeight(), phys.peakHeight());
        return 0;
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) throw std::runtime_error(SDL_GetError());
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    SDL_Window* window = SDL_CreateWindow("FlyBody", 1600, 950,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) throw std::runtime_error(SDL_GetError());
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    if (!ctx) throw std::runtime_error(SDL_GetError());
    SDL_GL_SetSwapInterval(1);

    const char* missing = nullptr;
    if (!gl::load(&missing)) {
        throw std::runtime_error(std::string("OpenGL 3.3 required, missing ") + missing);
    }

    const GLuint prog = buildProgram();
    Mesh cyl = Mesh::cylinder(14);
    Mesh sph = Mesh::sphere(14, 20);
    cyl.upload();
    sph.upload();

    const GLint uViewProj = gl::glGetUniformLocation(prog, "uViewProj");
    const GLint uModel = gl::glGetUniformLocation(prog, "uModel");
    const GLint uNormalMat = gl::glGetUniformLocation(prog, "uNormalMat");
    const GLint uColour = gl::glGetUniformLocation(prog, "uColour");
    const GLint uEyeLoc = gl::glGetUniformLocation(prog, "uEye");
    const GLint uGlow = gl::glGetUniformLocation(prog, "uGlow");

    viz::OrbitCamera cam;
    cam.zUp = true;
    cam.target = {-0.1f, 0, 0.30f};
    cam.distance = 3.0f;
    cam.pitch = 0.35f;
    cam.yaw = 0.8f;

    bool running = true, dragL = false, dragR = false, stimulating = false;
    float stimRemaining = 0.0f;
    std::uint64_t frame = 0;
    std::vector<FlyBody::SegmentPose> segments;

    auto fire = [&]() {
        for (const auto i : driven) net.setStimulus(i, 200.0f);
        stimulating = true;
        stimRemaining = pulseMs;
    };
    fire();

    double titleTimer = 0.0;
    std::uint64_t lastCounter = SDL_GetPerformanceCounter();
    const double freq = static_cast<double>(SDL_GetPerformanceFrequency());

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
                case SDL_EVENT_QUIT: running = false; break;
                case SDL_EVENT_KEY_DOWN:
                    if (ev.key.key == SDLK_ESCAPE) running = false;
                    else if (ev.key.key == SDLK_SPACE) fire();
                    else if (ev.key.key == SDLK_R) {
                        net.reset(); pools.reset(); body.resetPose(); fire();
                    } else if (ev.key.key == SDLK_LEFTBRACKET)
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
                    if (dragL) cam.orbit(-ev.motion.xrel * 0.006f, ev.motion.yrel * 0.006f);
                    else if (dragR) cam.pan(ev.motion.xrel, ev.motion.yrel);
                    break;
                case SDL_EVENT_MOUSE_WHEEL:
                    cam.zoom(ev.wheel.y > 0 ? 0.9f : 1.111f);
                    break;
                default: break;
            }
        }

        for (int s = 0; s < stepsPerFrame; ++s) {
            net.step();
            pools.accumulate(net);
            if (stimulating && pulseMs > 0.0f) {
                stimRemaining -= params.dtMs;
                if (stimRemaining <= 0.0f) { net.clearStimulus(); stimulating = false; }
            }
        }
        const float frameMs = static_cast<float>(stepsPerFrame) * params.dtMs;
        pools.update(frameMs);
        if (!usePhysics) pools.applyToSkeleton(body);
        if (usePhysics) {
            phys.step(frameMs / 1000.0f, pools);
            // Close the loop: body state drives the proprioceptors, which
            // drive the network on the next frame.
            if (useSensory) sensory.sense(phys, net, frameMs);
            phys.readPose(segments);
        } else {
            body.worldPose(segments);
        }

        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);

        gl::glUseProgram(prog);
        const float aspect = h > 0 ? static_cast<float>(w) / static_cast<float>(h) : 1.0f;
        const M4 vp = cam.viewProjection(aspect);
        gl::glUniformMatrix4fv(uViewProj, 1, GL_FALSE, vp.m);
        const auto eye = cam.eye();
        gl::glUniform3f(uEyeLoc, eye.x, eye.y, eye.z);

        auto drawPart = [&](const Transform& t, const V3& stretch, const V3& colour,
                            float glow, const Mesh& mesh) {
            const M4 model = M4::fromTransform(t, stretch);
            // Normals need R * S^-1, not R * S, or non-uniform stretch tilts them.
            const M4 nrm = M4::fromTransform({{0, 0, 0}, t.rotation, 1.0f},
                                             {1.0f / stretch.x, 1.0f / stretch.y,
                                              1.0f / stretch.z});
            gl::glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
            gl::glUniformMatrix4fv(uNormalMat, 1, GL_FALSE, nrm.m);
            gl::glUniform3f(uColour, colour.x, colour.y, colour.z);
            gl::glUniform1f(uGlow, glow);
            mesh.draw();
        };

        // Body: thorax, abdomen behind it, head in front. The long axis is X
        // (fore-aft), so that is where the length goes -- putting it on Z
        // stands the fly on end.
        drawPart({body.root.position, body.root.rotation, 1.0f},
                 {0.46f, 0.30f, 0.30f}, {0.40f, 0.31f, 0.21f}, 0.0f, sph);
        drawPart({body.root.apply({-0.78f, 0, -0.04f}), body.root.rotation, 1.0f},
                 {0.52f, 0.26f, 0.26f}, {0.27f, 0.21f, 0.14f}, 0.0f, sph);
        drawPart({body.root.apply({0.52f, 0, 0.05f}), body.root.rotation, 1.0f},
                 {0.22f, 0.21f, 0.21f}, {0.44f, 0.29f, 0.20f}, 0.0f, sph);
        // Eyes.
        for (int e = -1; e <= 1; e += 2) {
            drawPart({body.root.apply({0.60f, 0.15f * e, 0.07f}), body.root.rotation, 1.0f},
                     {0.13f, 0.11f, 0.15f}, {0.55f, 0.13f, 0.08f}, 0.0f, sph);
        }

        // Legs. Each segment glows with how hard its joint is being driven, so
        // you can see which muscle is active as well as where the leg is.
        for (const auto& seg : segments) {
            const V3 delta = seg.b - seg.a;
            const float len = length(delta);
            if (len < 1e-5f) continue;

            const int l = static_cast<int>(seg.leg);
            const int j = static_cast<int>(seg.joint);
            const float glow = pools.peakActivation(l, j);

            drawPart({seg.a, aimDownZ(delta), 1.0f},
                     {seg.radius, seg.radius, len},
                     {0.55f, 0.45f, 0.32f}, glow, cyl);
            // A sphere at the joint hides the seam between segments.
            drawPart({seg.a, Quat{}, 1.0f},
                     {seg.radius * 1.25f, seg.radius * 1.25f, seg.radius * 1.25f},
                     {0.38f, 0.32f, 0.24f}, glow, sph);
        }

        const bool last = frameLimit > 0 &&
                          frame + 1 >= static_cast<std::uint64_t>(frameLimit);
        if (last && !shotPath.empty() && saveScreenshot(shotPath, w, h)) {
            std::printf("wrote %s (%dx%d)\n", shotPath.c_str(), w, h);
        }
        SDL_GL_SwapWindow(window);

        const std::uint64_t now = SDL_GetPerformanceCounter();
        const double dt = static_cast<double>(now - lastCounter) / freq;
        lastCounter = now;
        titleTimer += dt;
        if (titleTimer > 0.25) {
            // Report the most active joint, which is the one worth watching.
            float best = 0.0f;
            const char* bestLeg = "-";
            const char* bestJoint = "-";
            for (int l = 0; l < kLegCount; ++l) {
                for (int j = 0; j < kJointCount; ++j) {
                    const float a = pools.peakActivation(l, j);
                    if (a > best) {
                        best = a;
                        bestLeg = legName(static_cast<LegId>(l));
                        bestJoint = jointName(static_cast<Joint>(j));
                    }
                }
            }
            char title[256];
            std::snprintf(title, sizeof(title),
                          "FlyBody  |  t = %.0f ms  |  most active: %s %s %.0f%%"
                          "  |  %.0f fps",
                          net.timeMs(), bestLeg, bestJoint, best * 100.0,
                          1.0 / std::max(dt, 1e-6));
            SDL_SetWindowTitle(window, title);
            titleTimer = 0.0;
        }

        ++frame;
        if (last) running = false;
    }

    // On a bounded run, report which joints actually moved. This is the
    // claim the whole bridge rests on: that driving a named motor pool moves
    // the joint that pool is named after, on the leg it belongs to.
    if (frameLimit > 0) {
        std::printf("\n%-10s %-6s %-22s %8s %8s\n",
                    "leg", "joint", "muscle", "activ", "torque");
        for (const auto& m : pools.muscles()) {
            if (m.activation < 0.01f) continue;
            std::printf("%-10s %-6s %-22s %8.3f %8.2f\n",
                        legName(static_cast<LegId>(m.leg)),
                        jointName(static_cast<Joint>(m.joint)), m.name.c_str(),
                        m.activation, m.activation * m.strength);
        }
    }

    cyl.destroy();
    sph.destroy();
    gl::glDeleteProgram(prog);
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        if (ownsItsConsole()) {
            SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "FlyBody", e.what(), nullptr);
        }
        return 1;
    }
}
