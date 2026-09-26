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

#include "body/Anatomy.h"
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
uniform float uAlpha;

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
    // Glass: translucent, and brighter where the surface turns away from
    // the viewer, which is how a real pane reads at a glancing angle.
    float a = clamp(uAlpha, 0.0, 1.0);
    if (a < 0.999) {
        float facing = abs(dot(n, normalize(uEye - vWorld)));
        float fresnel = pow(1.0 - facing, 3.0);
        vec3 tint = uColour * (0.35 + 0.65 * diffuse) + vec3(fresnel * 0.55);
        fragColour = vec4(tint, a + fresnel * 0.35);
        return;
    }
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
        "  --muscle-torque X  peak torque per muscle pool\n"
        "  --gait MS          walk with an imposed tripod of this period\n"
        "                     (a sine wave, not a neuron -- see the README)\n"
        "  --gait-swing RAD   coxa swing amplitude (default 0.3)\n"
        "  --probe-joint NAME print per-leg drive on one joint\n"
        "  --arena MM         half-width of the box (default 6)\n"
        "  --walls/--no-walls glass walls (on in the 3D view, off for --drop)\n"
        "  --ceiling          close the box at the top\n"
        "  --no-ground        remove the floor, so the fly falls\n"
        "  --no-gravity       switch gravity off\n"
        "  --no-arena         hide the arena without changing the physics\n\n"
        "controls\n"
        "  left drag   orbit          scroll  zoom         right drag  pan\n"
        "  space       fire stimulus  r       reset        esc         quit\n"
        "  g  ground on/off    b  glass walls    c  ceiling\n"
        "  v  gravity on/off   h  hide arena     -/+  arena size\n");
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
    // Save a numbered frame every N frames, so the motion can be looked
    // at rather than inferred from statistics. Whole sessions of this
    // project were spent tuning numbers without once watching the fly
    // move, and the limbs were visibly glitching the entire time.
    std::string filmPath;
    long filmEvery = 10;
    bool dumpPose = false;
    bool usePhysics = true;
    float dropMs = 0.0f;
    float muscleTorque = -1.0f;
    float hillVmax = -1.0f, hillTaper = -1.0f, maxAngVel = -1.0f;
    int hillOn = -1;
    int forceJoint = -1;
    float forceDrive = 0.0f;
    float corrVel = -1.0f;
    // Off by default. The loop is implemented and measurable, but it
    // degrades everything it touches: with it on the giant fibre stops
    // jumping and a Kenyon cell starts moving the fly, which inverts the
    // specificity result the rest of the project rests on. Opt in with
    // --sensory to study it; see the README for why it does not work yet.
    bool useSensory = false;
    bool noSplit = false;
    // Imposed tripod in the 3D view. Same hand-built pattern as
    // flyphys --gait, and the same caveat: the rhythm is a sine wave,
    // not a neuron. It is here so the walking can be watched.
    float gaitPeriodMs = 0.0f;
    float gaitSwing = 0.3f;
    // The arena. Walls default on in the 3D view so the fly is visibly in a
    // place rather than floating in a void, and default off in the engine so
    // the headless measurements are unchanged.
    bool showArena = true;
    bool showWings = true;
    // Camera overrides, for looking at the model from a chosen angle.
    float camYaw = 0.8f, camPitch = -0.32f, camDist = -1.0f;
    bool arenaWalls = true;
    bool wallsExplicit = false;
    bool arenaCeiling = false;
    bool noGround = false;
    bool noGravity = false;
    float arenaSize = 6.0f;
    float extraLoad = 0.0f;
    // Print the per-leg drive on one joint each sample, so a gait
    // pattern (or its absence) is visible directly.
    std::string probeJoint;
    float sensoryDrive = -1.0f;
    // Tracked separately because a *negative* gain is a meaningful
    // setting, not an unset one: it inverts the reflex. Gating on
    // sensoryDrive >= 0 silently ignored every negative value, so the
    // inverted reflex looked identical to the default and the one free
    // variable the connectome cannot supply was never actually tested.
    bool sensoryDriveSet = false;

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
        else if (a == "--adapt") params.adaptIncrement = std::stof(next("--adapt"));
        else if (a == "--tau-adapt") params.tauAdapt = std::stof(next("--tau-adapt"));
        else if (a == "--steps") stepsPerFrame = std::atoi(next("--steps").c_str());
        else if (a == "--pulse") pulseMs = std::stof(next("--pulse"));
        else if (a == "--frames") frameLimit = std::atol(next("--frames").c_str());
        else if (a == "--screenshot") shotPath = next("--screenshot");
        else if (a == "--film") filmPath = next("--film");
        else if (a == "--film-every") filmEvery = std::atol(next("--film-every").c_str());
        else if (a == "--dump-pose") dumpPose = true;
        else if (a == "--no-physics") usePhysics = false;
        else if (a == "--drop") dropMs = std::stof(next("--drop"));
        else if (a == "--muscle-torque") muscleTorque = std::stof(next("--muscle-torque"));
        else if (a == "--hill-vmax") hillVmax = std::stof(next("--hill-vmax"));
        else if (a == "--hill-taper") hillTaper = std::stof(next("--hill-taper"));
        else if (a == "--no-hill") hillOn = 0;
        else if (a == "--max-angvel") maxAngVel = std::stof(next("--max-angvel"));
        else if (a == "--force-joint") {
            const std::string jn = next("--force-joint");
            for (int j = 0; j < kJointCount; ++j) {
                if (jn == jointName(static_cast<Joint>(j))) forceJoint = j;
            }
            if (forceJoint < 0) throw std::runtime_error("unknown joint: " + jn);
        }
        else if (a == "--force-drive") forceDrive = std::stof(next("--force-drive"));
        else if (a == "--corr") corrVel = std::stof(next("--corr"));
        else if (a == "--probe-joint") probeJoint = next("--probe-joint");
        else if (a == "--sensory") useSensory = true;
        else if (a == "--no-split") noSplit = true;
        else if (a == "--gait") gaitPeriodMs = std::stof(next("--gait"));
        else if (a == "--gait-swing") gaitSwing = std::stof(next("--gait-swing"));
        else if (a == "--arena") arenaSize = std::stof(next("--arena"));
        else if (a == "--no-arena") showArena = false;
        else if (a == "--no-wings") showWings = false;
        else if (a == "--cam") {
            camYaw = std::stof(next("--cam"));
            camPitch = std::stof(next("--cam"));
            camDist = std::stof(next("--cam"));
        }
        else if (a == "--no-walls") { arenaWalls = false; wallsExplicit = true; }
        else if (a == "--walls") { arenaWalls = true; wallsExplicit = true; }
        else if (a == "--ceiling") arenaCeiling = true;
        else if (a == "--no-ground") noGround = true;
        else if (a == "--no-gravity") noGravity = true;
        else if (a == "--no-sensory") useSensory = false;
        else if (a == "--load") extraLoad = std::stof(next("--load"));
        else if (a == "--sensory-drive") {
            sensoryDrive = std::stof(next("--sensory-drive"));
            sensoryDriveSet = true;
        }
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
    if (sensoryDriveSet) sensoryParams.maxDrive = sensoryDrive;
    if (noSplit) sensoryParams.useSplit = false;
    SensoryOrgans sensory =
        SensoryOrgans::load(dataDir + "/sensory_map.tsv", sensoryParams);
    // Optional: without it every chordotonal neuron stays in one group and
    // the loop drives antagonist pathways with the same signal.
    sensory.loadSplit(dataDir + "/feco_split.tsv");
    std::printf("proprioceptors: %zu chordotonal (%zu split by target), "
                "%zu campaniform%s\n",
                sensory.chordotonalCount(), sensory.splitCount(),
                sensory.campaniformCount(),
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
    if (hillVmax > 0.0f) phys.params.hillShorteningRate = hillVmax;
    if (hillTaper > 0.0f) phys.params.hillTaperFrac = hillTaper;
    if (hillOn == 0) phys.params.hillMuscle = false;
    if (maxAngVel > 0.0f) phys.world.params.maxAngularVelocity = maxAngVel;
    phys.params.forceJoint = forceJoint;
    phys.params.forceDrive = forceDrive;
    if (usePhysics) phys.build(body);
    // Applied after build(), which recreates the world and its params.
    if (usePhysics) {
        phys.world.params.arenaHalfX = arenaSize;
        phys.world.params.arenaHalfY = arenaSize;
        phys.world.params.arenaHeight = arenaSize * 1.2f;
        // Walls are a feature of the 3D view, not of the measurements. A
        // headless --drop run keeps them off unless asked, because they are
        // not free: with a 6 mm box the giant fibre jump comes back 4.53 mm
        // instead of 7.20, having bounced off one, and every jump figure in
        // the README and findings was taken without them.
        phys.world.params.wallsOn =
            wallsExplicit ? arenaWalls : (arenaWalls && dropMs <= 0.0f);
        phys.world.params.ceilingOn = arenaCeiling;
        phys.world.params.groundOn = !noGround;
        if (noGravity) phys.world.params.gravity = {0, 0, 0};
    }
    if (gaitPeriodMs > 0.0f) phys.params.useManualTarget = true;
    float gaitClockMs = 0.0f;
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
        std::uint32_t spikesThisMs = 0;
        // Rhythmicity of the probed joint's drive, measured rather than
        // eyeballed. A tonic signal crosses its own running mean almost
        // never; an oscillation crosses it twice per cycle, so the crossing
        // count over a known duration gives a frequency directly.
        std::vector<float> probeHist[kLegCount];
        for (const auto i : driven) net.setStimulus(i, 200.0f);

        std::printf("running %.0f ms: %zu driven neuron(s), physics at %.0f Hz\n",
                    dropMs, driven.size(), 1.0f / physDt);
        // CTr act is the largest activation on either side of the joint;
        // CTr drv is the net of the two. When act is high and drv is near
        // zero the antagonists are firing together and cancelling, which
        // looks identical to "the muscle is working" in the activation
        // column alone.
        int probeIdx = -1;
        for (int j = 0; j < kJointCount; ++j) {
            if (probeJoint == jointName(static_cast<Joint>(j))) probeIdx = j;
        }
        if (probeIdx >= 0) {
            // Per-leg drive on one joint. A gait is visible here or nowhere:
            // the six columns should alternate in a tripod pattern, with
            // front_L, middle_R and hind_L moving together and against the
            // other three.
            std::printf("%8s %10s %9s", "t (ms)", "height", "spikes");
            for (int l = 0; l < kLegCount; ++l) {
                std::printf(" %9s", legName(static_cast<LegId>(l)));
            }
            std::printf("\n");
        } else {
            std::printf("%8s %10s %10s %9s %9s %9s %9s\n",
                        "t (ms)", "height", "peak", "contacts", "CTr act",
                        "CTr drv", "spikes");
        }
        for (int i = 0; i < n; ++i) {
            // Spikes across this physics step's worth of neural steps. A
            // stimulus that has ended should show this decaying; if it does
            // not, the network is self-sustaining rather than settling.
            spikesThisMs = 0;
            for (int k = 0; k < neuralPerPhys; ++k) {
                spikesThisMs += net.step().spikeCount;
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
            if (gaitPeriodMs > 0.0f) {
                static const bool tripodA[kLegCount] =
                    {true, false, false, true, true, false};
                gaitClockMs += physDt * 1000.0f;
                const float phase = 6.2831853f * gaitClockMs / gaitPeriodMs;
                for (int l = 0; l < kLegCount; ++l) {
                    const float ph = tripodA[l] ? phase : phase + 3.14159265f;
                    phys.params.manualTarget[l][static_cast<int>(Joint::ThC)] =
                        -gaitSwing * std::sin(ph);
                    const float c = std::cos(ph);
                    phys.params.manualTarget[l][static_cast<int>(Joint::CTr)] =
                        (c > 0.0f) ? gaitSwing * 0.6f * c : 0.0f;
                }
            }
            phys.step(physDt, pools);

            // Body state drives the proprioceptors, which drive the network on
            // the next step. This is the only place anything flows backwards.
            if (useSensory) sensory.sense(phys, net, 1.0f);

            if (probeIdx >= 0) {
                for (int l = 0; l < kLegCount; ++l) {
                    probeHist[l].push_back(pools.drive(l, probeIdx));
                }
            }
            if (i % std::max(1, n / 12) == 0 || i == n - 1) {
                // Coxa-trochanter is where the jump muscle pulls.
                float ctr = 0.0f, ctrDrive = 0.0f;
                for (int l = 0; l < kLegCount; ++l) {
                    ctr = std::max(ctr,
                        pools.peakActivation(l, static_cast<int>(Joint::CTr)));
                    const float d = pools.drive(l, static_cast<int>(Joint::CTr));
                    if (std::fabs(d) > std::fabs(ctrDrive)) ctrDrive = d;
                }
                if (probeIdx >= 0) {
                    std::printf("%8.0f %10.4f %9u", i * physDt * 1000.0f,
                                phys.bodyHeight(), spikesThisMs);
                    for (int l = 0; l < kLegCount; ++l) {
                        std::printf(" %+9.3f", pools.drive(l, probeIdx));
                    }
                    std::printf("\n");
                } else {
                    std::printf("%8.0f %10.4f %10.4f %9zu %9.3f %9.3f %9u\n",
                                i * physDt * 1000.0f, phys.bodyHeight(),
                                phys.peakHeight(), phys.world.contacts.size(),
                                ctr, ctrDrive, spikesThisMs);
                }
            }
            if (!std::isfinite(phys.bodyHeight()) ||
                std::fabs(phys.bodyHeight()) > 1e4f) {
                std::printf("\nSOLVER DIVERGED at %.0f ms\n", i * physDt * 1000.0f);
                return 2;
            }
        }
        // Rhythmicity of the probed joint's drive, measured rather than
        // eyeballed. A tonic signal crosses its own mean almost never; an
        // oscillation crosses it twice per cycle, so the crossing count over
        // a known duration gives a frequency directly.
        if (probeIdx >= 0 && !probeHist[0].empty()) {
            std::printf("\nrhythm on %s, per leg:\n",
                        jointName(static_cast<Joint>(probeIdx)));
            std::printf("%-10s %10s %10s %10s\n", "leg", "mean", "swing", "Hz");
            for (int l = 0; l < kLegCount; ++l) {
                const auto& h = probeHist[l];
                double sum = 0.0;
                for (const float v : h) sum += v;
                const auto mean = static_cast<float>(sum / h.size());
                float lo = h[0], hi = h[0];
                for (const float v : h) {
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
                // Deadband, so numerical noise around a flat signal is not
                // counted as an oscillation.
                const float dead = 0.05f * std::max(hi - lo, 1e-6f);
                int crossings = 0, sign = 0;
                for (const float v : h) {
                    const float d = v - mean;
                    const int sgn = (d > dead) ? 1 : (d < -dead ? -1 : 0);
                    if (sgn != 0 && sign != 0 && sgn != sign) ++crossings;
                    if (sgn != 0) sign = sgn;
                }
                const float secs = static_cast<float>(h.size()) * physDt;
                std::printf("%-10s %10.3f %10.3f %10.2f\n",
                            legName(static_cast<LegId>(l)), mean, hi - lo,
                            0.5f * static_cast<float>(crossings) / secs);
            }
        }

        std::printf("\nfinal %.4f mm, peak %.4f mm, peak joint rate %.0f rad/s\n",
                    phys.bodyHeight(), phys.peakHeight(), phys.peakJointRate());
        // Physical cross-check. Units are micrograms, millimetres and seconds,
        // so a torque of 1 is 1e-15 N m and a force of 1 is 1e-12 N. Body
        // weight is the natural yardstick: a fly weighs about 10 uN, and
        // Azevedo et al. measure a single fast leg motor neuron at about 10 uN
        // of muscle force, so one fast motor unit is roughly one body weight.
        {
            const float mass = phys.totalMass();          // ug
            const float weight = mass * 9810.0f;          // ug mm/s^2
            const float peakTorque =
                phys.peakDrive() * phys.params.maxMuscleTorque;
            const float arm = 0.3f;                       // mm, typical
            std::printf("mass %.0f ug (weight %.2f uN), peak drive %.2f, "
                        "peak torque %.2e = %.1f body weights at %.1f mm arm\n",
                        mass, weight * 1e-6f, phys.peakDrive(), peakTorque,
                        peakTorque / (weight * arm), arm);
        }
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
    Mesh cube = Mesh::box();
    cube.upload();

    const GLint uViewProj = gl::glGetUniformLocation(prog, "uViewProj");
    const GLint uModel = gl::glGetUniformLocation(prog, "uModel");
    const GLint uNormalMat = gl::glGetUniformLocation(prog, "uNormalMat");
    const GLint uColour = gl::glGetUniformLocation(prog, "uColour");
    const GLint uEyeLoc = gl::glGetUniformLocation(prog, "uEye");
    const GLint uGlow = gl::glGetUniformLocation(prog, "uGlow");
    const GLint uAlpha = gl::glGetUniformLocation(prog, "uAlpha");

    viz::OrbitCamera cam;
    cam.zUp = true;
    cam.target = {-0.1f, 0, 0.30f};
    // Pull back far enough to see the box the fly is standing in, but not
    // so far that the fly becomes a speck. Scaled off the arena so --arena
    // reframes automatically.
    cam.distance = (usePhysics && arenaWalls) ? arenaSize * 1.15f : 3.0f;
    // Negative, because eye = target - forward * distance and forward.z is
    // +sin(pitch): a positive pitch puts the camera *below* the target. It
    // always had, and with nothing drawn at z = 0 nobody noticed the fly was
    // being viewed from underground until there was a floor to hide behind.
    cam.pitch = camPitch;
    cam.yaw = camYaw;
    if (camDist > 0.0f) cam.distance = camDist;

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
                    // Arena controls. These change the physics live, so
                    // turning the floor off drops the fly and turning gravity
                    // off leaves it where it is -- which is the quickest way
                    // to see that the two are separate things.
                    else if (ev.key.key == SDLK_G) {
                        phys.world.params.groundOn = !phys.world.params.groundOn;
                        std::printf("ground %s\n",
                                    phys.world.params.groundOn ? "on" : "off");
                    } else if (ev.key.key == SDLK_B) {
                        phys.world.params.wallsOn = !phys.world.params.wallsOn;
                        std::printf("walls %s\n",
                                    phys.world.params.wallsOn ? "on" : "off");
                    } else if (ev.key.key == SDLK_C) {
                        phys.world.params.ceilingOn = !phys.world.params.ceilingOn;
                        std::printf("ceiling %s\n",
                                    phys.world.params.ceilingOn ? "on" : "off");
                    } else if (ev.key.key == SDLK_V) {
                        const bool on = phys.world.params.gravity.z != 0.0f;
                        phys.world.params.gravity = on ? V3{0, 0, 0}
                                                       : V3{0, 0, -9810.0f};
                        std::printf("gravity %s\n", on ? "off" : "on");
                    } else if (ev.key.key == SDLK_H) {
                        showArena = !showArena;
                    } else if (ev.key.key == SDLK_MINUS) {
                        arenaSize = std::max(3.0f, arenaSize - 2.0f);
                        phys.world.params.arenaHalfX = arenaSize;
                        phys.world.params.arenaHalfY = arenaSize;
                        phys.world.params.arenaHeight = arenaSize * 1.2f;
                    } else if (ev.key.key == SDLK_EQUALS) {
                        arenaSize = std::min(60.0f, arenaSize + 2.0f);
                        phys.world.params.arenaHalfX = arenaSize;
                        phys.world.params.arenaHalfY = arenaSize;
                        phys.world.params.arenaHeight = arenaSize * 1.2f;
                    }
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
            if (gaitPeriodMs > 0.0f) {
                // Alternating tripod: front_L, middle_R and hind_L swing
                // together, against the other three. Identical to the pattern
                // flyphys --gait measures, and carrying the same caveat --
                // the rhythm is a sine wave, not a neuron, so this shows the
                // body walking rather than the connectome walking it.
                static const bool tripodA[kLegCount] =
                    {true, false, false, true, true, false};
                gaitClockMs += frameMs;
                const float phase = 6.2831853f * gaitClockMs / gaitPeriodMs;
                for (int l = 0; l < kLegCount; ++l) {
                    const float ph = tripodA[l] ? phase : phase + 3.14159265f;
                    phys.params.manualTarget[l][static_cast<int>(Joint::ThC)] =
                        -gaitSwing * std::sin(ph);
                    const float c = std::cos(ph);
                    phys.params.manualTarget[l][static_cast<int>(Joint::CTr)] =
                        (c > 0.0f) ? gaitSwing * 0.6f * c : 0.0f;
                }
            }
            phys.step(frameMs / 1000.0f, pools);
            // Close the loop: body state drives the proprioceptors, which
            // drive the network on the next frame.
            if (useSensory) sensory.sense(phys, net, frameMs);
            phys.readPose(segments);
            // Carry the physics trunk back onto the skeleton, which is what
            // the body, head and eyes are drawn from.
            //
            // Without this the legs came from the solver and the trunk came
            // from a Transform nothing ever wrote to, so the two only agreed
            // while the fly stayed put. It looked fine standing and was wrong
            // every time the body actually moved -- during a jump, and
            // spectacularly while walking, where the legs strode off and left
            // the body hanging behind them.
            body.root.position = phys.thorax().position;
            body.root.rotation = phys.thorax().orientation;
            // Keep a walking fly in shot. Eased rather than locked, so the
            // camera does not inherit the gait's bounce, and only the
            // horizontal position is followed so height changes stay visible.
            if (gaitPeriodMs > 0.0f) {
                const V3 want{phys.thorax().position.x,
                              phys.thorax().position.y, cam.target.z};
                cam.target = cam.target + (want - cam.target) * 0.06f;
            }
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
            gl::glUniform1f(uAlpha, 1.0f);
            mesh.draw();
        };

        // Translucent draw with an arbitrary transform and mesh, for the
        // wing membrane.
        auto drawGlassAt = [&](const Transform& t, const V3& stretch,
                               const V3& colour, float alpha, const Mesh& mesh) {
            const M4 model = M4::fromTransform(t, stretch);
            const M4 nrm = M4::fromTransform({{0, 0, 0}, t.rotation, 1.0f},
                                             {1.0f / stretch.x, 1.0f / stretch.y,
                                              1.0f / stretch.z});
            gl::glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
            gl::glUniformMatrix4fv(uNormalMat, 1, GL_FALSE, nrm.m);
            gl::glUniform3f(uColour, colour.x, colour.y, colour.z);
            gl::glUniform1f(uGlow, 0.0f);
            gl::glUniform1f(uAlpha, alpha);
            mesh.draw();
        };

        // Same, but translucent: used for the arena's glass.
        auto drawGlass = [&](const V3& centre, const V3& half, const V3& colour,
                             float alpha) {
            const M4 model = M4::fromTransform({centre, Quat{}, 1.0f}, half);
            const M4 nrm = M4::fromTransform({{0, 0, 0}, Quat{}, 1.0f},
                                             {1.0f / half.x, 1.0f / half.y,
                                              1.0f / half.z});
            gl::glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
            gl::glUniformMatrix4fv(uNormalMat, 1, GL_FALSE, nrm.m);
            gl::glUniform3f(uColour, colour.x, colour.y, colour.z);
            gl::glUniform1f(uGlow, 0.0f);
            gl::glUniform1f(uAlpha, alpha);
            cube.draw();
        };

        // The arena. Drawn before the fly so the opaque floor is in the depth
        // buffer, with the glass left until after everything else.
        const auto& ap = phys.world.params;
        if (usePhysics && ap.groundOn && showArena) {
            // A thin slab rather than an infinite plane, so it has edges and
            // the eye can tell how big the box is.
            drawPart({{0, 0, ap.groundZ - 0.4f}, Quat{}, 1.0f},
                     {ap.arenaHalfX, ap.arenaHalfY, 0.4f},
                     {0.16f, 0.17f, 0.20f}, 0.0f, cube);
            // Grid lines, as thin raised slabs. A floor with no texture gives
            // the eye nothing to judge translation against, which is most of
            // why the fly looked as though it were floating.
            const float step = 1.0f;
            const float t = 0.012f;
            for (float g = -ap.arenaHalfX; g <= ap.arenaHalfX + 0.01f; g += step) {
                drawPart({{g, 0, ap.groundZ + 0.001f}, Quat{}, 1.0f},
                         {t, ap.arenaHalfY, 0.004f}, {0.30f, 0.33f, 0.38f}, 0.0f, cube);
            }
            for (float g = -ap.arenaHalfY; g <= ap.arenaHalfY + 0.01f; g += step) {
                drawPart({{0, g, ap.groundZ + 0.001f}, Quat{}, 1.0f},
                         {ap.arenaHalfX, t, 0.004f}, {0.30f, 0.33f, 0.38f}, 0.0f, cube);
            }
        }

        // Body: thorax, abdomen behind it, head in front. The long axis is X
        // (fore-aft), so that is where the length goes -- putting it on Z
        // stands the fly on end.
        const Quat& bq = body.root.rotation;
        auto at = [&](const V3& local) { return body.root.apply(local); };

        drawPart({body.root.position, bq, 1.0f}, anat::kThoraxHalf,
                 {0.40f, 0.31f, 0.21f}, 0.0f, sph);
        drawPart({at({anat::kAbdomenX, 0, anat::kAbdomenZ}), bq, 1.0f},
                 anat::kAbdomenHalf, {0.27f, 0.21f, 0.14f}, 0.0f, sph);
        drawPart({at({anat::kHeadX, 0, anat::kHeadZ}), bq, 1.0f},
                 anat::kHeadHalf, {0.44f, 0.29f, 0.20f}, 0.0f, sph);

        // Eyes: large, and most of the head.
        for (int e = -1; e <= 1; e += 2) {
            drawPart({at({anat::kHeadX + anat::kEyeOffset.x,
                          anat::kEyeOffset.y * e,
                          anat::kHeadZ + anat::kEyeOffset.z}), bq, 1.0f},
                     anat::kEyeHalf, {0.58f, 0.12f, 0.07f}, 0.0f, sph);
        }

        // Proboscis: rostrum, haustellum, and the paired labellar lobes that
        // touch the food. Drawn retracted, folded under the head. Without it
        // the fly had no mouthparts at all and could not have fed.
        drawPart({at({anat::kHeadX + anat::kRostrumOffset.x, 0,
                      anat::kHeadZ + anat::kRostrumOffset.z}), bq, 1.0f},
                 anat::kRostrumHalf, {0.36f, 0.25f, 0.17f}, 0.0f, sph);
        drawPart({at({anat::kHeadX + anat::kHaustellumOffset.x, 0,
                      anat::kHeadZ + anat::kHaustellumOffset.z}), bq, 1.0f},
                 anat::kHaustellumHalf, {0.31f, 0.22f, 0.15f}, 0.0f, sph);
        for (int e = -1; e <= 1; e += 2) {
            drawPart({at({anat::kHeadX + anat::kLabellumOffset.x,
                          anat::kLabellumOffset.y * e,
                          anat::kHeadZ + anat::kLabellumOffset.z}), bq, 1.0f},
                     anat::kLabellumHalf, {0.42f, 0.30f, 0.22f}, 0.0f, sph);
        }

        // Antennae: the funiculus and its arista, which is the feathery
        // bristle a fly senses air with.
        for (int e = -1; e <= 1; e += 2) {
            const V3 base = at({anat::kHeadX + anat::kFuniculusOffset.x,
                                anat::kFuniculusOffset.y * e,
                                anat::kHeadZ + anat::kFuniculusOffset.z});
            drawPart({base, bq, 1.0f}, anat::kFuniculusHalf,
                     {0.34f, 0.24f, 0.16f}, 0.0f, sph);
            // Swept forward and outward from the funiculus.
            const V3 tip = at({anat::kHeadX + anat::kFuniculusOffset.x + 0.20f,
                               (anat::kFuniculusOffset.y + 0.13f) * e,
                               anat::kHeadZ + anat::kFuniculusOffset.z + 0.10f});
            drawPart({base, aimDownZ(tip - base), 1.0f},
                     {anat::kAristaRadius, anat::kAristaRadius,
                      anat::kAristaLength},
                     {0.30f, 0.26f, 0.20f}, 0.0f, cyl);
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

        // Wings and halteres, after the opaque body so the membrane blends
        // against it rather than against an empty framebuffer.
        //
        // A fly's wing is a flat bilayered blade about as long as its whole
        // body, folded back over the abdomen at rest and overhanging its tip.
        // Halteres are the club-shaped organs behind them on the metathorax:
        // the serial homologue of the hind wings, and the fly's gyroscopes.
        if (showWings) {
            for (int e = -1; e <= 1; e += 2) {
                const V3 root = at({anat::kWingRoot.x, anat::kWingRoot.y * e,
                                    anat::kWingRoot.z});
                // Swept back along the body, splayed outward, tilted a little
                // nose-up. Built as a rotation about Z then X so the blade
                // lies roughly flat.
                const Quat sweep =
                    Quat::axisAngle({0, 0, 1}, (3.14159265f - anat::kWingSweepRad) * -e);
                const Quat tilt = Quat::axisAngle({1, 0, 0}, anat::kWingTiltRad * e);
                const Quat wq = (bq * sweep * tilt).normalised();
                // The blade's own centre is half a wing-length out from the
                // hinge, so it trails behind rather than straddling it.
                const V3 centre = root + wq.rotate({anat::kWingLength * 0.5f, 0, 0});
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                drawGlassAt({centre, wq, 1.0f},
                            {anat::kWingLength * 0.5f, anat::kWingWidth * 0.5f,
                             anat::kWingThickness},
                            {0.62f, 0.68f, 0.76f}, 0.22f, sph);
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);

                // Haltere: a thin stalk ending in a knob.
                const V3 hroot = at({anat::kHaltereRoot.x,
                                     anat::kHaltereRoot.y * e,
                                     anat::kHaltereRoot.z});
                const V3 hend = at({anat::kHaltereRoot.x - 0.15f,
                                    (anat::kHaltereRoot.y + 0.02f) * e,
                                    anat::kHaltereRoot.z - 0.09f});
                drawPart({hroot, aimDownZ(hend - hroot), 1.0f},
                         {anat::kHaltereStalkRadius, anat::kHaltereStalkRadius,
                          anat::kHaltereStalk},
                         {0.44f, 0.36f, 0.26f}, 0.0f, cyl);
                drawPart({hend, Quat{}, 1.0f},
                         {anat::kHaltereKnobRadius, anat::kHaltereKnobRadius,
                          anat::kHaltereKnobRadius},
                         {0.50f, 0.40f, 0.28f}, 0.0f, sph);
            }
        }

        // Glass last, and with depth writes off. Translucent surfaces have
        // to be drawn after everything behind them or they blend against an
        // empty framebuffer instead of the scene, and writing depth would
        // make the near pane occlude the far one.
        if (usePhysics && showArena && (ap.wallsOn || ap.ceilingOn)) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            const float hz = ap.arenaHeight * 0.5f;
            const float mid = ap.groundZ + hz;
            const float t = 0.05f;
            const V3 tint{0.45f, 0.62f, 0.72f};
            if (ap.wallsOn) {
                drawGlass({+ap.arenaHalfX, 0, mid}, {t, ap.arenaHalfY, hz}, tint, 0.07f);
                drawGlass({-ap.arenaHalfX, 0, mid}, {t, ap.arenaHalfY, hz}, tint, 0.07f);
                drawGlass({0, +ap.arenaHalfY, mid}, {ap.arenaHalfX, t, hz}, tint, 0.07f);
                drawGlass({0, -ap.arenaHalfY, mid}, {ap.arenaHalfX, t, hz}, tint, 0.07f);
            }
            if (ap.ceilingOn) {
                drawGlass({0, 0, ap.groundZ + ap.arenaHeight},
                          {ap.arenaHalfX, ap.arenaHalfY, t}, tint, 0.05f);
            }
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        if (!filmPath.empty() && filmEvery > 0 &&
            frame % static_cast<std::uint64_t>(filmEvery) == 0) {
            char name[512];
            std::snprintf(name, sizeof(name), "%s%04llu.bmp", filmPath.c_str(),
                          static_cast<unsigned long long>(frame / filmEvery));
            saveScreenshot(name, w, h);
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
        // "recruited" counts units above a tenth activation, which is what
        // makes recruitment order visible: a muscle can be producing a little
        // force with two small units or a lot with all of them.
        std::printf("\n%-10s %-6s %-22s %8s %9s %8s\n",
                    "leg", "joint", "muscle", "peak", "recruited", "force");
        for (const auto& m : pools.muscles()) {
            if (m.force() < 0.02f) continue;
            int recruited = 0;
            for (const auto& u : m.units) {
                if (u.activation > 0.1f) ++recruited;
            }
            std::printf("%-10s %-6s %-22s %8.3f %4d/%-4zu %8.2f\n",
                        legName(static_cast<LegId>(m.leg)),
                        jointName(static_cast<Joint>(m.joint)), m.name.c_str(),
                        m.peakActivation(), recruited, m.units.size(), m.force());
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
