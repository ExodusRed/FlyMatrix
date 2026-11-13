// flybody for the browser -- the fly's body and physics, running live as
// WebAssembly, driven by motor-unit activations recorded natively by flyrecord.
//
// Everything below the motor pools is the real thing: the same FlyBody
// skeleton, the same FlyPhysics solver (33 rigid bodies, 34 constraints,
// sequential impulses), stepped every millisecond in the browser. Only the
// nervous system is replaced by a recording, because the connectome does not
// fit in a web page. With the sensory loop off, as it is by default natively,
// nothing flows from body back to brain, so the drive is exactly what
// `flybody --drop` feeds its physics. The *outcome* is not bit-identical,
// though: the takeoff is sensitive to floating-point rounding (peak 2.9 mm
// native /fp:fast, 4.5 mm native strict, 7.4 mm WebAssembly, all from the
// same drive). fly_selftest() reports the browser's figure.
//
// The drawing code follows body_main.cpp, with shaders in GLSL ES 3.00.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <SDL3/SDL.h>
#include <emscripten.h>
#include <emscripten/html5.h>

#include "body/Anatomy.h"
#include "body/FlyBody.h"
#include "body/FlyPhysics.h"
#include "body/MotorPools.h"
#include "body/WalkController.h"
#include "engine/Math.h"
#include "engine/Mesh.h"
#include "viz/Camera.h"
#include "viz/GL.h"

using namespace fly;

namespace {

constexpr const char* kVert = R"(#version 300 es
precision highp float;
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

constexpr const char* kFrag = R"(#version 300 es
precision highp float;
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
    float fill = 0.28 + 0.22 * max(dot(n, vec3(0.0, 0.0, -1.0)), 0.0);

    vec3 viewDir = normalize(uEye - vWorld);
    float spec = pow(max(dot(reflect(-lightDir, n), viewDir), 0.0), 24.0);

    vec3 base = uColour * (fill + 0.75 * diffuse) + vec3(0.9) * spec * 0.25;
    vec3 hot = vec3(1.0, 0.75, 0.35);
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

Quat aimDownZ(const V3& dir) {
    const V3 d = normalise(dir);
    const V3 from{0, 0, -1};
    const float c = dot(from, d);
    if (c > 0.99999f) return {};
    if (c < -0.99999f) return Quat::axisAngle({1, 0, 0}, 3.14159265f);
    const V3 axis = cross(from, d);
    return Quat::axisAngle(axis, std::acos(std::clamp(c, -1.0f, 1.0f)));
}

// drive.bin, as written by flyrecord.
struct DriveRecording {
    std::uint32_t units = 0, steps = 0;
    float msPerStep = 1.0f, pulseMs = 0.0f;
    std::vector<std::uint16_t> data;  // steps * units

    static DriveRecording load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("cannot open " + path);
        char tag[4];
        std::uint32_t version = 0;
        DriveRecording r;
        f.read(tag, 4);
        f.read(reinterpret_cast<char*>(&version), 4);
        if (std::memcmp(tag, "FLYD", 4) != 0 || version != 1) {
            throw std::runtime_error(path + " is not a flyrecord drive file");
        }
        f.read(reinterpret_cast<char*>(&r.units), 4);
        f.read(reinterpret_cast<char*>(&r.steps), 4);
        f.read(reinterpret_cast<char*>(&r.msPerStep), 4);
        f.read(reinterpret_cast<char*>(&r.pulseMs), 4);
        r.data.resize(static_cast<std::size_t>(r.units) * r.steps);
        f.read(reinterpret_cast<char*>(r.data.data()),
               static_cast<std::streamsize>(r.data.size() * 2));
        if (!f) throw std::runtime_error(path + " is truncated");
        return r;
    }
};

struct App {
    SDL_Window* window = nullptr;
    GLuint prog = 0;
    Mesh cyl = Mesh::cylinder(14), sph = Mesh::sphere(14, 20), cube = Mesh::box();
    GLint uViewProj = -1, uModel = -1, uNormalMat = -1, uColour = -1, uEye = -1,
          uGlow = -1, uAlpha = -1;

    FlyBody body;
    FlyPhysics phys;
    MotorPools pools = MotorPools::load("/data/motor_map.tsv");
    DriveRecording drive = DriveRecording::load("/data/drive.bin");
    std::vector<float> activations;
    std::vector<FlyBody::SegmentPose> segments;
    viz::OrbitCamera cam;

    // Playback. The physics always advances in the recording's 1 ms steps, so
    // it reproduces the native run; `speed` is how many of those to take per
    // displayed frame, which is the slow-motion control.
    int step = 0;              // physics steps taken since the stimulus
    bool playing = true;       // feeding the recording (vs. idle after it ends)
    float speed = 1.0f;        // simulated ms per frame
    float carry = 0.0f;        // fractional steps owed when speed < 1
    bool paused = false;
    bool showArena = true, showWings = true;
    float peak = 0.0f;
    bool dragL = false, dragR = false;
    int canvasW = 0, canvasH = 0;

    // Two modes. Jump replays the recorded giant-fibre drive into the
    // physics. Walk runs the WalkController (a modelled pattern generator,
    // not the connectome -- see WalkController.h) live, steered by the user.
    enum class Mode { Walk, Jump };
    Mode mode = Mode::Walk;
    WalkController cpg;
    WalkController::Command walkCmd{1.0f, 0.0f};  // from the page's controls
    float keyTurn = 0.0f;                         // from held arrow / A-D keys
    float walkMsPerFrame = 4.0f;                  // fly ms per frame, budget permitting
    float measuredSpeed = 0.0f;                   // mm/s, smoothed
    V3 lastPos{};
    // Walking uses 32 solver iterations instead of 64: flyphys --walk gives
    // the same gait at both (and visibly worse at 16) for half the cost.
    // The jump keeps 64, which is what it was calibrated with.
    static constexpr int kWalkIterations = 32;

    App() {
        if (pools.unitCount() != drive.units) {
            throw std::runtime_error("drive.bin was recorded with a different motor map");
        }
        activations.assign(drive.units, 0.0f);
        resetBody();
        cam.zUp = true;
        cam.target = {-0.1f, 0, 0.45f};
        cam.distance = 7.0f;
        cam.pitch = -0.32f;
        cam.yaw = 0.8f;
    }

    // Same world as flyrecord's reference run: walls off so the jump is not
    // cut short, floor on, standard gravity.
    void resetBody() {
        body.resetPose();
        const bool walk = mode == Mode::Walk;
        phys.params.useManualTarget = walk;
        for (auto& leg : phys.params.manualTarget) for (float& t : leg) t = 0.0f;
        phys.build(body);
        // A big floor for walking (the fly is recentred long before it could
        // reach the edge, see recentre()); the usual 6 mm box for the jump.
        const float half = walk ? 30.0f : 6.0f;
        phys.world.params.arenaHalfX = half;
        phys.world.params.arenaHalfY = half;
        phys.world.params.arenaHeight = 7.2f;
        phys.world.params.wallsOn = false;
        if (walk) phys.world.params.iterations = kWalkIterations;
        pools.reset();
        cpg.reset();
        step = 0;
        carry = 0.0f;
        peak = 0.0f;
        playing = true;
        measuredSpeed = 0.0f;
        lastPos = phys.thorax().position;
        cam.target = {lastPos.x - 0.1f, lastPos.y, 0.45f};
    }

    void setMode(Mode m) {
        mode = m;
        resetBody();
    }

    float heading() const {
        const V3 fwd = phys.thorax().orientation.rotate({1, 0, 0});
        return std::atan2(fwd.y, fwd.x);
    }

    // The world is translation-invariant, so once the fly has walked a few
    // millimetres the whole scene is shifted back by a whole number of grid
    // squares. The floor looks identical, and the fly can walk forever.
    void recentre() {
        const V3 p = phys.thorax().position;
        if (std::fabs(p.x) < 10.0f && std::fabs(p.y) < 10.0f) return;
        const V3 shift{std::round(p.x), std::round(p.y), 0.0f};
        for (auto& b : phys.world.bodies) b.position = b.position - shift;
        cam.target = cam.target - shift;
        lastPos = lastPos - shift;
    }

    void walkStep() {
        const float dt = 0.001f;
        WalkController::Command cmd = walkCmd;
        cmd.turn = std::clamp(cmd.turn + keyTurn, -1.0f, 1.0f);
        const WalkController::Command steer = cpg.holdHeading(dt, cmd, heading());
        cpg.step(dt, steer, phys.params.manualTarget);
        phys.step(dt);
        ++step;
        // Ground speed, smoothed over ~0.2 s of fly time.
        const V3 p = phys.thorax().position;
        const float dx = p.x - lastPos.x, dy = p.y - lastPos.y;
        measuredSpeed += (std::sqrt(dx * dx + dy * dy) / dt - measuredSpeed) * 0.005f;
        lastPos = p;
        recentre();
    }

    void physicsStep() {
        if (mode == Mode::Walk) {
            walkStep();
            return;
        }
        if (playing && step < static_cast<int>(drive.steps)) {
            const std::uint16_t* row = &drive.data[static_cast<std::size_t>(step) * drive.units];
            for (std::uint32_t k = 0; k < drive.units; ++k) activations[k] = row[k] / 65535.0f;
        } else {
            playing = false;
            std::fill(activations.begin(), activations.end(), 0.0f);
        }
        pools.setActivations(activations);
        phys.step(drive.msPerStep / 1000.0f, pools);
        ++step;
        peak = std::max(peak, phys.bodyHeight());
    }

    void handle(const SDL_Event& ev) {
        switch (ev.type) {
            case SDL_EVENT_KEY_UP:
                if (ev.key.key == SDLK_LEFT || ev.key.key == SDLK_A ||
                    ev.key.key == SDLK_RIGHT || ev.key.key == SDLK_D) keyTurn = 0.0f;
                break;
            case SDL_EVENT_KEY_DOWN:
                // Walking: steer with arrows or WASD.
                if (ev.key.key == SDLK_LEFT || ev.key.key == SDLK_A) keyTurn = 1.0f;
                else if (ev.key.key == SDLK_RIGHT || ev.key.key == SDLK_D) keyTurn = -1.0f;
                else if (ev.key.key == SDLK_UP || ev.key.key == SDLK_W)
                    walkCmd.speed = std::min(1.0f, walkCmd.speed + 0.25f);
                else if (ev.key.key == SDLK_DOWN || ev.key.key == SDLK_S)
                    walkCmd.speed = std::max(0.0f, walkCmd.speed - 0.25f);
                else if (ev.key.key == SDLK_SPACE || ev.key.key == SDLK_R) resetBody();
                else if (ev.key.key == SDLK_P) paused = !paused;
                else if (ev.key.key == SDLK_G) phys.world.params.groundOn = !phys.world.params.groundOn;
                else if (ev.key.key == SDLK_B) phys.world.params.wallsOn = !phys.world.params.wallsOn;
                else if (ev.key.key == SDLK_V) {
                    const bool on = phys.world.params.gravity.z != 0.0f;
                    phys.world.params.gravity = on ? V3{0, 0, 0} : V3{0, 0, -9810.0f};
                } else if (ev.key.key == SDLK_H) showArena = !showArena;
                else if (ev.key.key == SDLK_LEFTBRACKET) speed = std::max(0.125f, speed * 0.5f);
                else if (ev.key.key == SDLK_RIGHTBRACKET) speed = std::min(1.0f, speed * 2.0f);
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
            default:
                break;
        }
    }

    // Keep the drawing buffer matched to the canvas's on-page size.
    void syncCanvasSize() {
        double cssW = 0, cssH = 0;
        emscripten_get_element_css_size("#canvas", &cssW, &cssH);
        const int w = static_cast<int>(cssW), h = static_cast<int>(cssH);
        if (w > 0 && h > 0 && (w != canvasW || h != canvasH)) {
            SDL_SetWindowSize(window, w, h);
            canvasW = w;
            canvasH = h;
        }
    }

    void frame() {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) handle(ev);
        syncCanvasSize();

        if (!paused) {
            // The solver is expensive (natively too: ~0.1 s of CPU per
            // simulated millisecond with the network). Spend at most ~20 ms a
            // frame on it so the page stays responsive; on a slow machine the
            // fly simply moves in slower motion instead of freezing the tab.
            carry += mode == Mode::Walk ? walkMsPerFrame : speed;
            const double start = emscripten_get_now();
            while (carry >= 1.0f && emscripten_get_now() - start < 20.0) {
                physicsStep();
                carry -= 1.0f;
            }
            carry = std::min(carry, 1.0f);  // don't bank a backlog
        }
        phys.readPose(segments);
        body.root.position = phys.thorax().position;
        body.root.rotation = phys.thorax().orientation;
        if (mode == Mode::Walk) {
            // Follow the fly, eased so the camera does not inherit the bounce
            // of every step; only the horizontal position is followed.
            const V3 want{body.root.position.x, body.root.position.y, cam.target.z};
            cam.target = cam.target + (want - cam.target) * 0.08f;
        }
        draw();
    }

    void draw() {
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
        gl::glUniform3f(uEye, eye.x, eye.y, eye.z);

        auto drawMesh = [&](const Transform& t, const V3& stretch, const V3& colour,
                            float glow, float alpha, const Mesh& mesh) {
            const M4 model = M4::fromTransform(t, stretch);
            const M4 nrm = M4::fromTransform({{0, 0, 0}, t.rotation, 1.0f},
                                             {1.0f / stretch.x, 1.0f / stretch.y,
                                              1.0f / stretch.z});
            gl::glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
            gl::glUniformMatrix4fv(uNormalMat, 1, GL_FALSE, nrm.m);
            gl::glUniform3f(uColour, colour.x, colour.y, colour.z);
            gl::glUniform1f(uGlow, glow);
            gl::glUniform1f(uAlpha, alpha);
            mesh.draw();
        };
        auto drawPart = [&](const Transform& t, const V3& stretch, const V3& colour,
                            float glow, const Mesh& mesh) {
            drawMesh(t, stretch, colour, glow, 1.0f, mesh);
        };

        const auto& ap = phys.world.params;
        if (ap.groundOn && showArena) {
            drawPart({{0, 0, ap.groundZ - 0.4f}, Quat{}, 1.0f},
                     {ap.arenaHalfX, ap.arenaHalfY, 0.4f}, {0.16f, 0.17f, 0.20f}, 0.0f, cube);
            const float t = 0.012f;
            for (float g = -ap.arenaHalfX; g <= ap.arenaHalfX + 0.01f; g += 1.0f) {
                drawPart({{g, 0, ap.groundZ + 0.001f}, Quat{}, 1.0f},
                         {t, ap.arenaHalfY, 0.004f}, {0.30f, 0.33f, 0.38f}, 0.0f, cube);
            }
            for (float g = -ap.arenaHalfY; g <= ap.arenaHalfY + 0.01f; g += 1.0f) {
                drawPart({{0, g, ap.groundZ + 0.001f}, Quat{}, 1.0f},
                         {ap.arenaHalfX, t, 0.004f}, {0.30f, 0.33f, 0.38f}, 0.0f, cube);
            }
        }

        const Quat& bq = body.root.rotation;
        auto at = [&](const V3& local) { return body.root.apply(local); };

        drawPart({body.root.position, bq, 1.0f}, anat::kThoraxHalf, {0.40f, 0.31f, 0.21f}, 0.0f, sph);
        drawPart({at({anat::kAbdomenX, 0, anat::kAbdomenZ}), bq, 1.0f}, anat::kAbdomenHalf,
                 {0.27f, 0.21f, 0.14f}, 0.0f, sph);
        drawPart({at({anat::kHeadX, 0, anat::kHeadZ}), bq, 1.0f}, anat::kHeadHalf,
                 {0.44f, 0.29f, 0.20f}, 0.0f, sph);
        for (int e = -1; e <= 1; e += 2) {
            drawPart({at({anat::kHeadX + anat::kEyeOffset.x, anat::kEyeOffset.y * e,
                          anat::kHeadZ + anat::kEyeOffset.z}), bq, 1.0f},
                     anat::kEyeHalf, {0.58f, 0.12f, 0.07f}, 0.0f, sph);
        }
        drawPart({at({anat::kHeadX + anat::kRostrumOffset.x, 0, anat::kHeadZ + anat::kRostrumOffset.z}),
                  bq, 1.0f}, anat::kRostrumHalf, {0.36f, 0.25f, 0.17f}, 0.0f, sph);
        drawPart({at({anat::kHeadX + anat::kHaustellumOffset.x, 0,
                      anat::kHeadZ + anat::kHaustellumOffset.z}), bq, 1.0f},
                 anat::kHaustellumHalf, {0.31f, 0.22f, 0.15f}, 0.0f, sph);
        for (int e = -1; e <= 1; e += 2) {
            drawPart({at({anat::kHeadX + anat::kLabellumOffset.x, anat::kLabellumOffset.y * e,
                          anat::kHeadZ + anat::kLabellumOffset.z}), bq, 1.0f},
                     anat::kLabellumHalf, {0.42f, 0.30f, 0.22f}, 0.0f, sph);
        }
        for (int e = -1; e <= 1; e += 2) {
            const V3 base = at({anat::kHeadX + anat::kFuniculusOffset.x, anat::kFuniculusOffset.y * e,
                                anat::kHeadZ + anat::kFuniculusOffset.z});
            drawPart({base, bq, 1.0f}, anat::kFuniculusHalf, {0.34f, 0.24f, 0.16f}, 0.0f, sph);
            const V3 tip = at({anat::kHeadX + anat::kFuniculusOffset.x + 0.20f,
                               (anat::kFuniculusOffset.y + 0.13f) * e,
                               anat::kHeadZ + anat::kFuniculusOffset.z + 0.10f});
            drawPart({base, aimDownZ(tip - base), 1.0f},
                     {anat::kAristaRadius, anat::kAristaRadius, anat::kAristaLength},
                     {0.30f, 0.26f, 0.20f}, 0.0f, cyl);
        }

        // Legs glow with how hard their joint is being driven.
        for (const auto& seg : segments) {
            const V3 delta = seg.b - seg.a;
            const float len = length(delta);
            if (len < 1e-5f) continue;
            // Jump: glow with muscle activation. Walk: legs in swing glow, so
            // the alternating tripod is visible at a glance.
            const int legIdx = static_cast<int>(seg.leg);
            const float glow = mode == Mode::Walk
                ? (cpg.swinging(legIdx) ? 0.45f : 0.0f)
                : pools.peakActivation(legIdx, static_cast<int>(seg.joint));
            drawPart({seg.a, aimDownZ(delta), 1.0f}, {seg.radius, seg.radius, len},
                     {0.55f, 0.45f, 0.32f}, glow, cyl);
            drawPart({seg.a, Quat{}, 1.0f},
                     {seg.radius * 1.25f, seg.radius * 1.25f, seg.radius * 1.25f},
                     {0.38f, 0.32f, 0.24f}, glow, sph);
        }

        if (showWings) {
            for (int e = -1; e <= 1; e += 2) {
                const V3 root = at({anat::kWingRoot.x, anat::kWingRoot.y * e, anat::kWingRoot.z});
                const Quat sweep = Quat::axisAngle({0, 0, 1}, (3.14159265f - anat::kWingSweepRad) * -e);
                const Quat tilt = Quat::axisAngle({1, 0, 0}, anat::kWingTiltRad * e);
                const Quat wq = (bq * sweep * tilt).normalised();
                const V3 centre = root + wq.rotate({anat::kWingLength * 0.5f, 0, 0});
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                drawMesh({centre, wq, 1.0f},
                         {anat::kWingLength * 0.5f, anat::kWingWidth * 0.5f, anat::kWingThickness},
                         {0.62f, 0.68f, 0.76f}, 0.0f, 0.22f, sph);
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);

                const V3 hroot = at({anat::kHaltereRoot.x, anat::kHaltereRoot.y * e, anat::kHaltereRoot.z});
                const V3 hend = at({anat::kHaltereRoot.x - 0.15f, (anat::kHaltereRoot.y + 0.02f) * e,
                                    anat::kHaltereRoot.z - 0.09f});
                drawPart({hroot, aimDownZ(hend - hroot), 1.0f},
                         {anat::kHaltereStalkRadius, anat::kHaltereStalkRadius, anat::kHaltereStalk},
                         {0.44f, 0.36f, 0.26f}, 0.0f, cyl);
                drawPart({hend, Quat{}, 1.0f},
                         {anat::kHaltereKnobRadius, anat::kHaltereKnobRadius, anat::kHaltereKnobRadius},
                         {0.50f, 0.40f, 0.28f}, 0.0f, sph);
            }
        }

        if (showArena && (ap.wallsOn || ap.ceilingOn)) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            const float hz = ap.arenaHeight * 0.5f;
            const float mid = ap.groundZ + hz;
            const float t = 0.05f;
            const V3 tint{0.45f, 0.62f, 0.72f};
            auto glass = [&](const V3& c, const V3& half, float a) {
                drawMesh({c, Quat{}, 1.0f}, half, tint, 0.0f, a, cube);
            };
            if (ap.wallsOn) {
                glass({+ap.arenaHalfX, 0, mid}, {t, ap.arenaHalfY, hz}, 0.07f);
                glass({-ap.arenaHalfX, 0, mid}, {t, ap.arenaHalfY, hz}, 0.07f);
                glass({0, +ap.arenaHalfY, mid}, {ap.arenaHalfX, t, hz}, 0.07f);
                glass({0, -ap.arenaHalfY, mid}, {ap.arenaHalfX, t, hz}, 0.07f);
            }
            if (ap.ceilingOn) glass({0, 0, ap.groundZ + ap.arenaHeight}, {ap.arenaHalfX, ap.arenaHalfY, t}, 0.05f);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }
    }

    // Status for the page, as JSON.
    std::string status() const {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "{\"mode\":\"%s\",\"t\":%d,\"height\":%.4f,\"peak\":%.4f,\"playing\":%s,\"speed\":%g,"
                      "\"paused\":%s,\"recordMs\":%u,\"ground\":%s,\"walls\":%s,\"gravity\":%s,"
                      "\"walkSpeed\":%.2f,\"walkCmd\":%.2f,\"turn\":%.2f,\"stepHz\":%.1f}",
                      mode == Mode::Walk ? "walk" : "jump",
                      step, phys.bodyHeight(), peak, playing ? "true" : "false", speed,
                      paused ? "true" : "false", drive.steps,
                      phys.world.params.groundOn ? "true" : "false",
                      phys.world.params.wallsOn ? "true" : "false",
                      phys.world.params.gravity.z != 0.0f ? "true" : "false",
                      measuredSpeed, walkCmd.speed, std::clamp(walkCmd.turn + keyTurn, -1.0f, 1.0f),
                      std::fabs(cpg.speed()) < 1e-3f ? 0.0f
                          : cpg.params.minFrequencyHz +
                            (cpg.params.maxFrequencyHz - cpg.params.minFrequencyHz) * std::fabs(cpg.speed()));
        return buf;
    }
};

App* g_app = nullptr;

void tick() {
    try {
        g_app->frame();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        emscripten_cancel_main_loop();
    }
}

}  // namespace

// Called from the page's buttons.
extern "C" {
EMSCRIPTEN_KEEPALIVE void fly_jump() { if (g_app) g_app->resetBody(); }
// 0 = walk, 1 = jump.
EMSCRIPTEN_KEEPALIVE void fly_mode(int m) {
    if (g_app) g_app->setMode(m == 1 ? App::Mode::Jump : App::Mode::Walk);
}
// Walking command from the page: speed 0..1, turn -1..1 (positive = left).
EMSCRIPTEN_KEEPALIVE void fly_walk(float speed, float turn) {
    if (!g_app) return;
    g_app->walkCmd.speed = std::clamp(speed, 0.0f, 1.0f);
    g_app->walkCmd.turn = std::clamp(turn, -1.0f, 1.0f);
}
EMSCRIPTEN_KEEPALIVE void fly_pause(int p) { if (g_app) g_app->paused = p != 0; }
EMSCRIPTEN_KEEPALIVE void fly_speed(float s) { if (g_app) g_app->speed = std::clamp(s, 0.125f, 1.0f); }
EMSCRIPTEN_KEEPALIVE void fly_toggle(int what) {
    if (!g_app) return;
    auto& p = g_app->phys.world.params;
    if (what == 0) p.groundOn = !p.groundOn;
    if (what == 1) p.wallsOn = !p.wallsOn;
    if (what == 2) p.gravity = p.gravity.z != 0.0f ? V3{0, 0, 0} : V3{0, 0, -9810.0f};
    if (what == 3) g_app->showArena = !g_app->showArena;
}
// Verification: replay the whole recording without drawing and report the
// peak height, which must match flyrecord's reference run, plus how long the
// physics took per simulated millisecond.
EMSCRIPTEN_KEEPALIVE const char* fly_selftest() {
    static std::string s;
    if (!g_app) return "{}";
    const auto previous = g_app->mode;
    g_app->mode = App::Mode::Jump;  // the self-test checks the recorded jump
    g_app->resetBody();
    const double t0 = emscripten_get_now();
    for (std::uint32_t i = 0; i < g_app->drive.steps; ++i) g_app->physicsStep();
    const double ms = emscripten_get_now() - t0;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "{\"steps\":%u,\"peak\":%.4f,\"final\":%.4f,\"msPerStep\":%.3f}",
                  g_app->drive.steps, g_app->peak, g_app->phys.bodyHeight(), ms / g_app->drive.steps);
    g_app->mode = previous;
    g_app->resetBody();
    s = buf;
    return s.c_str();
}
EMSCRIPTEN_KEEPALIVE const char* fly_status() {
    static std::string s;
    s = g_app ? g_app->status() : "{}";
    return s.c_str();
}
}

int main() {
    try {
        if (!SDL_Init(SDL_INIT_VIDEO)) throw std::runtime_error(SDL_GetError());
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

        SDL_Window* window = SDL_CreateWindow("FlyBody", 960, 600,
                                              SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
        if (!window) throw std::runtime_error(SDL_GetError());
        if (!SDL_GL_CreateContext(window)) throw std::runtime_error(SDL_GetError());

        static App app;
        app.window = window;
        app.prog = buildProgram();
        app.cyl.upload();
        app.sph.upload();
        app.cube.upload();
        app.uViewProj = gl::glGetUniformLocation(app.prog, "uViewProj");
        app.uModel = gl::glGetUniformLocation(app.prog, "uModel");
        app.uNormalMat = gl::glGetUniformLocation(app.prog, "uNormalMat");
        app.uColour = gl::glGetUniformLocation(app.prog, "uColour");
        app.uEye = gl::glGetUniformLocation(app.prog, "uEye");
        app.uGlow = gl::glGetUniformLocation(app.prog, "uGlow");
        app.uAlpha = gl::glGetUniformLocation(app.prog, "uAlpha");
        g_app = &app;

        EM_ASM({ if (Module.onFlyReady) Module.onFlyReady(); });
        emscripten_set_main_loop(tick, 0, false);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        EM_ASM({ if (Module.onFlyError) Module.onFlyError(UTF8ToString($0)); }, e.what());
        return 1;
    }
    return 0;
}
