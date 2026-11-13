// flyrecord -- records simulation runs for the web build.
//
// The browser cannot hold the connectome (cns.bin is ~100 MB, and the network
// needs several hundred MB of RAM), so the web version plays back runs that
// were simulated here, natively, with exactly the same code and parameters:
//
//   brain.bin  every neuron's position plus, per display frame, which neurons
//              spiked and what the graded neurons put out. The web viewer
//              rebuilds flyviz's activity trace from it with the same decay.
//
//   drive.bin  every leg motor unit's muscle activation, once per millisecond.
//              The web body feeds these into the live physics in place of the
//              network. With the sensory loop off (the default) nothing flows
//              from body back to brain, so the replayed drive is identical to
//              the live one and the physics reproduces the native run.
//
// Both formats are little-endian. Spike lists are delta-coded varints, which
// shrinks them by roughly 4x against raw indices.

#ifdef _WIN32
#define NOMINMAX
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "body/FlyBody.h"
#include "body/FlyPhysics.h"
#include "body/MotorPools.h"
#include "core/Connectome.h"
#include "core/DataPath.h"
#include "core/LIFNetwork.h"

using namespace fly;

namespace {

class Writer {
public:
    void u8(std::uint8_t v) { buf_.push_back(v); }
    void u16(std::uint16_t v) { raw(&v, 2); }
    void u32(std::uint32_t v) { raw(&v, 4); }
    void f32(float v) { raw(&v, 4); }
    void tag(const char (&t)[5]) { raw(t, 4); }
    // LEB128: seven bits per byte, high bit set while more follow.
    void varint(std::uint32_t v) {
        while (v >= 0x80) {
            buf_.push_back(static_cast<std::uint8_t>(v | 0x80));
            v >>= 7;
        }
        buf_.push_back(static_cast<std::uint8_t>(v));
    }
    std::size_t size() const { return buf_.size(); }
    void save(const std::string& path) const {
        std::ofstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("cannot write " + path);
        f.write(reinterpret_cast<const char*>(buf_.data()),
                static_cast<std::streamsize>(buf_.size()));
    }

private:
    void raw(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::uint8_t*>(p);
        buf_.insert(buf_.end(), b, b + n);
    }
    std::vector<std::uint8_t> buf_;
};

void printHelp() {
    std::printf(
        "flyrecord -- record runs for the web viewer\n\n"
        "  --data DIR        directory holding cns.bin (default: data/bin)\n"
        "  --out DIR         where to write brain.bin and drive.bin (default: web/data)\n"
        "  --stim-body ID    neuron to drive (default: 10001, the giant fibre)\n"
        "  --brain-frames N  display frames of brain activity (default: 150)\n"
        "  --steps N         simulation steps per brain frame (default: 8, as flyviz)\n"
        "  --body-ms MS      length of the body recording (default: 400)\n");
}

// Same stimulus, pulse and display rules as flyviz, so the recording shows
// what flyviz would show.
std::size_t recordBrain(const Connectome& conn, std::uint32_t driven, int frames,
                        int stepsPerFrame, const std::string& path) {
    const LifParams params;
    LIFNetwork net(conn, params);
    const float pulseMs = 15.0f;
    const std::uint32_t n = conn.neuronCount();

    Writer w;
    w.tag("FLYB");
    w.u32(1);  // format version
    w.u32(n);
    w.u32(static_cast<std::uint32_t>(frames));
    w.f32(static_cast<float>(stepsPerFrame) * params.dtMs);           // ms per frame
    w.f32(std::pow(0.90f, static_cast<float>(stepsPerFrame)));         // trace decay per frame
    w.f32(pulseMs);

    // Positions quantised to 16 bits across the bounding box: about 0.02 um
    // of resolution on a ~1300 um brain, far below a rendered point.
    const Vec3 lo = conn.bboxMin(), hi = conn.bboxMax();
    for (float v : {lo.x, lo.y, lo.z, hi.x, hi.y, hi.z}) w.f32(v);
    auto q = [](float v, float a, float b) {
        const float t = (b > a) ? (v - a) / (b - a) : 0.0f;
        return static_cast<std::uint16_t>(std::lround(std::clamp(t, 0.0f, 1.0f) * 65535.0f));
    };
    for (const auto& p : conn.positions()) {
        w.u16(q(p.x, lo.x, hi.x));
        w.u16(q(p.y, lo.y, hi.y));
        w.u16(q(p.z, lo.z, hi.z));
    }
    // Graded flag per neuron as a bitmask isn't needed: graded neurons are
    // listed explicitly per frame below.

    net.setStimulus(driven, 200.0f);
    float stimLeft = pulseMs;
    std::vector<std::uint8_t> spiked(n, 0);
    std::vector<std::uint32_t> list;
    std::uint64_t totalSpikes = 0;

    for (int f = 0; f < frames; ++f) {
        std::fill(spiked.begin(), spiked.end(), std::uint8_t{0});
        for (int s = 0; s < stepsPerFrame; ++s) {
            net.step();
            for (const auto j : net.lastSpikes()) spiked[j] = 1;
            if (stimLeft > 0.0f) {
                stimLeft -= params.dtMs;
                if (stimLeft <= 0.0f) net.clearStimulus();
            }
        }
        // Neurons that spiked at least once this frame: flyviz sets these to
        // full activity, so a neuron firing twice in one frame looks the same.
        list.clear();
        for (std::uint32_t i = 0; i < n; ++i) {
            if (spiked[i]) list.push_back(i);
        }
        totalSpikes += list.size();
        w.varint(static_cast<std::uint32_t>(list.size()));
        std::uint32_t prev = 0;
        for (const auto i : list) { w.varint(i - prev); prev = i; }

        // Graded neurons: activity = min(1, output / 80), as in flyviz.
        // Only non-zero values are stored.
        const auto out = net.gradedOutput();
        list.clear();
        std::vector<std::uint8_t> vals;
        for (const auto j : net.gradedNeurons()) {
            const float a = std::min(1.0f, out[j] / 80.0f);
            const auto v = static_cast<std::uint8_t>(std::lround(std::max(a, 0.0f) * 255.0f));
            if (v > 0) { list.push_back(j); vals.push_back(v); }
        }
        w.varint(static_cast<std::uint32_t>(list.size()));
        prev = 0;
        for (std::size_t k = 0; k < list.size(); ++k) {
            w.varint(list[k] - prev);
            prev = list[k];
            w.u8(vals[k]);
        }
    }
    w.save(path);
    std::printf("brain: %d frames, %.1f ms, %llu neuron-frames active, %.2f MB -> %s\n",
                frames, frames * stepsPerFrame * params.dtMs,
                static_cast<unsigned long long>(totalSpikes), w.size() / 1e6, path.c_str());
    return w.size();
}

// Same run as `flybody --drop`: 0.1 ms neural steps, 1 ms physics steps,
// a 20 ms pulse. Physics runs alongside purely as a reference for the web
// build to be checked against.
std::size_t recordDrive(const Connectome& conn, std::uint32_t driven,
                        const std::string& motorMap, float bodyMs,
                        const std::string& path) {
    const LifParams params;
    LIFNetwork net(conn, params);
    auto pools = MotorPools::load(motorMap);
    FlyBody body;
    FlyPhysics phys;
    phys.build(body);
    phys.world.params.wallsOn = false;

    const float physDt = 0.001f;
    const int neuralPerPhys = static_cast<int>(1.0f / params.dtMs);
    const int steps = static_cast<int>(bodyMs);
    const float pulseMs = 20.0f;
    const std::size_t units = pools.unitCount();

    Writer w;
    w.tag("FLYD");
    w.u32(1);  // format version
    w.u32(static_cast<std::uint32_t>(units));
    w.u32(static_cast<std::uint32_t>(steps));
    w.f32(physDt * 1000.0f);  // ms per step
    w.f32(pulseMs);

    net.setStimulus(driven, 200.0f);
    float stimLeft = pulseMs;
    for (int i = 0; i < steps; ++i) {
        for (int k = 0; k < neuralPerPhys; ++k) {
            net.step();
            pools.accumulate(net);
            if (stimLeft > 0.0f) {
                stimLeft -= params.dtMs;
                if (stimLeft <= 0.0f) net.setStimulus(driven, 0.0f);
            }
        }
        pools.update(1.0f);
        for (const auto& m : pools.muscles()) {
            for (const auto& u : m.units) {
                w.u16(static_cast<std::uint16_t>(std::lround(std::clamp(u.activation, 0.0f, 1.0f) * 65535.0f)));
            }
        }
        phys.step(physDt, pools);
    }
    w.save(path);
    std::printf("drive: %zu motor units x %d ms, %.2f MB -> %s\n", units, steps,
                w.size() / 1e6, path.c_str());
    std::printf("reference physics: peak height %.4f mm, final %.4f mm\n",
                phys.peakHeight(), phys.bodyHeight());
    return w.size();
}

int run(int argc, char** argv) {
    std::string dataDir = "data/bin", outDir = "web/data";
    std::int64_t stimBody = 10001;
    int frames = 150, steps = 8;
    float bodyMs = 400.0f;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { printHelp(); return 0; }
        else if (a == "--data") dataDir = next();
        else if (a == "--out") outDir = next();
        else if (a == "--stim-body") stimBody = std::atoll(next().c_str());
        else if (a == "--brain-frames") frames = std::atoi(next().c_str());
        else if (a == "--steps") steps = std::atoi(next().c_str());
        else if (a == "--body-ms") bodyMs = std::stof(next());
        else throw std::runtime_error("unknown option: " + a);
    }

    dataDir = findDataDir(dataDir, argv[0]);
    const Connectome conn = Connectome::load(dataDir + "/cns.bin");
    const auto driven = conn.indexOf(stimBody);
    if (driven == UINT32_MAX) throw std::runtime_error("no neuron with that bodyId");
    std::printf("loaded %u neurons; driving bodyId %lld\n", conn.neuronCount(),
                static_cast<long long>(stimBody));

    recordBrain(conn, driven, frames, steps, outDir + "/brain.bin");
    recordDrive(conn, driven, dataDir + "/motor_map.tsv", bodyMs, outDir + "/drive.bin");

    // The web body loads the motor map itself (to build the same muscles the
    // drive was recorded against), so it travels with the recordings.
    std::filesystem::copy_file(dataDir + "/motor_map.tsv", outDir + "/motor_map.tsv",
                               std::filesystem::copy_options::overwrite_existing);
    std::printf("copied motor_map.tsv -> %s\n", outDir.c_str());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
