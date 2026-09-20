// Headless driver: load the connectome, drive some neurons, report who fires.
//
//   flysim --stim-type DNp01 --rate 250 --duration 200
//   flysim --stim-body 10001 --epsp 0.3 --top 40

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/Connectome.h"
#include "core/DataPath.h"
#include "core/LIFNetwork.h"
#include "core/NeuronNames.h"

namespace {

[[noreturn]] void usage() {
    std::cout <<
        "flysim -- LIF simulation over the male-cns:v1.0 connectome\n\n"
        "  --data DIR         directory holding cns.bin (default: data/bin)\n"
        "  --stim-type NAME   drive every neuron of this cell type (repeatable)\n"
        "  --stim-body ID     drive one neuron by neuPrint bodyId (repeatable)\n"
        "  --rate MV_PER_MS   stimulus strength (default: 200)\n"
        "  --duration MS      simulated milliseconds (default: 200)\n"
        "  --epsp MV          depolarisation per synapse (default: 0.275)\n"
        "  --dt MS            timestep (default: 0.1)\n"
        "  --adapt MV         threshold rise per spike, 0 disables (default: 0.4)\n"
        "  --tau-adapt MS     adaptation decay (default: 150)\n"
        "  --size-exp E       size-scaled excitability, 0 disables (default: 0.5)\n"
        "  --graded-rate HZ   graded output at threshold (default: 50)\n"
        "  --top N            how many active neurons to list (default: 25)\n"
        "  --targets N        also report the N strongest direct targets of the\n"
        "                     stimulated neurons, by connection weight\n"
        "  --list-types SUB   print cell types containing SUB, then exit\n";
    std::exit(0);
}

double argNum(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        std::cerr << "missing value for " << argv[i] << "\n";
        std::exit(1);
    }
    return std::stod(argv[++i]);
}

std::string argStr(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        std::cerr << "missing value for " << argv[i] << "\n";
        std::exit(1);
    }
    return argv[++i];
}

}  // namespace

int main(int argc, char** argv) {
    std::string dataDir = "data/bin";
    std::vector<std::string> stimTypes;
    std::vector<std::int64_t> stimBodies;
    std::string listTypes;
    double rate = 200.0, durationMs = 200.0;
    std::size_t top = 25;
    std::size_t nTargets = 0;
    fly::LifParams p;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") usage();
        else if (a == "--data") dataDir = argStr(argc, argv, i);
        else if (a == "--stim-type") stimTypes.push_back(argStr(argc, argv, i));
        else if (a == "--stim-body") stimBodies.push_back(
                     static_cast<std::int64_t>(argNum(argc, argv, i)));
        else if (a == "--rate") rate = argNum(argc, argv, i);
        else if (a == "--duration") durationMs = argNum(argc, argv, i);
        else if (a == "--epsp") p.epspPerSynapse = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--dt") p.dtMs = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--adapt") p.adaptIncrement = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--tau-adapt") p.tauAdapt = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--size-exp") p.sizeExponent = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--graded-rate") p.gradedRateAtThreshold = static_cast<float>(argNum(argc, argv, i));
        else if (a == "--top") top = static_cast<std::size_t>(argNum(argc, argv, i));
        else if (a == "--targets") nTargets = static_cast<std::size_t>(argNum(argc, argv, i));
        else if (a == "--list-types") listTypes = argStr(argc, argv, i);
        else {
            std::cerr << "unknown option: " << a << "\n";
            return 1;
        }
    }

    try {
        dataDir = fly::findDataDir(dataDir, argv[0]);
        const auto conn = fly::Connectome::load(dataDir + "/cns.bin");
        bool haveNames = false;
        const auto names = fly::NeuronNames::load(dataDir + "/cns_names.tsv",
                                             conn.neuronCount(), &haveNames);
        if (!haveNames) {
            std::cerr << "warning: cns_names.tsv not found, neurons unlabelled"
                      << "\n";
        }

        if (!listTypes.empty()) {
            std::vector<std::string> hits;
            for (const auto& entry : names.byType()) {
                if (entry.first.find(listTypes) != std::string::npos) {
                    hits.push_back(entry.first + "  (" +
                                   std::to_string(entry.second.size()) + " neurons)");
                }
            }
            std::sort(hits.begin(), hits.end());
            for (const auto& h : hits) std::cout << h << "\n";
            std::cout << hits.size() << " matching cell types\n";
            return 0;
        }

        std::uint32_t nGraded = 0;
        for (std::uint32_t i = 0; i < conn.neuronCount(); ++i) {
            if (conn.isGraded(i)) ++nGraded;
        }
        std::printf("connectome: %u neurons (%u graded), %llu edges (weight >= %u)\n",
                    conn.neuronCount(), nGraded,
                    static_cast<unsigned long long>(conn.edgeCount()),
                    conn.minWeight());
        std::printf("model: adapt=%.2f mV/spike tau=%.0f ms, size-exp=%.2f, "
                    "graded=%.0f Hz at threshold\n",
                    p.adaptIncrement, p.tauAdapt, p.sizeExponent,
                    p.gradedRateAtThreshold);

        fly::LIFNetwork net(conn, p);

        std::vector<std::uint32_t> driven;
        for (const auto& t : stimTypes) {
            const auto of = names.ofType(t);
            if (of.empty()) {
                std::cerr << "no cell type named " << t
                          << " -- try --list-types " << t << "\n";
                return 1;
            }
            driven.insert(driven.end(), of.begin(), of.end());
        }
        for (const auto id : stimBodies) {
            const auto idx = conn.indexOf(id);
            if (idx == UINT32_MAX) {
                std::cerr << "no neuron with bodyId " << id << "\n";
                return 1;
            }
            driven.push_back(idx);
        }
        if (driven.empty()) {
            std::cerr << "nothing to stimulate -- pass --stim-type or --stim-body\n";
            return 1;
        }
        for (const auto i : driven) net.setStimulus(i, static_cast<float>(rate));
        std::printf("driving %zu neurons at %.0f mV/ms for %.0f ms "
                    "(dt=%.2f, epsp=%.3f mV)\n",
                    driven.size(), rate, durationMs, p.dtMs, p.epspPerSynapse);

        const auto steps = static_cast<std::uint64_t>(durationMs / p.dtMs);
        const auto t0 = std::chrono::steady_clock::now();
        std::uint64_t totalSpikes = 0, totalGradedUpdates = 0;
        for (std::uint64_t s = 0; s < steps; ++s) {
            const auto st = net.step();
            totalSpikes += st.spikeCount;
            totalGradedUpdates += st.gradedUpdates;
        }
        const double wall = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - t0).count();

        std::printf("\n%llu steps in %.2f s  (%.2fx real time)\n",
                    static_cast<unsigned long long>(steps), wall,
                    (durationMs / 1000.0) / (wall > 0 ? wall : 1e-9));
        std::printf("%llu spikes total, mean rate %.2f Hz across the network\n",
                    static_cast<unsigned long long>(totalSpikes),
                    totalSpikes / (conn.neuronCount() * durationMs / 1000.0));

        if (nGraded > 0) {
            const auto gout = net.gradedOutput();
            double sum = 0.0, peak = 0.0;
            std::size_t engaged = 0;
            for (const auto j : net.gradedNeurons()) {
                sum += gout[j];
                peak = std::max(peak, static_cast<double>(gout[j]));
                if (gout[j] > 0.5) ++engaged;
            }
            std::printf("graded: %zu of %u active, mean %.1f Hz-equiv, peak %.1f, "
                        "%llu propagations\n",
                        engaged, nGraded, sum / (nGraded ? nGraded : 1), peak,
                        static_cast<unsigned long long>(totalGradedUpdates));
        }
        std::printf("\n");

        const auto totals = net.spikeTotals();
        std::vector<std::uint32_t> order(totals.size());
        std::iota(order.begin(), order.end(), 0u);
        const auto k = std::min(top, order.size());
        std::partial_sort(order.begin(), order.begin() + k, order.end(),
                          [&](std::uint32_t a, std::uint32_t b) {
                              return totals[a] > totals[b];
                          });

        std::printf("%-5s %-12s %-24s %-20s %8s %9s\n",
                    "rank", "bodyId", "type", "superclass", "spikes", "Hz");
        for (std::size_t r = 0; r < k; ++r) {
            const auto i = order[r];
            if (totals[i] == 0) break;
            std::printf("%-5zu %-12lld %-24s %-20s %8u %9.1f\n",
                        r + 1, static_cast<long long>(conn.bodyIds()[i]),
                        names.label(i).c_str(),
                        names.superclass(i).c_str(),
                        totals[i], totals[i] / (durationMs / 1000.0));
        }

        // Ranking by spike count answers "what is loudest", which in a
        // recurrent network is often not "what did the stimulus reach". This
        // walks the actual out-edges instead, so a silent target shows up as a
        // zero rather than by being absent from the table above.
        if (nTargets > 0) {
            struct Target { std::uint32_t idx; std::uint16_t w; };
            std::vector<Target> ts;
            for (const auto j : driven) {
                const auto cols = conn.targetsOf(j);
                const auto ws = conn.weightsOf(j);
                for (std::size_t e = 0; e < cols.size(); ++e) ts.push_back({cols[e], ws[e]});
            }
            std::sort(ts.begin(), ts.end(),
                      [](const Target& a, const Target& b) { return a.w > b.w; });
            const auto m = std::min(nTargets, ts.size());
            std::printf("\ndirect targets of the stimulated neurons "
                        "(%zu partners, strongest %zu):\n", ts.size(), m);
            std::printf("%-5s %-12s %-24s %-20s %7s %8s\n",
                        "rank", "bodyId", "type", "superclass", "synapses", "spikes");
            for (std::size_t r = 0; r < m; ++r) {
                const auto i = ts[r].idx;
                std::printf("%-5zu %-12lld %-24s %-20s %7u %8u\n",
                            r + 1, static_cast<long long>(conn.bodyIds()[i]),
                            names.label(i).c_str(),
                            names.superclass(i).c_str(),
                            ts[r].w, totals[i]);
            }
        }

        std::size_t active = 0;
        for (const auto t : totals) {
            if (t) ++active;
        }
        std::printf("\n%zu of %u neurons fired at least once (%.1f%%)\n",
                    active, conn.neuronCount(), 100.0 * active / conn.neuronCount());
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
