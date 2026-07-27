#include "core/ConfigLoader.h"
#include "core/SimulationEngine.h"
#include "montecarlo/ExperimentManager.h"
#include "output/OutputManager.h"

#ifdef CONSTELLATION_VIZ_ENABLED
#include "viz/Renderer.h"
#endif

#include <iostream>
#include <fstream>
#include <future>
#include <string>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

struct CliArgs {
    std::string config_path;
    bool        visualize{false};
    bool        is_montecarlo{false};
};

static CliArgs parseArgs(int argc, char* argv[]) {
    CliArgs args;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--visualize" || arg == "-v") {
            args.visualize = true;
        } else if (arg.starts_with("--")) {
            std::cerr << "Unknown flag: " << arg << "\n";
        } else {
            args.config_path = arg;
        }
    }
    return args;
}

static bool isMonteCarlo(const std::string& path) {
    try {
        std::ifstream f(path);
        if (!f.is_open()) return false;
        nlohmann::json j;
        f >> j;
        return j.contains("experiment") || j.contains("sweep");
    } catch (...) {
        return false;
    }
}

static void printUsage(const char* prog) {
    std::cerr
        << "Usage:\n"
        << "  " << prog << " <config.json>             — single simulation (headless)\n"
        << "  " << prog << " <config.json> --visualize — single simulation + playback\n"
        << "  " << prog << " <mc_config.json>          — Monte Carlo (always headless)\n"
        << "\n"
        << "Visualizer controls (when --visualize):\n"
        << "  Space        — pause / resume\n"
        << "  1 / 2 / 3 / 4 — speed 1×, 10×, 100×, 1000×\n"
        << "  + / -        — step speed up / down\n"
        << "  Left-drag    — orbit camera\n"
        << "  Right-drag   — pan\n"
        << "  Scroll       — zoom\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) { printUsage(argv[0]); return 1; }

    CliArgs args = parseArgs(argc, argv);
    if (args.config_path.empty()) { printUsage(argv[0]); return 1; }

    if (!fs::exists(args.config_path)) {
        std::cerr << "Error: config file not found: " << args.config_path << "\n";
        return 1;
    }

#ifndef CONSTELLATION_VIZ_ENABLED
    if (args.visualize) {
        std::cerr << "Warning: --visualize requested but not compiled in "
                     "(rebuild with -DBUILD_VIZ=ON).\n";
        args.visualize = false;
    }
#endif

    args.is_montecarlo = isMonteCarlo(args.config_path);

    // -----------------------------------------------------------------------
    // Monte Carlo path (always headless)
    // -----------------------------------------------------------------------
    if (args.is_montecarlo) {
        if (args.visualize)
            std::cout << "Note: --visualize ignored for Monte Carlo runs.\n";
        try {
            MCConfig mc = ConfigLoader::loadMCConfig(args.config_path);
            ExperimentManager mgr(mc);
            mgr.run();
        } catch (const std::exception& e) {
            std::cerr << "Monte Carlo failed: " << e.what() << "\n";
            return 1;
        }
        return 0;
    }

    // -----------------------------------------------------------------------
    // Single simulation
    // -----------------------------------------------------------------------
    try {
        SimConfig cfg = ConfigLoader::loadSimConfig(args.config_path);

#ifdef CONSTELLATION_VIZ_ENABLED
        if (args.visualize) {
            std::cout << "Starting simulation + live visualization (Space=pause, 1-4=speed)...\n";

            auto queue = std::make_shared<FrameQueue>();
            SimulationEngine engine(cfg);

            // Capture static satellite metadata before launching the sim thread.
            auto sat_info = engine.satelliteInfo();

            engine.setFrameCallback([queue](const SimulationEngine::FrameData& f) {
                queue->push(f);
            });

            // Run simulation on a background thread; window opens immediately.
            std::future<ConstellationResult> sim_future =
                std::async(std::launch::async, [&engine, queue]() {
                    ConstellationResult cr = engine.run(0);
                    queue->markSimDone();
                    return cr;
                });

            {
                Renderer renderer(queue, cfg.ground_targets,
                                  cfg.metrics.coverage.min_elevation_deg,
                                  std::move(sat_info), cfg.epoch_jd);
                renderer.run();
            }

            // Wait for simulation to finish (it's likely done already) then write output.
            const ConstellationResult cr = sim_future.get();
            OutputManager output(cfg.output_directory, cfg.run_name);
            output.writeRun(0, cr, engine.satelliteResults(), engine.groundTargetResults());
            if (!engine.trajectorySnapshots().empty())
                output.writeTrajectory(0, engine.trajectorySnapshots());
            if (!engine.ephemerisSamples().empty())
                output.writeOem(0, engine.ephemerisSamples(), cfg.epoch_jd);
            output.finalize();

            std::cout << "Output: " << cfg.output_directory << "/" << cfg.run_name << "\n";
            return 0;
        }
#endif

        // Headless single simulation
        SimulationEngine engine(cfg);
        ConstellationResult cr = engine.run(0);

        OutputManager output(cfg.output_directory, cfg.run_name);
        output.writeRun(0, cr, engine.satelliteResults(), engine.groundTargetResults());
        if (!engine.trajectorySnapshots().empty())
            output.writeTrajectory(0, engine.trajectorySnapshots());
        if (!engine.passEvents().empty())
            output.writePassEvents(0, engine.passEvents());
        if (!engine.ephemerisSamples().empty())
            output.writeOem(0, engine.ephemerisSamples(), cfg.epoch_jd);
        if (cfg.metrics.conjunction.enabled)
            output.writeConjunctions(0, engine.conjunctionEvents());
        output.finalize();

        std::cout << "\nResults:\n"
                  << "  Coverage:    " << cr.coverage_pct            << " %\n"
                  << "  Avg revisit: " << cr.revisit_time_avg_s/60.0 << " min\n"
                  << "  Avg drag ΔV: " << cr.avg_drag_dv_ms          << " m/s/day\n"
                  << "  Output:      " << cfg.output_directory << "/" << cfg.run_name << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Simulation failed: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
