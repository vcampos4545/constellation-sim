#pragma once

#include "core/ConfigLoader.h"
#include "core/ThreadPool.h"
#include "montecarlo/ParameterSweep.h"
#include "montecarlo/SimulationBatch.h"
#include "output/OutputManager.h"
#include <atomic>
#include <optional>
#include <functional>
#include <mutex>

// Orchestrates a Monte Carlo experiment:
//   1. Generates all SimConfigs via ParameterSweep
//   2. Submits each simulation to the ThreadPool
//   3. Writes results to OutputManager as they complete
//   4. Prints progress to stdout
class ExperimentManager {
public:
    using ProgressCallback = std::function<void(int completed, int total)>;

    ExperimentManager(const MCConfig& mc_cfg,
                      const std::string& output_base = "output");

    void run();

    void setProgressCallback(ProgressCallback cb) { progress_cb_ = std::move(cb); }

    int totalRuns()    const { return total_runs_; }
    int completedRuns() const { return completed_.load(); }

private:
    MCConfig       mc_cfg_;
    OutputManager  output_;
    std::atomic<int> completed_{0};
    int            total_runs_{0};
    std::optional<ProgressCallback> progress_cb_;

    // Populated only when mc_cfg_.sampling == "pareto", for post-run
    // Pareto-frontier extraction.
    std::mutex results_mutex_;
    std::vector<ConstellationResult> collected_results_;

    void runOne(int run_id, const SimConfig& cfg);
    void writeParetoFrontier();
};
