#include "montecarlo/ExperimentManager.h"
#include "montecarlo/ParetoFrontier.h"
#include "core/SimulationEngine.h"
#include <iostream>
#include <future>
#include <vector>

ExperimentManager::ExperimentManager(const MCConfig& mc_cfg,
                                      const std::string& output_base)
    : mc_cfg_(mc_cfg),
      output_(output_base.empty() ? mc_cfg.output_directory : output_base,
              mc_cfg.experiment_name)
{
    ParameterSweep sweep(mc_cfg_);
    total_runs_ = sweep.totalRuns();
}

void ExperimentManager::runOne(int run_id, const SimConfig& cfg) {
    SimulationEngine engine(cfg);
    ConstellationResult cr = engine.run(run_id);
    output_.writeRun(run_id, cr, engine.satelliteResults(), engine.groundTargetResults());

    if (mc_cfg_.sampling == "pareto") {
        std::lock_guard lock(results_mutex_);
        collected_results_.push_back(cr);
    }

    const int done = ++completed_;
    if (progress_cb_) (*progress_cb_)(done, total_runs_);

    std::cout << "[MC] run " << run_id << "/" << total_runs_
              << " complete — " << cfg.run_name
              << "  coverage=" << cr.coverage_pct << "%"
              << "  revisit=" << (cr.revisit_time_avg_s/60.0) << " min\n";
}

void ExperimentManager::writeParetoFrontier() {
    if (mc_cfg_.pareto_objectives.empty()) {
        std::cerr << "[MC] sampling=\"pareto\" but no pareto_objectives configured; "
                  << "skipping frontier extraction (results are still in experiment_summary.csv).\n";
        return;
    }
    try {
        const auto objectives = ParetoFrontier::parseObjectives(mc_cfg_.pareto_objectives);
        const auto frontier_idx = ParetoFrontier::nonDominated(collected_results_, objectives);

        std::vector<ConstellationResult> frontier;
        frontier.reserve(frontier_idx.size());
        for (int i : frontier_idx) frontier.push_back(collected_results_[i]);

        output_.writeParetoFrontier(frontier, mc_cfg_.pareto_objectives);
        std::cout << "[MC] Pareto frontier: " << frontier.size() << " / "
                  << collected_results_.size() << " runs are non-dominated.\n";
    } catch (const std::exception& e) {
        std::cerr << "[MC] Pareto frontier extraction failed: " << e.what() << "\n";
    }
}

void ExperimentManager::run() {
    ParameterSweep sweep(mc_cfg_);
    const auto configs = sweep.generateConfigs();
    total_runs_ = static_cast<int>(configs.size());
    completed_  = 0;

    const int num_threads = (mc_cfg_.threads > 0)
        ? mc_cfg_.threads
        : static_cast<int>(std::thread::hardware_concurrency());

    std::cout << "[MC] " << mc_cfg_.name
              << ": " << total_runs_ << " runs on " << num_threads << " threads\n";

    if (num_threads <= 1 || total_runs_ <= 1) {
        // Serial fallback — simpler stack traces and useful for debugging
        for (int i = 0; i < total_runs_; ++i) {
            runOne(i, configs[i]);
        }
    } else {
        ThreadPool pool(static_cast<unsigned int>(num_threads));
        std::vector<std::future<void>> futures;
        futures.reserve(total_runs_);

        for (int i = 0; i < total_runs_; ++i) {
            futures.push_back(
                pool.submit([this, i, &configs] { runOne(i, configs[i]); })
            );
        }
        // Wait for all runs to complete
        for (auto& f : futures) f.get();
    }

    if (mc_cfg_.sampling == "pareto") writeParetoFrontier();

    output_.finalize();
    std::cout << "[MC] experiment complete. Results in: "
              << output_.experimentDir() << "\n";
}
