#include "montecarlo/ParameterSweep.h"
#include <random>
#include <stdexcept>
#include <numeric>

ParameterSweep::ParameterSweep(const MCConfig& mc_cfg) : mc_cfg_(mc_cfg) {
    // "pareto" mode enumerates the full parameter grid too -- Pareto-optimal
    // extraction is a post-processing step over that same candidate set,
    // not a different config-generation strategy.
    if (mc_cfg_.sampling == "grid" || mc_cfg_.sampling == "pareto") {
        // Total runs = product of all parameter list lengths
        total_runs_ = 1;
        for (const auto& p : mc_cfg_.parameters) {
            total_runs_ *= static_cast<int>(p.values.size());
        }
    } else {
        total_runs_ = (mc_cfg_.runs > 0) ? mc_cfg_.runs : 100;
    }
}

std::vector<SimConfig> ParameterSweep::generateConfigs() const {
    if (mc_cfg_.sampling == "grid" || mc_cfg_.sampling == "pareto") return gridSweep();
    return randomSweep();
}

// Enumerate all combinations of parameter values (Cartesian product).
std::vector<SimConfig> ParameterSweep::gridSweep() const {
    std::vector<SimConfig> configs;
    configs.push_back(mc_cfg_.base_config);

    for (const auto& param : mc_cfg_.parameters) {
        std::vector<SimConfig> expanded;
        for (const auto& base_cfg : configs) {
            for (double val : param.values) {
                SimConfig c = base_cfg;
                ConfigLoader::applyParameter(c, param.name, val);
                expanded.push_back(c);
            }
        }
        configs = std::move(expanded);
    }

    // Assign unique run names
    for (int i = 0; i < static_cast<int>(configs.size()); ++i) {
        configs[i].run_name = mc_cfg_.experiment_name + "_" + std::to_string(i);
    }
    return configs;
}

// Random sampling: independently sample each parameter.
std::vector<SimConfig> ParameterSweep::randomSweep() const {
    std::vector<SimConfig> configs;
    configs.reserve(total_runs_);

    std::mt19937 rng(42);  // fixed seed for reproducibility

    for (int r = 0; r < total_runs_; ++r) {
        SimConfig c = mc_cfg_.base_config;
        for (const auto& param : mc_cfg_.parameters) {
            if (param.values.empty()) continue;
            std::uniform_int_distribution<int> dist(0, static_cast<int>(param.values.size()) - 1);
            ConfigLoader::applyParameter(c, param.name, param.values[dist(rng)]);
        }
        c.run_name = mc_cfg_.experiment_name + "_" + std::to_string(r);
        configs.push_back(c);
    }
    return configs;
}
