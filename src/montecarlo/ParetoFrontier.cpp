#include "montecarlo/ParetoFrontier.h"
#include <stdexcept>
#include <unordered_map>
#include <functional>

namespace ParetoFrontier {

namespace {
using Accessor = std::function<double(const ConstellationResult&)>;

const std::unordered_map<std::string, Accessor>& fieldRegistry() {
    static const std::unordered_map<std::string, Accessor> registry = {
        {"altitude_km",                 [](const ConstellationResult& c) { return c.altitude_km; }},
        {"inclination_deg",             [](const ConstellationResult& c) { return c.inclination_deg; }},
        {"total_satellites",            [](const ConstellationResult& c) { return static_cast<double>(c.total_satellites); }},
        {"planes",                      [](const ConstellationResult& c) { return static_cast<double>(c.planes); }},
        {"sats_per_plane",              [](const ConstellationResult& c) { return static_cast<double>(c.sats_per_plane); }},
        {"coverage_pct",                [](const ConstellationResult& c) { return c.coverage_pct; }},
        {"revisit_time_avg_s",          [](const ConstellationResult& c) { return c.revisit_time_avg_s; }},
        {"revisit_time_max_s",          [](const ConstellationResult& c) { return c.revisit_time_max_s; }},
        {"avg_pass_duration_s",         [](const ConstellationResult& c) { return c.avg_pass_duration_s; }},
        {"avg_elevation_deg",           [](const ConstellationResult& c) { return c.avg_elevation_deg; }},
        {"avg_nearest_neighbor_km",     [](const ConstellationResult& c) { return c.avg_nearest_neighbor_km; }},
        {"avg_drag_dv_ms",              [](const ConstellationResult& c) { return c.avg_drag_dv_ms; }},
        {"avg_sk_dv_ms",                [](const ConstellationResult& c) { return c.avg_sk_dv_ms; }},
        {"avg_annual_sk_dv_ms_per_year",[](const ConstellationResult& c) { return c.avg_annual_sk_dv_ms_per_year; }},
        {"avg_sunlit_pct",              [](const ConstellationResult& c) { return c.avg_sunlit_pct; }},
        {"avg_altitude_km",             [](const ConstellationResult& c) { return c.avg_altitude_km; }},
        {"min_altitude_km",             [](const ConstellationResult& c) { return c.min_altitude_km; }},
        {"deployment_dv_per_sat_ms",    [](const ConstellationResult& c) { return c.deployment_dv_per_sat_ms; }},
    };
    return registry;
}
} // namespace

double extractField(const ConstellationResult& cr, const std::string& field) {
    const auto& registry = fieldRegistry();
    const auto it = registry.find(field);
    if (it == registry.end())
        throw std::runtime_error("ParetoFrontier: unknown objective field '" + field + "'");
    return it->second(cr);
}

std::vector<Objective> parseObjectives(const std::vector<std::string>& specs) {
    std::vector<Objective> objectives;
    for (const auto& spec : specs) {
        const auto colon = spec.find(':');
        if (colon == std::string::npos)
            throw std::runtime_error("ParetoFrontier: objective '" + spec +
                                     "' must be \"maximize:<field>\" or \"minimize:<field>\"");
        const std::string dir = spec.substr(0, colon);
        const std::string field = spec.substr(colon + 1);

        bool maximize;
        if (dir == "maximize" || dir == "max") maximize = true;
        else if (dir == "minimize" || dir == "min") maximize = false;
        else throw std::runtime_error("ParetoFrontier: objective direction must be 'maximize' or 'minimize', got '" + dir + "'");

        // Validate the field name eagerly so bad configs fail fast.
        if (fieldRegistry().find(field) == fieldRegistry().end())
            throw std::runtime_error("ParetoFrontier: unknown objective field '" + field + "'");

        objectives.push_back({field, maximize});
    }
    return objectives;
}

std::vector<int> nonDominated(const std::vector<ConstellationResult>& results,
                              const std::vector<Objective>& objectives) {
    const int n = static_cast<int>(results.size());
    std::vector<int> frontier;
    if (objectives.empty()) return frontier;

    // Precompute objective values once per result.
    std::vector<std::vector<double>> values(n, std::vector<double>(objectives.size()));
    for (int i = 0; i < n; ++i)
        for (size_t k = 0; k < objectives.size(); ++k)
            values[i][k] = extractField(results[i], objectives[k].field);

    auto dominates = [&](int a, int b) {
        // a dominates b: a is at least as good on every objective and
        // strictly better on at least one.
        bool strictly_better = false;
        for (size_t k = 0; k < objectives.size(); ++k) {
            const double va = values[a][k];
            const double vb = values[b][k];
            const bool as_good = objectives[k].maximize ? (va >= vb) : (va <= vb);
            if (!as_good) return false;
            if (va != vb) strictly_better = true;
        }
        return strictly_better;
    };

    for (int i = 0; i < n; ++i) {
        bool dominated = false;
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            if (dominates(j, i)) { dominated = true; break; }
        }
        if (!dominated) frontier.push_back(i);
    }
    return frontier;
}

} // namespace ParetoFrontier
