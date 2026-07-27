#pragma once
#include "metrics/MetricsCollector.h"
#include <string>
#include <vector>

// Multi-objective post-processing for a completed Monte Carlo sweep.
//
// Rather than a separate search algorithm, this treats the sweep's full
// parameter grid as the candidate set and extracts the Pareto-optimal
// subset by the requested objectives -- e.g. "which of the runs I already
// computed are not strictly worse, on every axis, than some other run."
// This answers questions like "minimum satellites for >=95% coverage
// under a given ΔV budget" directly: filter the frontier for the
// constraint, then read off the extremes.
namespace ParetoFrontier {

struct Objective {
    std::string field;      // ConstellationResult field name, e.g. "coverage_pct"
    bool        maximize;   // true = higher is better, false = lower is better
};

// Parses specs like "maximize:coverage_pct" / "minimize:total_satellites".
// Throws std::runtime_error on a malformed spec or unknown field name.
std::vector<Objective> parseObjectives(const std::vector<std::string>& specs);

// Extracts a named numeric field from a ConstellationResult.
// Throws std::runtime_error if the field name is not recognized.
double extractField(const ConstellationResult& cr, const std::string& field);

// Returns the indices (into `results`) of the non-dominated (Pareto-optimal)
// runs for the given objectives. A result is non-dominated if no other
// result is at least as good on every objective and strictly better on at
// least one.
std::vector<int> nonDominated(const std::vector<ConstellationResult>& results,
                              const std::vector<Objective>& objectives);

} // namespace ParetoFrontier
