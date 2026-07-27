#pragma once
#include "core/ConfigLoader.h"
#include "orbit/Satellite.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

// Pairwise closest-approach (conjunction) screening within a constellation.
//
// This is a *discrete* screen: it checks pairwise separation at
// sample_interval_s and records the minimum observed separation per pair.
// True closest approach between samples can be missed -- for a rigorous
// analytic time-of-closest-approach, refine candidate events with a finer
// search around the reported time. This is the right fidelity level for
// trade-study screening (which pairs/regions are worth a closer look),
// not for operational conjunction assessment.
//
// Cost: O(N^2) per sample after a one-time radius-band pre-filter that
// discards pairs whose orbits can never come within threshold_km (e.g.
// different altitude shells). Within a single Walker shell the pre-filter
// does not help -- keep sample_interval_s coarse for large constellations.
class ConjunctionScreener {
public:
    struct Event {
        int    sat_i{0};
        int    sat_j{0};
        double time_of_min_dist_s{0.0};
        double min_distance_km{0.0};
    };

    // Builds the one-time candidate-pair pre-filter from the satellites'
    // initial orbital state. Call once before propagation begins.
    ConjunctionScreener(const ConjunctionConfig& cfg, const std::vector<Satellite*>& sats);

    // Call at cfg.sample_interval_s cadence with current satellite states.
    void update(const std::vector<Satellite*>& sats, double time_s);

    // All pairs that were ever observed closer than threshold_km, sorted by
    // minimum distance (closest first).
    std::vector<Event> finalizeEvents() const;

    int candidatePairCount() const { return static_cast<int>(candidate_pairs_.size()); }

private:
    ConjunctionConfig cfg_;
    std::vector<std::pair<int,int>> candidate_pairs_;
    std::unordered_map<int64_t, Event> tracked_;

    void buildCandidatePairs(const std::vector<Satellite*>& sats);
};
