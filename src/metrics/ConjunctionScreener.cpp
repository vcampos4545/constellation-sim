#include "metrics/ConjunctionScreener.h"
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"
#include <algorithm>
#include <iostream>

namespace {
constexpr int kPairCountWarningThreshold = 500'000;
}

ConjunctionScreener::ConjunctionScreener(const ConjunctionConfig& cfg,
                                         const std::vector<Satellite*>& sats)
    : cfg_(cfg) {
    buildCandidatePairs(sats);
}

void ConjunctionScreener::buildCandidatePairs(const std::vector<Satellite*>& sats) {
    const int n = static_cast<int>(sats.size());
    const double mu = Constants::GM_EARTH;
    const double margin_m = cfg_.threshold_km * 1000.0;

    // Perigee/apogee radius band per satellite, from initial state.
    std::vector<double> rp(n), ra(n);
    for (int i = 0; i < n; ++i) {
        const OrbitalElements e = OrbitalElements::fromStateVector(sats[i]->state(), mu);
        rp[i] = e.sma * (1.0 - e.ecc);
        ra[i] = e.sma * (1.0 + e.ecc);
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            // Radius bands (with margin) must overlap for a conjunction to
            // ever be possible, regardless of RAAN/phase evolution.
            const bool disjoint = (ra[i] + margin_m < rp[j]) || (ra[j] + margin_m < rp[i]);
            if (!disjoint) candidate_pairs_.emplace_back(i, j);
        }
    }

    if (static_cast<int>(candidate_pairs_.size()) > kPairCountWarningThreshold) {
        std::cerr << "Warning: conjunction screening has " << candidate_pairs_.size()
                  << " candidate pairs after pre-filtering; this may be slow. "
                  << "Consider a coarser metrics.conjunction.sample_interval_s.\n";
    }
}

void ConjunctionScreener::update(const std::vector<Satellite*>& sats, double time_s) {
    for (const auto& [i, j] : candidate_pairs_) {
        const double dist_km =
            (sats[i]->state().position - sats[j]->state().position).norm() / 1000.0;
        if (dist_km >= cfg_.threshold_km) continue;

        const int64_t key = static_cast<int64_t>(i) * 1'000'000 + j;
        auto it = tracked_.find(key);
        if (it == tracked_.end() || dist_km < it->second.min_distance_km) {
            tracked_[key] = Event{i, j, time_s, dist_km};
        }
    }
}

std::vector<ConjunctionScreener::Event> ConjunctionScreener::finalizeEvents() const {
    std::vector<Event> events;
    events.reserve(tracked_.size());
    for (const auto& [key, ev] : tracked_) events.push_back(ev);
    std::sort(events.begin(), events.end(),
             [](const Event& a, const Event& b) { return a.min_distance_km < b.min_distance_km; });
    return events;
}
