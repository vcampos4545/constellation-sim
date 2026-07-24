#include "constellation/Constellation.h"
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"
#include <map>

Constellation Constellation::createWalker(const WalkerConfig& cfg, int start_id) {
    Constellation c;

    const int T = cfg.total_satellites;
    const int P = cfg.planes;
    const int S = T / P;   // satellites per plane (assume integer division)
    const int F = cfg.phasing_factor;

    const double sma_m     = (Constants::EARTH_RADIUS_M + cfg.altitude_km * 1000.0);
    const double inc_rad   = cfg.inclination_deg * Constants::DEG2RAD;
    const double aop_rad   = cfg.arg_of_perigee_deg * Constants::DEG2RAD;
    const double mu        = Constants::GM_EARTH;

    const double d_raan = Constants::TWO_PI / P;      // RAAN step between planes
    const double d_nu   = Constants::TWO_PI / S;      // mean anomaly step within plane
    const double d_phase = F * Constants::TWO_PI / T; // cross-plane phasing offset

    c.planes_.reserve(P);
    int global_id = start_id;

    for (int p = 0; p < P; ++p) {
        const double raan = p * d_raan;
        Plane plane(p, inc_rad, raan);
        plane.satellites().reserve(S);

        for (int s = 0; s < S; ++s) {
            // True anomaly: in-plane spacing + cross-plane phase offset
            const double ta_rad = std::fmod(s * d_nu + p * d_phase, Constants::TWO_PI);

            OrbitalElements elems;
            elems.sma  = sma_m;
            elems.ecc  = cfg.eccentricity;
            elems.inc  = inc_rad;
            elems.raan = raan;
            elems.aop  = aop_rad;
            elems.ta   = ta_rad;

            const OrbitState state = elems.toStateVector(mu);

            PhysicalProperties props;  // uses defaults; caller should override if needed
            plane.addSatellite(Satellite(global_id++, p, s, state, props));
        }
        c.planes_.push_back(std::move(plane));
    }

    c.rebuildSatPtrs();
    return c;
}

void Constellation::rebuildSatPtrs() {
    sat_ptrs_.clear();
    for (auto& plane : planes_) {
        for (auto& sat : plane.satellites()) {
            sat_ptrs_.push_back(&sat);
        }
    }
}

Constellation Constellation::createCustom(const std::vector<SatelliteSpec>& specs)
{
    Constellation c;
    const double mu = Constants::GM_EARTH;

    // Map plane_id → index in c.planes_ so satellites can be placed into planes.
    std::map<int, int> plane_idx_by_id;

    for (int i = 0; i < static_cast<int>(specs.size()); ++i) {
        const SatelliteSpec& spec = specs[i];

        OrbitalElements elems;
        elems.sma  = spec.sma_km * 1000.0;
        elems.ecc  = spec.eccentricity;
        elems.inc  = spec.inclination_deg    * Constants::DEG2RAD;
        elems.raan = spec.raan_deg           * Constants::DEG2RAD;
        elems.aop  = spec.arg_of_perigee_deg * Constants::DEG2RAD;
        elems.ta   = spec.true_anomaly_deg   * Constants::DEG2RAD;

        const OrbitState state = elems.toStateVector(mu);

        // Create the plane on first encounter
        if (plane_idx_by_id.find(spec.plane_id) == plane_idx_by_id.end()) {
            int idx = static_cast<int>(c.planes_.size());
            plane_idx_by_id[spec.plane_id] = idx;
            c.planes_.emplace_back(spec.plane_id, elems.inc, elems.raan);
        }

        PhysicalProperties props;  // caller overrides via SimulationEngine
        c.planes_[plane_idx_by_id[spec.plane_id]].addSatellite(
            Satellite(i, spec.plane_id, spec.seat_id, state, props));
    }

    c.rebuildSatPtrs();
    return c;
}
