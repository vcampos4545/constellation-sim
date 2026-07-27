#include "orbit/StationKeeping.h"
#include "orbit/OrbitalElements.h"
#include <cmath>

namespace StationKeeping {

double maybeReboost(OrbitState& state, double nominal_sma_m, double deadband_m, double mu) {
    const OrbitalElements elems = OrbitalElements::fromStateVector(state, mu);
    if (nominal_sma_m - elems.sma <= deadband_m) return 0.0;

    const double r = state.position.norm();
    const double v_current = state.velocity.norm();

    // Vis-viva: speed needed at the current radius for an orbit whose
    // semi-major axis is nominal_sma_m.
    const double v_needed_sq = mu * (2.0 / r - 1.0 / nominal_sma_m);
    if (v_needed_sq <= 0.0) return 0.0;
    const double v_needed = std::sqrt(v_needed_sq);

    const double dv = v_needed - v_current;
    if (dv <= 0.0) return 0.0;

    state.velocity = state.velocity.normalized() * v_needed;
    return dv;
}

} // namespace StationKeeping
