#pragma once
#include "orbit/OrbitState.h"

// Active altitude station-keeping: monitors semi-major axis decay and, once
// it exceeds a deadband, applies a tangential re-boost burn back to the
// target SMA. This turns the existing passive drag model into a closed
// control loop, so the reported ΔV reflects what actually operating the
// constellation costs (what a real ops team burns to hold the shell),
// rather than just how fast it would decay if left alone.
//
// RAAN / plane-keeping is deliberately not modeled here: within a single
// Walker shell every satellite shares the same semi-major axis and
// inclination, so their J2 nodal drift rates are identical (common-mode)
// and relative plane spacing is preserved with zero ΔV expenditure -- this
// is a basic property of Walker constellation design, not an omission.
// Constellations that DO need plane-keeping (mixed-altitude shells, or
// correcting injection dispersion) are outside this project's current scope.
namespace StationKeeping {

// Checks the satellite's current osculating SMA against nominal_sma_m and,
// if it has decayed by more than deadband_m, applies a single tangential
// burn (evaluated via vis-viva at the current position) to restore it.
// Mutates `state.velocity` in place. Returns the ΔV magnitude applied
// [m/s], or 0 if no burn was needed.
double maybeReboost(OrbitState& state, double nominal_sma_m, double deadband_m, double mu);

} // namespace StationKeeping
