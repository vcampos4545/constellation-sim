#pragma once
#include <cmath>

// Simple free-space link-budget model: EIRP + ground-terminal G/T over a
// free-space path, with Shannon-capacity data rate as the achievable-rate
// figure of merit.
//
// This is a trade-study capacity estimate, not an RF engineering design
// tool: no rain fade, pointing loss, modulation/coding efficiency, or
// antenna pattern beyond an ideal isotropic/fixed-gain assumption. It
// turns "is a satellite geometrically visible" into "how much data could
// actually move through that link right now," which is what mission
// designers actually care about when comparing constellation shapes.
namespace LinkBudget {

struct Config {
    bool   enabled          = false;
    double frequency_ghz    = 12.0;  // Ku-band downlink, typical LEO comms
    double eirp_dbw         = 45.0;  // satellite effective isotropic radiated power
    double gt_dbk           = 15.0;  // ground terminal figure of merit (G/T)
    double bandwidth_mhz    = 50.0;  // channel bandwidth
    double other_losses_db  = 3.0;   // atmospheric + implementation margin
};

// Free-space path loss [dB] for a slant range [m] at the given frequency [GHz].
// FSPL(dB) = 20*log10(d) + 20*log10(f) - 147.55, with d in meters, f in Hz.
inline double freeSpacePathLossDb(double range_m, double frequency_ghz) {
    const double freq_hz = frequency_ghz * 1e9;
    return 20.0 * std::log10(range_m) + 20.0 * std::log10(freq_hz) - 147.55;
}

// Achievable Shannon-capacity data rate [Mbps] at the given slant range [m].
inline double dataRateMbps(double range_m, const Config& cfg) {
    if (range_m <= 0.0) return 0.0;

    const double fspl_db = freeSpacePathLossDb(range_m, cfg.frequency_ghz);
    // -10*log10(Boltzmann constant) = 228.6 dBW/K/Hz
    const double neg10log10_k = 228.6;
    const double c_n0_dbhz = cfg.eirp_dbw - fspl_db - cfg.other_losses_db + cfg.gt_dbk + neg10log10_k;

    const double bandwidth_hz = cfg.bandwidth_mhz * 1e6;
    const double c_n0_linear = std::pow(10.0, c_n0_dbhz / 10.0);
    const double snr = c_n0_linear / bandwidth_hz;
    const double capacity_bps = bandwidth_hz * std::log2(1.0 + snr);
    return capacity_bps / 1e6;
}

} // namespace LinkBudget
