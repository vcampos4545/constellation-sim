#pragma once
#include "orbit/TLE.h"
#include "orbit/OrbitState.h"
#include "core/math/Vec3.h"

// SGP4 analytic propagator (near-Earth case only).
//
// This is a from-scratch reimplementation of the public near-Earth SGP4
// algorithm described in Vallado, Crawford, Hujsak & Kelso, "Revisiting
// Spacetrack Report #3" (AIAA 2006-6753), using WGS72 constants -- the
// convention NORAD TLEs are distributed under. It exists in this project
// so real TLE-derived constellations can be imported and so the project's
// own RK4 propagator has an independent analytic cross-check.
//
// Scope / caveats (documented rather than silently assumed):
//   - Near-Earth objects only (period < 225 min). LEO constellations, this
//     project's whole focus, are always near-Earth; deep-space resonance
//     terms (SDP4, for GEO/Molniya-class orbits) are not implemented.
//   - The low-perigee (<220 km) simplified-drag branch ("isimp" in the
//     reference algorithm) is not implemented; accuracy degrades for
//     objects that low, which is outside this project's design altitudes.
//   - Output is in the TEME frame (True Equator, Mean Equinox), SGP4's
//     native frame. This project's ECI frame elsewhere assumes a simpler
//     GMST-only Earth orientation with no precession/nutation, so TEME is
//     treated as an approximation of ECI here -- adequate for the
//     trade-study fidelity level this project targets, not for
//     high-precision cross-frame work.
//
// For flight-critical or high-precision use, cross-check against a vetted
// reference implementation (e.g. the "sgp4" Python package, which wraps
// Vallado's original public-domain C++ source).
class SGP4 {
public:
    explicit SGP4(const TLE& tle);

    struct StateVector {
        Vec3 position_km;
        Vec3 velocity_km_s;
    };

    // Propagate to `minutes_since_epoch` (may be negative). Returns TEME
    // position/velocity in km and km/s -- SGP4's native output units.
    StateVector propagate(double minutes_since_epoch) const;

    // Same propagation, converted to this project's OrbitState (SI units).
    // `time_s` on the returned state is set to minutes_since_epoch * 60.
    OrbitState toOrbitState(double minutes_since_epoch) const;

    double epochJD() const { return epoch_jd_; }

private:
    double epoch_jd_{0.0};

    // Recovered mean elements (post Kozai -> Brouwer correction)
    double no_{0.0}, ao_{0.0};
    double ecco_{0.0}, inclo_{0.0}, nodeo_{0.0}, argpo_{0.0}, mo_{0.0}, bstar_{0.0};

    // Derived geometry constants
    double cosio_{0.0}, sinio_{0.0};
    double x3thm1_{0.0}, x1mth2_{0.0}, x7thm1_{0.0};

    // Secular drag / gravity coefficients
    double c1_{0.0}, c4_{0.0}, c5_{0.0};
    double mdot_{0.0}, argpdot_{0.0}, nodedot_{0.0}, nodecf_{0.0}, t2cof_{0.0};
    double omgcof_{0.0}, xmcof_{0.0}, xlcof_{0.0}, aycof_{0.0};
    double delmo_{0.0}, sinmao_{0.0};
    double eta_{0.0};

    void initialize(const TLE& tle);
};
