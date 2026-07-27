#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "orbit/TLE.h"
#include "orbit/SGP4.h"
#include "orbit/Propagator.h"
#include "physics/Gravity.h"
#include "physics/J2Perturbation.h"
#include "core/math/Constants.h"
#include <memory>
#include <cmath>

using Catch::Matchers::WithinAbs;

namespace {
// Classic Spacetrack Report #3 / Vallado verification vector (satellite 88888).
// This exact TLE is the standard cross-implementation SGP4 regression case.
const std::string kLine1 = "1 88888U          80275.98708465  .00073094  13844-3  66816-4 0    8";
const std::string kLine2 = "2 88888  72.8435 115.9689 0086731  52.6988 110.5714 16.05824518  105";
}

TEST_CASE("TLE parses the standard verification vector", "[tle]") {
    const TLE tle = TLE::parse(kLine1, kLine2);

    CHECK(tle.satellite_number == 88888);
    CHECK_THAT(tle.inclination_rad * Constants::RAD2DEG, WithinAbs(72.8435, 1e-3));
    CHECK_THAT(tle.raan_rad * Constants::RAD2DEG, WithinAbs(115.9689, 1e-3));
    CHECK_THAT(tle.eccentricity, WithinAbs(0.0086731, 1e-7));
    CHECK_THAT(tle.arg_perigee_rad * Constants::RAD2DEG, WithinAbs(52.6988, 1e-3));
    CHECK_THAT(tle.mean_anomaly_rad * Constants::RAD2DEG, WithinAbs(110.5714, 1e-3));
    CHECK_THAT(tle.bstar, WithinAbs(0.66816e-4, 1e-9));
}

TEST_CASE("SGP4 matches the published Spacetrack Report #3 reference vector at epoch", "[sgp4]") {
    // Reference position/velocity from Vallado, Crawford, Hujsak & Kelso,
    // "Revisiting Spacetrack Report #3" (AIAA 2006-6753), the standard
    // cross-implementation SGP4 verification case for satellite 88888.
    // Tolerance is set to a few km / mm-s^-1, not bit-exact, since this is
    // an independent reimplementation rather than a port of the reference
    // source.
    const TLE tle = TLE::parse(kLine1, kLine2);
    const SGP4 sgp4(tle);

    const auto sv = sgp4.propagate(0.0);
    CHECK_THAT(sv.position_km.x, WithinAbs(2328.97048951, 10.0));
    CHECK_THAT(sv.position_km.y, WithinAbs(-5995.22076416, 10.0));
    CHECK_THAT(sv.position_km.z, WithinAbs(1719.97067261, 10.0));
    CHECK_THAT(sv.velocity_km_s.x, WithinAbs(2.91207230, 0.01));
    CHECK_THAT(sv.velocity_km_s.y, WithinAbs(-0.98341546, 0.01));
    CHECK_THAT(sv.velocity_km_s.z, WithinAbs(-7.09081703, 0.01));
}

TEST_CASE("SGP4 propagates a stable, bounded orbit for the verification vector", "[sgp4]") {
    const TLE tle = TLE::parse(kLine1, kLine2);
    const SGP4 sgp4(tle);

    // Sanity bounds rather than a bit-exact reference: the propagated radius
    // should stay near the TLE's implied semi-major axis for a low-eccentricity
    // orbit, at epoch and after half a day.
    const double mu = 398600.8; // km^3/s^2, WGS72 (matches SGP4's internal constants)
    const double n_rad_s = tle.mean_motion_rad_min / 60.0;
    const double sma_km = std::cbrt(mu / (n_rad_s * n_rad_s)) ;

    for (double t : {0.0, 90.0, 360.0, 720.0, 1440.0}) {
        const auto sv = sgp4.propagate(t);
        const double r = sv.position_km.norm();
        // Within ~10% of the mean SMA -- loose bound, just catches gross errors
        // (wrong units, blown-up secular terms, NaNs).
        CHECK(std::isfinite(r));
        CHECK(r > sma_km * 0.8);
        CHECK(r < sma_km * 1.2);

        const double v = sv.velocity_km_s.norm();
        CHECK(std::isfinite(v));
        CHECK(v > 5.0);   // km/s -- reasonable for a ~700 km LEO-ish orbit
        CHECK(v < 9.0);
    }
}

TEST_CASE("SGP4 and this project's RK4+J2 propagator agree over a short span", "[sgp4]") {
    // Use a near-circular, low-drag TLE so the two propagators (which model
    // drag very differently) should stay close over a short window purely
    // from shared two-body + J2 physics.
    const std::string line1 = "1 25544U 98067A   24010.50000000  .00016717  00000-0  10270-3 0  9000";
    const std::string line2 = "2 25544  51.6416 339.9340 0007033 168.0995 192.0562 15.49560971 20000";

    const TLE tle = TLE::parse(line1, line2);
    const SGP4 sgp4(tle);

    // Seed our own propagator from SGP4's osculating state at epoch (treating
    // TEME as ECI, consistent with this project's simplified frame handling).
    const OrbitState seed = sgp4.toOrbitState(0.0);
    OrbitState state = seed;

    PhysicalProperties props;

    Propagator rk4;
    rk4.addForceModel(std::make_unique<Gravity>(Constants::GM_EARTH));
    rk4.addForceModel(std::make_unique<J2Perturbation>(Constants::GM_EARTH, Constants::J2, Constants::EARTH_RADIUS_M));

    const double dt = 30.0;
    const double span_s = 3600.0; // 1 hour -- short enough that drag divergence stays small
    const int steps = static_cast<int>(span_s / dt);
    for (int i = 0; i < steps; ++i) rk4.step(state, props, dt);

    const auto sgp4_end = sgp4.propagate(span_s / 60.0);
    const Vec3 sgp4_pos_m = sgp4_end.position_km * 1000.0;

    const double sep_km = (state.position - sgp4_pos_m).norm() / 1000.0;
    // A few km of separation over an hour is expected (different J2 formulation
    // details, drag term present in SGP4 but not in this RK4 config, TEME vs
    // simplified-ECI frame drift) -- this checks the two propagators are
    // describing the *same orbit*, not bit-identical trajectories.
    CHECK(sep_km < 50.0);
}
