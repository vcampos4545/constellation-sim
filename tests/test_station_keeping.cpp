#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "orbit/StationKeeping.h"
#include "orbit/OrbitalElements.h"
#include "orbit/Propagator.h"
#include "physics/Gravity.h"
#include "physics/AtmosphericDrag.h"
#include "core/math/Constants.h"
#include <memory>

using Catch::Matchers::WithinRel;

TEST_CASE("StationKeeping does nothing within the deadband", "[station_keeping]") {
    const double mu = Constants::GM_EARTH;
    OrbitalElements elems;
    elems.sma = Constants::EARTH_RADIUS_M + 550'000.0;
    OrbitState state = elems.toStateVector(mu);

    const double dv = StationKeeping::maybeReboost(state, elems.sma, 5000.0, mu);
    CHECK(dv == 0.0);
}

TEST_CASE("StationKeeping restores a decayed orbit to nominal SMA", "[station_keeping]") {
    const double mu = Constants::GM_EARTH;
    const double nominal_sma = Constants::EARTH_RADIUS_M + 550'000.0;

    // Simulate a decayed orbit: same position, but slower (lower-energy) velocity.
    OrbitalElements decayed;
    decayed.sma = nominal_sma - 10'000.0; // decayed 10 km
    OrbitState state = decayed.toStateVector(mu);

    const double dv = StationKeeping::maybeReboost(state, nominal_sma, 5000.0, mu);
    CHECK(dv > 0.0);

    const OrbitalElements restored = OrbitalElements::fromStateVector(state, mu);
    CHECK_THAT(restored.sma, WithinRel(nominal_sma, 1e-6));
}

TEST_CASE("Active station-keeping holds altitude against drag decay over many orbits", "[station_keeping]") {
    const double mu = Constants::GM_EARTH;
    const double re = Constants::EARTH_RADIUS_M;
    const double nominal_sma = re + 350'000.0; // low enough for drag to matter on this timescale

    OrbitalElements elems;
    elems.sma = nominal_sma;
    OrbitState state = elems.toStateVector(mu);

    PhysicalProperties props;
    props.mass_kg = 50.0;
    props.drag_coefficient = 2.2;
    props.drag_area_m2 = 10.0; // deliberately high beta to force visible decay in a short test

    Propagator prop;
    prop.addForceModel(std::make_unique<Gravity>(mu));
    prop.addForceModel(std::make_unique<AtmosphericDrag>(re));

    const double dt = 30.0;
    const double deadband_m = 1000.0;
    const double check_interval_s = 1800.0;
    double next_check = check_interval_s;
    double total_dv = 0.0;
    int maneuvers = 0;

    const double duration_s = 3.0 * 86400.0; // 3 days
    for (double t = 0.0; t < duration_s; t += dt) {
        prop.step(state, props, dt);
        if (t + dt >= next_check) {
            const double dv = StationKeeping::maybeReboost(state, nominal_sma, deadband_m, mu);
            if (dv > 0.0) { total_dv += dv; ++maneuvers; }
            next_check += check_interval_s;
        }
    }

    const OrbitalElements final_elems = OrbitalElements::fromStateVector(state, mu);
    // With active station-keeping, SMA should never have drifted more than
    // one deadband below nominal (modulo the drag accrued within one check
    // interval past the deadband).
    CHECK(nominal_sma - final_elems.sma < deadband_m * 5.0);
    CHECK(maneuvers > 0);
    CHECK(total_dv > 0.0);
}
