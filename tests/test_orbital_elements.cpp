#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("Elements <-> state vector round trip", "[orbital_elements]") {
    const double mu = Constants::GM_EARTH;

    OrbitalElements elems;
    elems.sma  = Constants::EARTH_RADIUS_M + 550'000.0;
    elems.ecc  = 0.001;
    elems.inc  = 53.0 * Constants::DEG2RAD;
    elems.raan = 120.0 * Constants::DEG2RAD;
    elems.aop  = 30.0 * Constants::DEG2RAD;
    elems.ta   = 200.0 * Constants::DEG2RAD;

    const OrbitState state = elems.toStateVector(mu);
    const OrbitalElements back = OrbitalElements::fromStateVector(state, mu);

    CHECK_THAT(back.sma,  WithinRel(elems.sma,  1e-9));
    CHECK_THAT(back.ecc,  WithinAbs(elems.ecc,  1e-9));
    CHECK_THAT(back.inc,  WithinAbs(elems.inc,  1e-9));
    CHECK_THAT(back.raan, WithinAbs(elems.raan, 1e-9));
    CHECK_THAT(back.aop,  WithinAbs(elems.aop,  1e-9));
    CHECK_THAT(back.ta,   WithinAbs(elems.ta,   1e-9));
}

TEST_CASE("Circular orbit has constant radius", "[orbital_elements]") {
    const double mu = Constants::GM_EARTH;
    OrbitalElements elems;
    elems.sma = Constants::EARTH_RADIUS_M + 700'000.0;
    elems.ecc = 0.0;
    elems.inc = 98.0 * Constants::DEG2RAD;

    for (double ta_deg = 0.0; ta_deg < 360.0; ta_deg += 30.0) {
        elems.ta = ta_deg * Constants::DEG2RAD;
        const OrbitState s = elems.toStateVector(mu);
        CHECK_THAT(s.position.norm(), WithinRel(elems.sma, 1e-9));
    }
}

TEST_CASE("Kepler solver converges for a range of eccentricities", "[orbital_elements]") {
    for (double e = 0.0; e < 0.9; e += 0.1) {
        for (double M_deg = 0.0; M_deg < 360.0; M_deg += 45.0) {
            const double M = M_deg * Constants::DEG2RAD;
            const double E = OrbitalElements::solveKepler(M, e);
            // Kepler's equation: M = E - e*sin(E)
            const double M_check = E - e * std::sin(E);
            double diff = std::fmod(M_check - M + Constants::PI, Constants::TWO_PI) - Constants::PI;
            CHECK_THAT(diff, WithinAbs(0.0, 1e-10));
        }
    }
}

TEST_CASE("Period matches Kepler's third law", "[orbital_elements]") {
    const double mu = Constants::GM_EARTH;
    OrbitalElements elems;
    elems.sma = Constants::EARTH_RADIUS_M + 550'000.0;
    const double period = elems.period_s(mu);
    // LEO at 550 km should be roughly 95-96 minutes
    CHECK(period > 94.0 * 60.0);
    CHECK(period < 97.0 * 60.0);
}
