#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "orbit/Propagator.h"
#include "orbit/OrbitalElements.h"
#include "physics/Gravity.h"
#include "physics/J2Perturbation.h"
#include "core/math/Constants.h"
#include <cmath>
#include <memory>

using Catch::Matchers::WithinRel;

namespace {
PhysicalProperties makeProps() {
    PhysicalProperties p;
    p.mass_kg = 260.0;
    return p;
}
}

TEST_CASE("Two-body RK4 conserves specific energy and angular momentum", "[propagator]") {
    const double mu = Constants::GM_EARTH;

    OrbitalElements elems;
    elems.sma = Constants::EARTH_RADIUS_M + 550'000.0;
    elems.ecc = 0.01;
    elems.inc = 53.0 * Constants::DEG2RAD;
    elems.raan = 10.0 * Constants::DEG2RAD;
    elems.aop = 15.0 * Constants::DEG2RAD;
    elems.ta = 0.0;

    OrbitState state = elems.toStateVector(mu);
    const auto props = makeProps();

    Propagator prop;
    prop.addForceModel(std::make_unique<Gravity>(mu));

    const double r0 = state.position.norm();
    const double v0 = state.velocity.norm();
    const double energy0 = 0.5 * v0 * v0 - mu / r0;
    const double h0 = state.position.cross(state.velocity).norm();

    const double dt = 10.0;
    const double period = elems.period_s(mu);
    const int steps = static_cast<int>(period / dt);

    for (int i = 0; i < steps; ++i) {
        prop.step(state, props, dt);
    }

    const double r1 = state.position.norm();
    const double v1 = state.velocity.norm();
    const double energy1 = 0.5 * v1 * v1 - mu / r1;
    const double h1 = state.position.cross(state.velocity).norm();

    // Over one orbit with RK4 at 10s steps, energy and angular momentum
    // should be conserved to a very tight relative tolerance.
    CHECK_THAT(energy1, WithinRel(energy0, 1e-8));
    CHECK_THAT(h1, WithinRel(h0, 1e-8));

    // The satellite should also return close to its starting position.
    CHECK_THAT(r1, WithinRel(r0, 1e-6));
}

TEST_CASE("J2 perturbation produces the expected secular RAAN drift", "[propagator]") {
    const double mu = Constants::GM_EARTH;
    const double re = Constants::EARTH_RADIUS_M;
    const double j2 = Constants::J2;

    OrbitalElements elems;
    elems.sma = Constants::EARTH_RADIUS_M + 550'000.0;
    elems.ecc = 0.0;
    elems.inc = 53.0 * Constants::DEG2RAD;
    elems.raan = 0.0;
    elems.aop = 0.0;
    elems.ta = 0.0;

    OrbitState state = elems.toStateVector(mu);
    const auto props = makeProps();

    Propagator prop;
    prop.addForceModel(std::make_unique<Gravity>(mu));
    prop.addForceModel(std::make_unique<J2Perturbation>(mu, j2, re));

    const double dt = 30.0;
    const double period = elems.period_s(mu);
    const int orbits = 20;
    const int steps = static_cast<int>(period * orbits / dt);

    for (int i = 0; i < steps; ++i) {
        prop.step(state, props, dt);
    }

    const OrbitalElements final_elems = OrbitalElements::fromStateVector(state, mu);

    // Analytic first-order secular RAAN drift rate (Vallado eq. 9-38):
    // dRAAN/dt = -1.5 * n * J2 * (Re/p)^2 * cos(i)
    const double n = elems.meanMotion_rads(mu);
    const double p = elems.sma * (1.0 - elems.ecc * elems.ecc);
    const double raan_dot = -1.5 * n * j2 * (re / p) * (re / p) * std::cos(elems.inc);
    const double expected_raan = std::fmod(raan_dot * steps * dt, Constants::TWO_PI);

    double actual_raan = final_elems.raan;
    // Normalize both to [-pi, pi] for comparison
    auto wrap = [](double a) {
        a = std::fmod(a + Constants::PI, Constants::TWO_PI);
        if (a < 0) a += Constants::TWO_PI;
        return a - Constants::PI;
    };

    // Within 5% of the analytic secular rate over 20 orbits.
    CHECK_THAT(wrap(actual_raan), WithinRel(wrap(expected_raan), 0.05));
}
