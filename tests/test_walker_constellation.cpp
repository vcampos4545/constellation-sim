#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "constellation/Constellation.h"
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"

using Catch::Matchers::WithinAbs;

TEST_CASE("Walker constellation generates the requested satellite/plane counts", "[constellation]") {
    WalkerConfig cfg;
    cfg.altitude_km      = 550.0;
    cfg.inclination_deg  = 53.0;
    cfg.total_satellites = 1584;
    cfg.planes           = 72;
    cfg.phasing_factor   = 13;

    const Constellation c = Constellation::createWalker(cfg);

    CHECK(c.totalSatellites() == 1584);
    CHECK(c.planes().size() == 72);
    for (const auto& plane : c.planes()) {
        CHECK(plane.satellites().size() == 1584 / 72);
    }
}

TEST_CASE("Walker constellation satellites all share the same altitude and inclination", "[constellation]") {
    WalkerConfig cfg;
    cfg.altitude_km      = 500.0;
    cfg.inclination_deg  = 70.0;
    cfg.total_satellites = 48;
    cfg.planes           = 6;
    cfg.phasing_factor   = 1;

    const Constellation c = Constellation::createWalker(cfg);
    const double mu = Constants::GM_EARTH;
    const double expected_sma = Constants::EARTH_RADIUS_M + cfg.altitude_km * 1000.0;

    for (const auto* sat : c.satellites()) {
        const OrbitalElements elems = OrbitalElements::fromStateVector(sat->state(), mu);
        CHECK_THAT(elems.sma, WithinAbs(expected_sma, 1.0));
        CHECK_THAT(elems.inc * Constants::RAD2DEG, WithinAbs(cfg.inclination_deg, 1e-6));
    }
}

TEST_CASE("Walker planes are evenly spaced in RAAN", "[constellation]") {
    WalkerConfig cfg;
    cfg.altitude_km      = 550.0;
    cfg.inclination_deg  = 53.0;
    cfg.total_satellites = 40;
    cfg.planes           = 4;
    cfg.phasing_factor   = 1;

    const Constellation c = Constellation::createWalker(cfg);
    REQUIRE(c.planes().size() == 4);

    const double expected_step_rad = Constants::TWO_PI / cfg.planes;
    for (size_t i = 0; i < c.planes().size(); ++i) {
        const double expected_raan = i * expected_step_rad;
        CHECK_THAT(c.planes()[i].raanRad(), WithinAbs(expected_raan, 1e-9));
    }
}
