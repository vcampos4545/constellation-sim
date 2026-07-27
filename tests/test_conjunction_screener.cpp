#include <catch2/catch_test_macros.hpp>
#include "metrics/ConjunctionScreener.h"
#include "orbit/OrbitalElements.h"
#include "orbit/Satellite.h"
#include "core/math/Constants.h"

namespace {
Satellite makeSat(int id, double sma_m, double ecc, double inc_deg,
                  double raan_deg, double aop_deg, double ta_deg) {
    OrbitalElements e;
    e.sma  = sma_m;
    e.ecc  = ecc;
    e.inc  = inc_deg  * Constants::DEG2RAD;
    e.raan = raan_deg * Constants::DEG2RAD;
    e.aop  = aop_deg  * Constants::DEG2RAD;
    e.ta   = ta_deg   * Constants::DEG2RAD;
    const OrbitState state = e.toStateVector(Constants::GM_EARTH);
    PhysicalProperties props;
    return Satellite(id, 0, id, state, props);
}
}

TEST_CASE("Conjunction screener flags two closely-spaced co-orbital satellites", "[conjunction]") {
    const double sma = Constants::EARTH_RADIUS_M + 550'000.0;
    std::vector<Satellite> sats;
    sats.push_back(makeSat(0, sma, 0.0, 53.0, 0.0, 0.0, 0.0));
    sats.push_back(makeSat(1, sma, 0.0, 53.0, 0.0, 0.0, 0.01)); // ~1.2 km in-track separation

    std::vector<Satellite*> ptrs{&sats[0], &sats[1]};

    ConjunctionConfig cfg;
    cfg.enabled = true;
    cfg.threshold_km = 5.0;
    cfg.sample_interval_s = 60.0;

    ConjunctionScreener screener(cfg, ptrs);
    CHECK(screener.candidatePairCount() == 1);

    screener.update(ptrs, 0.0);
    const auto events = screener.finalizeEvents();

    REQUIRE(events.size() == 1);
    CHECK(events[0].sat_i == 0);
    CHECK(events[0].sat_j == 1);
    CHECK(events[0].min_distance_km < 5.0);
    CHECK(events[0].min_distance_km > 0.5); // sanity: not degenerate/zero
}

TEST_CASE("Conjunction screener does not flag satellites on opposite sides of the same orbit", "[conjunction]") {
    const double sma = Constants::EARTH_RADIUS_M + 550'000.0;
    std::vector<Satellite> sats;
    sats.push_back(makeSat(0, sma, 0.0, 53.0, 0.0, 0.0, 0.0));
    sats.push_back(makeSat(1, sma, 0.0, 53.0, 0.0, 0.0, 180.0));

    std::vector<Satellite*> ptrs{&sats[0], &sats[1]};

    ConjunctionConfig cfg;
    cfg.enabled = true;
    cfg.threshold_km = 5.0;

    ConjunctionScreener screener(cfg, ptrs);
    screener.update(ptrs, 0.0);
    const auto events = screener.finalizeEvents();

    CHECK(events.empty());
}

TEST_CASE("Conjunction screener pre-filter discards pairs in non-overlapping altitude shells", "[conjunction]") {
    std::vector<Satellite> sats;
    sats.push_back(makeSat(0, Constants::EARTH_RADIUS_M + 550'000.0, 0.0, 53.0, 0.0, 0.0, 0.0));
    sats.push_back(makeSat(1, Constants::EARTH_RADIUS_M + 1500'000.0, 0.0, 53.0, 0.0, 0.0, 0.0));

    std::vector<Satellite*> ptrs{&sats[0], &sats[1]};

    ConjunctionConfig cfg;
    cfg.enabled = true;
    cfg.threshold_km = 5.0;

    ConjunctionScreener screener(cfg, ptrs);
    CHECK(screener.candidatePairCount() == 0);
}
