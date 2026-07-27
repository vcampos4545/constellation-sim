#include <catch2/catch_test_macros.hpp>
#include "montecarlo/ParetoFrontier.h"
#include <algorithm>

namespace {
ConstellationResult makeResult(int sats, double coverage_pct, double sk_dv) {
    ConstellationResult cr;
    cr.total_satellites = sats;
    cr.coverage_pct = coverage_pct;
    cr.avg_annual_sk_dv_ms_per_year = sk_dv;
    return cr;
}
}

TEST_CASE("ParetoFrontier parses objective specs", "[pareto]") {
    const auto objs = ParetoFrontier::parseObjectives({
        "minimize:total_satellites", "maximize:coverage_pct"
    });
    REQUIRE(objs.size() == 2);
    CHECK(objs[0].field == "total_satellites");
    CHECK(objs[0].maximize == false);
    CHECK(objs[1].field == "coverage_pct");
    CHECK(objs[1].maximize == true);
}

TEST_CASE("ParetoFrontier rejects malformed or unknown specs", "[pareto]") {
    CHECK_THROWS(ParetoFrontier::parseObjectives({"bogus_no_colon"}));
    CHECK_THROWS(ParetoFrontier::parseObjectives({"sideways:coverage_pct"}));
    CHECK_THROWS(ParetoFrontier::parseObjectives({"maximize:not_a_real_field"}));
}

TEST_CASE("ParetoFrontier excludes a strictly-dominated result", "[pareto]") {
    // B has more satellites AND lower coverage than A -- strictly worse on
    // both a "minimize satellites" and "maximize coverage" objective.
    std::vector<ConstellationResult> results = {
        makeResult(/*sats=*/24, /*coverage=*/90.0, /*sk_dv=*/100.0),  // A
        makeResult(/*sats=*/48, /*coverage=*/85.0, /*sk_dv=*/100.0),  // B: dominated by A
        makeResult(/*sats=*/72, /*coverage=*/98.0, /*sk_dv=*/100.0),  // C: more sats but more coverage than A
    };

    const auto objectives = ParetoFrontier::parseObjectives({
        "minimize:total_satellites", "maximize:coverage_pct"
    });
    const auto frontier = ParetoFrontier::nonDominated(results, objectives);

    CHECK(std::find(frontier.begin(), frontier.end(), 0) != frontier.end()); // A survives
    CHECK(std::find(frontier.begin(), frontier.end(), 1) == frontier.end()); // B is dominated
    CHECK(std::find(frontier.begin(), frontier.end(), 2) != frontier.end()); // C survives (trade-off)
}

TEST_CASE("ParetoFrontier keeps all results when none dominates another", "[pareto]") {
    std::vector<ConstellationResult> results = {
        makeResult(24, 80.0, 50.0),
        makeResult(48, 90.0, 100.0),
        makeResult(72, 95.0, 200.0),
    };
    const auto objectives = ParetoFrontier::parseObjectives({
        "minimize:total_satellites", "maximize:coverage_pct", "minimize:avg_annual_sk_dv_ms_per_year"
    });
    const auto frontier = ParetoFrontier::nonDominated(results, objectives);
    CHECK(frontier.size() == 3);
}
