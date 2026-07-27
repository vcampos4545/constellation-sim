#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "environment/LinkBudget.h"

using Catch::Matchers::WithinAbs;

TEST_CASE("Free-space path loss matches the standard formula at a known range", "[link_budget]") {
    // FSPL(dB) = 20log10(d) + 20log10(f) - 147.55
    // At d = 1000 km, f = 12 GHz:
    //   20*log10(1e6) + 20*log10(12e9) - 147.55
    // = 120 + 201.58 - 147.55 = 174.03 dB (approx)
    const double fspl = LinkBudget::freeSpacePathLossDb(1'000'000.0, 12.0);
    CHECK_THAT(fspl, WithinAbs(174.03, 0.1));
}

TEST_CASE("Data rate decreases monotonically with range", "[link_budget]") {
    LinkBudget::Config cfg;
    cfg.enabled = true;

    const double r1 = LinkBudget::dataRateMbps(500'000.0, cfg);
    const double r2 = LinkBudget::dataRateMbps(1'000'000.0, cfg);
    const double r3 = LinkBudget::dataRateMbps(2'000'000.0, cfg);

    CHECK(r1 > r2);
    CHECK(r2 > r3);
    CHECK(r3 > 0.0); // still finite/positive at LEO-scale slant ranges
}

TEST_CASE("Data rate is zero for a degenerate zero range", "[link_budget]") {
    LinkBudget::Config cfg;
    CHECK(LinkBudget::dataRateMbps(0.0, cfg) == 0.0);
}

TEST_CASE("Higher EIRP and G/T increase achievable data rate", "[link_budget]") {
    LinkBudget::Config weak;
    weak.eirp_dbw = 30.0;
    weak.gt_dbk = 5.0;

    LinkBudget::Config strong;
    strong.eirp_dbw = 50.0;
    strong.gt_dbk = 20.0;

    const double range_m = 800'000.0;
    CHECK(LinkBudget::dataRateMbps(range_m, strong) > LinkBudget::dataRateMbps(range_m, weak));
}
