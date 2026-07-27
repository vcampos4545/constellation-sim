// Generates a CSV comparing this project's RK4+J2 propagator against the
// project's own SGP4 implementation, both seeded from the same TLE. Used
// by analysis/sgp4_validation.ipynb to plot propagator agreement over time.
//
// This is a cross-validation between two propagators built for different
// purposes (RK4: configurable-fidelity numerical integration for trade
// studies; SGP4: the NORAD analytic standard for TLE-based propagation),
// not a substitute for validating either one against an external reference
// implementation -- see src/orbit/SGP4.h for that discussion and the
// Spacetrack Report #3 test-vector check in tests/test_sgp4.cpp.
#include "orbit/TLE.h"
#include "orbit/SGP4.h"
#include "orbit/Propagator.h"
#include "physics/Gravity.h"
#include "physics/J2Perturbation.h"
#include "core/math/Constants.h"
#include "output/CsvWriter.h"
#include <filesystem>
#include <iostream>
#include <memory>

int main() {
    // ISS-like TLE (same one used in tests/test_sgp4.cpp's cross-check test).
    const std::string line1 = "1 25544U 98067A   24010.50000000  .00016717  00000-0  10270-3 0  9000";
    const std::string line2 = "2 25544  51.6416 339.9340 0007033 168.0995 192.0562 15.49560971 20000";

    const TLE tle = TLE::parse(line1, line2);
    const SGP4 sgp4(tle);

    OrbitState state = sgp4.toOrbitState(0.0);
    PhysicalProperties props;

    Propagator rk4;
    rk4.addForceModel(std::make_unique<Gravity>(Constants::GM_EARTH));
    rk4.addForceModel(std::make_unique<J2Perturbation>(Constants::GM_EARTH, Constants::J2, Constants::EARTH_RADIUS_M));

    const double dt = 30.0;              // RK4 integration step [s]
    const double output_interval_s = 600.0; // sample every 10 minutes
    const double duration_s = 3.0 * 86400.0; // 3 days

    const std::string out_dir = "output/sgp4_validation";
    std::filesystem::create_directories(out_dir);
    CsvWriter w(out_dir + "/comparison.csv");
    w.writeHeader({
        "time_hours",
        "rk4_x_km", "rk4_y_km", "rk4_z_km",
        "sgp4_x_km", "sgp4_y_km", "sgp4_z_km",
        "separation_km",
        "rk4_alt_km", "sgp4_alt_km"
    });

    double next_output = 0.0;
    for (double t = 0.0; t <= duration_s; t += dt) {
        if (t >= next_output) {
            const auto sgp4_sv = sgp4.propagate(t / 60.0);
            const Vec3 sgp4_pos_m = sgp4_sv.position_km * 1000.0;
            const double sep_km = (state.position - sgp4_pos_m).norm() / 1000.0;

            const double rk4_alt_km  = state.position.norm() / 1000.0 - Constants::EARTH_RADIUS_KM;
            const double sgp4_alt_km = sgp4_sv.position_km.norm() - Constants::EARTH_RADIUS_KM;

            w.writeRowV(t / 3600.0,
                       state.position.x / 1000.0, state.position.y / 1000.0, state.position.z / 1000.0,
                       sgp4_sv.position_km.x, sgp4_sv.position_km.y, sgp4_sv.position_km.z,
                       sep_km, rk4_alt_km, sgp4_alt_km);

            next_output += output_interval_s;
        }
        rk4.step(state, props, dt);
    }

    std::cout << "Wrote " << out_dir << "/comparison.csv\n";
    return 0;
}
