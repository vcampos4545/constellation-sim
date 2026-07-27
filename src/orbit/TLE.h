#pragma once
#include <string>

// A parsed NORAD two-line element set (TLE).
//
// Angles are stored in radians, mean motion in rad/min (SGP4's native units),
// matching the convention used by the reference SGP4 implementation so the
// values here can be fed directly into SGP4::propagate() with no conversion.
struct TLE {
    std::string name;              // optional line-0 name ("" if not present)
    int    satellite_number{0};
    double epoch_jd{0.0};          // Julian Date (UTC) of the TLE epoch

    double inclination_rad{0.0};
    double raan_rad{0.0};
    double eccentricity{0.0};
    double arg_perigee_rad{0.0};
    double mean_anomaly_rad{0.0};
    double mean_motion_rad_min{0.0};   // Kozai mean motion, rad/min

    double ndot_over_2{0.0};       // first derivative of mean motion / 2 [rad/min^2]
    double nddot_over_6{0.0};      // second derivative of mean motion / 6 [rad/min^3]
    double bstar{0.0};             // drag term [1/Earth radii]

    int    element_set_number{0};
    int    rev_number_at_epoch{0};

    // Parse a two-line (or three-line, with a name line) element set.
    // Throws std::runtime_error on malformed input.
    static TLE parse(const std::string& line1, const std::string& line2,
                     const std::string& name = "");

    // Convenience: parse from a block of text containing one TLE
    // (optionally preceded by a name line). Splits on newlines.
    static TLE parseBlock(const std::string& text);

    // Orbital period implied by the mean motion [s].
    double period_s() const;
};
