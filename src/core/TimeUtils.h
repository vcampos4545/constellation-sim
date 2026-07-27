#pragma once
#include <cmath>
#include <string>
#include <cstdio>

// Julian Date <-> calendar conversions shared by TLE parsing, config epoch
// strings, and CCSDS ephemeris export.
namespace TimeUtils {

// Julian Date -> ISO 8601 UTC timestamp ("YYYY-MM-DDTHH:MM:SS.ffffff").
// Standard Meeus (Ch. 7) JD -> calendar algorithm.
inline std::string jdToISO8601(double jd) {
    const double jd_shifted = jd + 0.5;
    double Z = std::floor(jd_shifted);
    const double F = jd_shifted - Z;

    double A = Z;
    if (Z >= 2299161.0) {
        const double alpha = std::floor((Z - 1867216.25) / 36524.25);
        A = Z + 1.0 + alpha - std::floor(alpha / 4.0);
    }
    const double B = A + 1524.0;
    const double C = std::floor((B - 122.1) / 365.25);
    const double D = std::floor(365.25 * C);
    const double E = std::floor((B - D) / 30.6001);

    const double day_frac = B - D - std::floor(30.6001 * E) + F;
    const int day = static_cast<int>(std::floor(day_frac));
    const int month = (E < 14) ? static_cast<int>(E - 1) : static_cast<int>(E - 13);
    const int year = (month > 2) ? static_cast<int>(C - 4716) : static_cast<int>(C - 4715);

    double hours_frac = (day_frac - day) * 24.0;
    const int hour = static_cast<int>(hours_frac);
    double minutes_frac = (hours_frac - hour) * 60.0;
    const int minute = static_cast<int>(minutes_frac);
    const double second = (minutes_frac - minute) * 60.0;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%09.6f",
                  year, month, day, hour, minute, second);
    return std::string(buf);
}

} // namespace TimeUtils
