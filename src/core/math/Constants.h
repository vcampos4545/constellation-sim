#pragma once

#include <numbers>

namespace Constants {

// Mathematical
inline constexpr double PI      = std::numbers::pi;
inline constexpr double TWO_PI  = 2.0 * PI;
inline constexpr double DEG2RAD = PI / 180.0;
inline constexpr double RAD2DEG = 180.0 / PI;

// WGS84 Earth constants
inline constexpr double EARTH_RADIUS_M      = 6'378'137.0;          // [m]
inline constexpr double EARTH_RADIUS_KM     = 6378.137;             // [km]
inline constexpr double GM_EARTH            = 3.986004418e14;        // [m^3/s^2]
inline constexpr double J2                  =  1.08262668e-3;        // EGM2008 [-]
inline constexpr double J3                  = -2.53265649e-6;        // EGM2008 [-]
inline constexpr double J4                  = -1.61962160e-6;        // EGM2008 [-]
inline constexpr double EARTH_OMEGA_RAD_S   = 7.2921150e-5;          // [rad/s]
inline constexpr double EARTH_FLATTENING    = 1.0 / 298.257223563;   // [-]

// Sun
inline constexpr double AU_M                = 1.495978707e11;        // [m]
inline constexpr double GM_SUN              = 1.32712440018e20;      // [m^3/s^2]
inline constexpr double SOLAR_PRESSURE_PA   = 4.56e-6;              // [N/m^2] at 1 AU
inline constexpr double SOLAR_CONSTANT_WM2  = 1361.0;               // [W/m^2] at 1 AU (TSI)
inline constexpr double SUN_RADIUS_M        = 6.957e8;              // [m]

// Moon
inline constexpr double GM_MOON             = 4.9048695e12;          // [m^3/s^2]
inline constexpr double MOON_RADIUS_M       = 1.7374e6;             // [m]

// Time
inline constexpr double J2000_JD            = 2451545.0;             // J2000 Julian Date
inline constexpr double SEC_PER_DAY         = 86400.0;
inline constexpr double SEC_PER_YEAR        = 365.25 * SEC_PER_DAY;

// Atmosphere (exponential model reference values at sea level)
inline constexpr double ATMO_RHO_SL_KG_M3  = 1.225;                 // [kg/m^3]
inline constexpr double ATMO_SCALE_H_M      = 8500.0;               // [m] scale height at low alt

} // namespace Constants
