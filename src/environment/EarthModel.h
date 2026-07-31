#pragma once
#include "core/math/Constants.h"
#include "core/math/Vec3.h"
#include <algorithm>

// Utility functions for Earth geometry and coordinate transformations.
// Uses a spherical Earth with WGS84 constants for radius and rotation rate.
namespace EarthModel {

    // Convert geodetic latitude/longitude (degrees) + altitude (m) to ECEF [m].
    inline Vec3 geodeticToECEF(double lat_deg, double lon_deg, double alt_m = 0.0) {
        const double lat = lat_deg * Constants::DEG2RAD;
        const double lon = lon_deg * Constants::DEG2RAD;
        const double r   = Constants::EARTH_RADIUS_M + alt_m;
        return {
            r * std::cos(lat) * std::cos(lon),
            r * std::cos(lat) * std::sin(lon),
            r * std::sin(lat)
        };
    }

    // Rotate an ECEF vector to ECI by applying the Greenwich Sidereal Angle.
    // theta_gst is the Earth rotation angle [rad] at the current epoch.
    inline Vec3 ecefToECI(const Vec3& ecef, double theta_gst) {
        const double c = std::cos(theta_gst);
        const double s = std::sin(theta_gst);
        return {
             c*ecef.x - s*ecef.y,
             s*ecef.x + c*ecef.y,
             ecef.z
        };
    }

    // Greenwich Mean Sidereal Time [rad] for Julian Date jd.
    // IAU 1982 formula (Vallado, Alg 15). Accurate to ~0.1 arcsec.
    inline double gmst_rad(double jd) {
        const double T = (jd - Constants::J2000_JD) / 36525.0;
        double theta = 280.46061837
                     + 360.98564736629 * (jd - Constants::J2000_JD)
                     + 0.000387933     * T * T
                     - T * T * T / 38710000.0;
        theta = std::fmod(theta, 360.0);
        if (theta < 0.0) theta += 360.0;
        return theta * Constants::DEG2RAD;
    }

    // Earth rotation angle [rad] at t_s seconds after epoch_jd.
    inline double gstAtTime(double t_s, double epoch_jd = Constants::J2000_JD) {
        return gmst_rad(epoch_jd + t_s / Constants::SEC_PER_DAY);
    }

    // Atmospheric co-rotation velocity at position r [ECI, m]: v = omega x r
    inline Vec3 atmosphericVelocity(const Vec3& r) {
        return { -Constants::EARTH_OMEGA_RAD_S * r.y,
                  Constants::EARTH_OMEGA_RAD_S * r.x,
                  0.0 };
    }

    // Elevation angle [rad] of a satellite at r_sat_eci from a ground point at r_gnd_eci.
    // Positive = above horizon.
    inline double elevationAngle(const Vec3& r_gnd_eci, const Vec3& r_sat_eci) {
        const Vec3 slant = r_sat_eci - r_gnd_eci;
        const Vec3 gnd_hat = r_gnd_eci.normalized();
        return std::asin(std::clamp(gnd_hat.dot(slant.normalized()), -1.0, 1.0));
    }

} // namespace EarthModel
