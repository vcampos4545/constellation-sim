#include "environment/EclipseModel.h"
#include "core/math/Constants.h"
#include <algorithm>
#include <cmath>

bool EclipseModel::inEclipse(const Vec3& sat_pos_eci, const Vec3& sun_dir_eci) {
    // Project satellite position onto Sun direction.
    // If the satellite is on the night side and the perpendicular distance
    // from the Sun-Earth line is less than Earth's radius, it is in shadow.
    const double proj = sat_pos_eci.dot(sun_dir_eci);
    if (proj > 0.0) return false;  // satellite on Sun side of Earth

    const Vec3 perp = sat_pos_eci - proj * sun_dir_eci;
    return perp.norm() < Constants::EARTH_RADIUS_M;
}

EclipseModel::ShadowState EclipseModel::shadowState(const Vec3& sat_pos,
                                                     const Vec3& sun_pos,
                                                     double Re,
                                                     double Rs) {
    // Based on Vallado "Fundamentals of Astrodynamics", Section 5.3
    // Compute angles for conical shadow model
    const double r_sat = sat_pos.norm();
    const double r_sun = sun_pos.norm();

    // Apparent angular radii as seen from satellite
    const double theta_earth = std::asin(std::clamp(Re / r_sat, 0.0, 1.0));
    const double theta_sun   = std::asin(std::clamp(Rs / r_sun, 0.0, 1.0));

    // Angle between Sun-Earth and satellite vectors
    const Vec3  sat_to_sun = sun_pos - sat_pos;
    const double cos_angle = sat_pos.normalized().dot((-sat_to_sun).normalized());
    const double theta_sep = std::acos(std::clamp(cos_angle, -1.0, 1.0));

    if (theta_sep >= theta_earth + theta_sun) return ShadowState::Sunlit;
    if (theta_sep <= std::abs(theta_earth - theta_sun)) {
        if (theta_earth > theta_sun) return ShadowState::Umbra;
        return ShadowState::Penumbra;  // annular eclipse (very rare)
    }
    return ShadowState::Penumbra;
}
