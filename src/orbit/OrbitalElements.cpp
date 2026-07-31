#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Kepler's equation solver (Newton-Raphson, typically converges in <10 iters)
// ---------------------------------------------------------------------------
double OrbitalElements::solveKepler(double M, double e) {
    // Normalize M to [0, 2pi)
    M = std::fmod(M, Constants::TWO_PI);
    if (M < 0.0) M += Constants::TWO_PI;

    double E = M;  // initial guess
    for (int i = 0; i < 50; ++i) {
        const double dE = (M - E + e * std::sin(E)) / (1.0 - e * std::cos(E));
        E += dE;
        if (std::abs(dE) < 1e-12) break;
    }
    return E;
}

double OrbitalElements::meanToTrue(double M, double e) {
    const double E = solveKepler(M, e);
    // tan(nu/2) = sqrt((1+e)/(1-e)) * tan(E/2)
    const double half_nu = std::atan2(
        std::sqrt(1.0 + e) * std::sin(E / 2.0),
        std::sqrt(1.0 - e) * std::cos(E / 2.0));
    return 2.0 * half_nu;
}

// ---------------------------------------------------------------------------
// Elements -> ECI state vector
// Uses the perifocal (PQW) frame then rotates to ECI via RAAN, inc, AOP.
// ---------------------------------------------------------------------------
OrbitState OrbitalElements::toStateVector(double mu) const {
    const double p = sma * (1.0 - ecc * ecc);  // semi-latus rectum
    const double r_mag = p / (1.0 + ecc * std::cos(ta));

    // Position and velocity in perifocal frame (PQW)
    const double cos_ta = std::cos(ta);
    const double sin_ta = std::sin(ta);
    const double sqrt_mu_p = std::sqrt(mu / p);

    const Vec3 r_pqw{r_mag * cos_ta, r_mag * sin_ta, 0.0};
    const Vec3 v_pqw{-sqrt_mu_p * sin_ta, sqrt_mu_p * (ecc + cos_ta), 0.0};

    // Rotation matrix: PQW -> ECI
    // R = R_z(-RAAN) * R_x(-inc) * R_z(-AOP)
    const double cO = std::cos(raan), sO = std::sin(raan);
    const double ci = std::cos(inc),  si = std::sin(inc);
    const double cw = std::cos(aop),  sw = std::sin(aop);

    // Row vectors of the rotation matrix (column-major: each Vec3 is a column)
    const double Q00 =  cO*cw - sO*sw*ci;
    const double Q10 =  sO*cw + cO*sw*ci;
    const double Q20 =  sw*si;
    const double Q01 = -cO*sw - sO*cw*ci;
    const double Q11 = -sO*sw + cO*cw*ci;
    const double Q21 =  cw*si;
    const double Q02 =  sO*si;
    const double Q12 = -cO*si;
    const double Q22 =  ci;

    const Vec3 r_eci{
        Q00*r_pqw.x + Q01*r_pqw.y + Q02*r_pqw.z,
        Q10*r_pqw.x + Q11*r_pqw.y + Q12*r_pqw.z,
        Q20*r_pqw.x + Q21*r_pqw.y + Q22*r_pqw.z
    };
    const Vec3 v_eci{
        Q00*v_pqw.x + Q01*v_pqw.y + Q02*v_pqw.z,
        Q10*v_pqw.x + Q11*v_pqw.y + Q12*v_pqw.z,
        Q20*v_pqw.x + Q21*v_pqw.y + Q22*v_pqw.z
    };

    return {r_eci, v_eci, 0.0};
}

// ---------------------------------------------------------------------------
// ECI state vector -> Elements
// ---------------------------------------------------------------------------
OrbitalElements OrbitalElements::fromStateVector(const OrbitState& state, double mu) {
    const Vec3& r = state.position;
    const Vec3& v = state.velocity;
    const double r_mag = r.norm();
    const double v_mag = v.norm();

    // Angular momentum
    const Vec3 h = r.cross(v);
    const double h_mag = h.norm();

    // Node vector (crosses ascending node in equatorial plane)
    const Vec3 k{0.0, 0.0, 1.0};
    const Vec3 n = k.cross(h);
    const double n_mag = n.norm();

    // Eccentricity vector
    const Vec3 e_vec = (1.0/mu) * ((v_mag*v_mag - mu/r_mag)*r - r.dot(v)*v);
    const double e = e_vec.norm();

    // Semi-major axis (energy equation)
    const double energy = 0.5*v_mag*v_mag - mu/r_mag;
    const double a = (std::abs(energy) > 1e-10) ? -mu / (2.0*energy) : h_mag*h_mag/mu;

    // Inclination
    const double i = std::acos(std::clamp(h.z / h_mag, -1.0, 1.0));

    // RAAN
    double RAAN = 0.0;
    if (n_mag > 1e-10) {
        RAAN = std::acos(std::clamp(n.x / n_mag, -1.0, 1.0));
        if (n.y < 0.0) RAAN = Constants::TWO_PI - RAAN;
    }

    // Argument of perigee
    double omega = 0.0;
    if (n_mag > 1e-10 && e > 1e-10) {
        omega = std::acos(std::clamp(n.dot(e_vec) / (n_mag * e), -1.0, 1.0));
        if (e_vec.z < 0.0) omega = Constants::TWO_PI - omega;
    }

    // True anomaly
    double nu = 0.0;
    if (e > 1e-10) {
        nu = std::acos(std::clamp(e_vec.dot(r) / (e * r_mag), -1.0, 1.0));
        if (r.dot(v) < 0.0) nu = Constants::TWO_PI - nu;
    } else {
        // Circular orbit: measure from ascending node
        if (n_mag > 1e-10) {
            nu = std::acos(std::clamp(n.dot(r) / (n_mag * r_mag), -1.0, 1.0));
            if (r.z < 0.0) nu = Constants::TWO_PI - nu;
        }
    }

    return {a, e, i, RAAN, omega, nu};
}

double OrbitalElements::period_s(double mu) const {
    return Constants::TWO_PI * std::sqrt(sma*sma*sma / mu);
}

double OrbitalElements::meanMotion_rads(double mu) const {
    return std::sqrt(mu / (sma*sma*sma));
}
