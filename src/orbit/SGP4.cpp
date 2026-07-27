#include "orbit/SGP4.h"
#include "core/math/Constants.h"
#include <cmath>
#include <stdexcept>

namespace {
// WGS72 constants -- the gravity model NORAD TLEs are conventionally
// distributed against (matches the reference SGP4 implementation).
constexpr double kMu            = 398600.8;           // [km^3/s^2]
constexpr double kRadiusEarthKm = 6378.135;            // [km]
constexpr double kJ2            = 0.001082616;
constexpr double kJ3            = -0.00000253881;
constexpr double kJ4            = -0.00000165597;
constexpr double kX2o3          = 2.0 / 3.0;

double xke() { return 60.0 / std::sqrt(kRadiusEarthKm * kRadiusEarthKm * kRadiusEarthKm / kMu); }
} // namespace

SGP4::SGP4(const TLE& tle) {
    initialize(tle);
}

void SGP4::initialize(const TLE& tle) {
    epoch_jd_ = tle.epoch_jd;
    ecco_     = tle.eccentricity;
    inclo_    = tle.inclination_rad;
    nodeo_    = tle.raan_rad;
    argpo_    = tle.arg_perigee_rad;
    mo_       = tle.mean_anomaly_rad;
    bstar_    = tle.bstar;

    const double xke_v = xke();
    const double j3oj2 = kJ3 / kJ2;

    cosio_ = std::cos(inclo_);
    sinio_ = std::sin(inclo_);
    const double theta2 = cosio_ * cosio_;
    x3thm1_ = 3.0 * theta2 - 1.0;
    x1mth2_ = 1.0 - theta2;
    x7thm1_ = 7.0 * theta2 - 1.0;

    const double eosq   = ecco_ * ecco_;
    const double betao2 = 1.0 - eosq;
    const double betao  = std::sqrt(betao2);

    // Recover the original (Brouwer) mean motion and semi-major axis from
    // the Kozai mean motion encoded in the TLE.
    const double a1 = std::pow(xke_v / tle.mean_motion_rad_min, kX2o3);
    const double del1 = 1.5 * kJ2 * x3thm1_ / (a1 * a1 * betao * betao2);
    double ao = a1 * (1.0 - del1 * (0.5 * (1.0 / 3.0) + del1 * (1.0 + 134.0 / 81.0 * del1)));
    const double delo = 1.5 * kJ2 * x3thm1_ / (ao * ao * betao * betao2);
    no_ = tle.mean_motion_rad_min / (1.0 + delo);
    ao  = std::pow(xke_v / no_, kX2o3);
    ao_ = ao;

    const double po = ao * betao2;
    const double perige_km = (ao * (1.0 - ecco_) - 1.0) * kRadiusEarthKm;

    double s4, qoms24;
    if (perige_km < 156.0) {
        const double s4_alt = (perige_km < 98.0) ? 20.0 : (perige_km - 78.0);
        qoms24 = std::pow((120.0 - s4_alt) / kRadiusEarthKm, 4.0);
        s4 = s4_alt / kRadiusEarthKm + 1.0;
    } else {
        s4 = 78.0 / kRadiusEarthKm + 1.0;
        qoms24 = std::pow((120.0 - 78.0) / kRadiusEarthKm, 4.0);
    }

    const double pinvsq = 1.0 / (po * po);
    const double tsi = 1.0 / (po - s4);
    eta_ = ao * ecco_ * tsi;
    const double etasq = eta_ * eta_;
    const double eeta = ecco_ * eta_;
    const double psisq = std::abs(1.0 - etasq);
    const double coef = qoms24 * std::pow(tsi, 4.0);
    const double coef1 = coef / std::pow(psisq, 3.5);

    const double c2 = coef1 * no_ *
        (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) +
         0.375 * kJ2 * tsi / psisq * x3thm1_ * (8.0 + 3.0 * etasq * (8.0 + etasq)));
    c1_ = bstar_ * c2;

    const double a3ovk2 = -kJ3 / kJ2;
    double c3 = 0.0;
    if (ecco_ > 1e-4) {
        c3 = coef * tsi * a3ovk2 * no_ * sinio_ / ecco_;
    }

    c4_ = 2.0 * no_ * coef1 * ao * betao2 *
        (eta_ * (2.0 + 0.5 * etasq) + ecco_ * (0.5 + 2.0 * etasq) -
         kJ2 * tsi / (ao * psisq) *
             (-3.0 * x3thm1_ * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) +
              0.75 * x1mth2_ * (2.0 * etasq - eeta * (1.0 + etasq)) * std::cos(2.0 * argpo_)));
    c5_ = 2.0 * coef1 * ao * betao2 * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);

    const double theta4 = theta2 * theta2;
    const double temp1 = 1.5 * kJ2 * pinvsq * no_;
    const double temp2 = 0.5 * temp1 * kJ2 * pinvsq;
    const double temp3 = -0.46875 * kJ4 * pinvsq * pinvsq * no_;

    mdot_ = no_ + 0.5 * temp1 * betao * x3thm1_ +
            0.0625 * temp2 * betao * (13.0 - 78.0 * theta2 + 137.0 * theta4);
    argpdot_ = -0.5 * temp1 * (1.0 - 5.0 * theta2) +
               0.0625 * temp2 * (7.0 - 114.0 * theta2 + 395.0 * theta4) +
               temp3 * (3.0 - 36.0 * theta2 + 49.0 * theta4);

    const double xhdot1 = -temp1 * cosio_;
    nodedot_ = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * theta2) + 2.0 * temp3 * (3.0 - 7.0 * theta2)) * cosio_;
    nodecf_ = 3.5 * betao2 * xhdot1 * c1_;
    t2cof_ = 1.5 * c1_;

    // Guard the near-180deg-inclination singularity (1 + cosio -> 0); not
    // expected for the LEO/near-polar constellations this project targets.
    const double denom = (1.0 + cosio_);
    xlcof_ = (std::abs(denom) > 1.5e-12)
                 ? 0.125 * a3ovk2 * sinio_ * (3.0 + 5.0 * cosio_) / denom
                 : 0.0;
    aycof_ = 0.25 * a3ovk2 * sinio_;

    omgcof_ = bstar_ * c3 * std::cos(argpo_);
    xmcof_ = (ecco_ > 1e-4) ? (-kX2o3 * coef * bstar_ / eeta) : 0.0;

    const double delmo_base = 1.0 + eta_ * std::cos(mo_);
    delmo_ = delmo_base * delmo_base * delmo_base;
    sinmao_ = std::sin(mo_);

    (void)j3oj2;
}

SGP4::StateVector SGP4::propagate(double tsince) const {
    const double xke_v = xke();
    const double twopi = Constants::TWO_PI;

    const double xmdf   = mo_ + mdot_ * tsince;
    const double argpdf = argpo_ + argpdot_ * tsince;
    const double nodedf = nodeo_ + nodedot_ * tsince;

    double argpm = argpdf;
    double mm    = xmdf;

    const double t2 = tsince * tsince;
    double nodem = nodedf + nodecf_ * t2;
    double tempa = 1.0 - c1_ * tsince;
    double tempe = bstar_ * c4_ * tsince;
    double templ = t2cof_ * t2;

    const double delomg = omgcof_ * tsince;
    const double delmtemp = 1.0 + eta_ * std::cos(xmdf);
    const double delm = xmcof_ * (delmtemp * delmtemp * delmtemp - delmo_);
    const double temp_dm = delomg + delm;
    mm = xmdf + temp_dm;
    argpm = argpdf - temp_dm;
    tempe = tempe + bstar_ * c5_ * (std::sin(mm) - sinmao_);

    double nm = no_;
    double em = ecco_ - tempe;

    const double am = std::pow(xke_v / nm, kX2o3) * tempa * tempa;
    nm = xke_v / std::pow(am, 1.5);

    if (em >= 1.0 || em < -0.001)
        throw std::runtime_error("SGP4: eccentricity out of bounds during propagation");
    if (em < 1.0e-6) em = 1.0e-6;

    mm = mm + no_ * templ;
    double xlm = mm + argpm + nodem;

    nodem = std::fmod(nodem, twopi);
    argpm = std::fmod(argpm, twopi);
    xlm   = std::fmod(xlm, twopi);
    mm    = std::fmod(xlm - argpm - nodem, twopi);

    const double sinim = std::sin(inclo_);
    const double cosim = std::cos(inclo_);

    // Long-period periodics
    const double axnl = em * std::cos(argpm);
    const double temp_ep = 1.0 / (am * (1.0 - em * em));
    const double aynl = em * std::sin(argpm) + temp_ep * aycof_;
    const double xl = mm + argpm + nodem + temp_ep * xlcof_ * axnl;

    // Solve Kepler's equation
    const double u = std::fmod(xl - nodem, twopi);
    double eo1 = u;
    double sineo1 = 0.0, coseo1 = 0.0;
    for (int ktr = 0; ktr < 10; ++ktr) {
        sineo1 = std::sin(eo1);
        coseo1 = std::cos(eo1);
        double tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl;
        tem5 = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
        if (std::abs(tem5) >= 0.95) tem5 = (tem5 > 0.0) ? 0.95 : -0.95;
        eo1 += tem5;
        if (std::abs(tem5) < 1.0e-12) break;
    }

    const double ecose = axnl * coseo1 + aynl * sineo1;
    const double esine = axnl * sineo1 - aynl * coseo1;
    const double el2   = axnl * axnl + aynl * aynl;
    const double pl    = am * (1.0 - el2);
    if (pl < 0.0)
        throw std::runtime_error("SGP4: negative semi-latus rectum during propagation");

    const double rl     = am * (1.0 - ecose);
    const double rdotl  = std::sqrt(am) * esine / rl;
    const double rvdotl = std::sqrt(pl) / rl;
    const double betal  = std::sqrt(1.0 - el2);
    const double temp_su = esine / (1.0 + betal);
    const double sinu = am / rl * (sineo1 - aynl - axnl * temp_su);
    const double cosu = am / rl * (coseo1 - axnl + aynl * temp_su);
    double su = std::atan2(sinu, cosu);
    const double sin2u = (cosu + cosu) * sinu;
    const double cos2u = 1.0 - 2.0 * sinu * sinu;
    const double temp_pl = 1.0 / pl;
    const double temp1s = 0.5 * kJ2 * temp_pl;
    const double temp2s = temp1s * temp_pl;

    const double mrt   = rl * (1.0 - 1.5 * temp2s * betal * x3thm1_) + 0.5 * temp1s * x1mth2_ * cos2u;
    su                 = su - 0.25 * temp2s * x7thm1_ * sin2u;
    const double xnode = nodem + 1.5 * temp2s * cosim * sin2u;
    const double xinc  = inclo_ + 1.5 * temp2s * cosim * sinim * cos2u;
    const double mvt    = rdotl - nm * temp1s * x1mth2_ * sin2u / xke_v;
    const double rvdot  = rvdotl + nm * temp1s * (x1mth2_ * cos2u + 1.5 * x3thm1_) / xke_v;

    const double sinsu = std::sin(su), cossu = std::cos(su);
    const double snod  = std::sin(xnode), cnod = std::cos(xnode);
    const double sini  = std::sin(xinc), cosi = std::cos(xinc);
    const double xmx = -snod * cosi;
    const double xmy =  cnod * cosi;
    const double ux = xmx * sinsu + cnod * cossu;
    const double uy = xmy * sinsu + snod * cossu;
    const double uz = sini * sinsu;
    const double vx = xmx * cossu - cnod * sinsu;
    const double vy = xmy * cossu - snod * sinsu;
    const double vz = sini * cossu;

    const double mr = mrt * kRadiusEarthKm;
    const double vscale = kRadiusEarthKm * xke_v / 60.0;

    StateVector sv;
    sv.position_km = {mr * ux, mr * uy, mr * uz};
    sv.velocity_km_s = {
        (mvt * ux + rvdot * vx) * vscale,
        (mvt * uy + rvdot * vy) * vscale,
        (mvt * uz + rvdot * vz) * vscale
    };
    return sv;
}

OrbitState SGP4::toOrbitState(double minutes_since_epoch) const {
    const StateVector sv = propagate(minutes_since_epoch);
    OrbitState s;
    s.position = sv.position_km * 1000.0;
    s.velocity = sv.velocity_km_s * 1000.0;
    s.time_s   = minutes_since_epoch * 60.0;
    return s;
}
