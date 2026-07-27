#pragma once
#include "core/math/Vec3.h"
#include <string>
#include <vector>

// Writes CCSDS Orbit Ephemeris Message (OEM) files -- the standard
// interchange format for position/velocity time histories, readable by
// STK, GMAT, FreeFlyer, and most other mission-design tools.
//
// This covers the commonly-used subset of OEM 2.0: a single META/DATA
// segment with Cartesian state vectors. Covariance blocks and multi-segment
// files are not produced.
namespace OemWriter {

struct EphemerisPoint {
    double time_s{0.0};   // seconds since epoch_jd
    Vec3   position_m;
    Vec3   velocity_ms;
};

struct Metadata {
    std::string object_name;
    std::string object_id;
    double      epoch_jd{0.0};
    std::string center_name = "EARTH";
    std::string ref_frame   = "EME2000";
};

// Write a single-satellite OEM file. `points` must be sorted by time_s.
void write(const std::string& path,
          const Metadata& meta,
          const std::vector<EphemerisPoint>& points);

} // namespace OemWriter
