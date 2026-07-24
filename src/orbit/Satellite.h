#pragma once

#include "orbit/OrbitState.h"
#include "core/ConfigLoader.h"

struct SatelliteMetrics {
    double time_in_sunlight_s    = 0.0;
    double time_in_eclipse_s     = 0.0;
    double accumulated_drag_dv_ms = 0.0;
    double accumulated_sk_dv_ms  = 0.0;
    double total_time_s          = 0.0;

    double sunlit_fraction()  const { return (total_time_s > 0) ? time_in_sunlight_s / total_time_s : 0.0; }
    double eclipse_fraction() const { return (total_time_s > 0) ? time_in_eclipse_s  / total_time_s : 0.0; }
};

class Satellite {
public:
    Satellite(int id, int plane_id, int seat_id,
              const OrbitState&      initial_state,
              const PhysicalProperties& props);

    int id()       const { return id_; }
    int planeId()  const { return plane_id_; }
    int seatId()   const { return seat_id_; }

    OrbitState&               state()       { return state_; }
    const OrbitState&         state() const { return state_; }
    const PhysicalProperties& properties()  const { return props_; }
    SatelliteMetrics&         metrics()     { return metrics_; }
    const SatelliteMetrics&   metrics() const { return metrics_; }

    double altitude_m(double earth_radius_m) const;

private:
    int                id_;
    int                plane_id_;
    int                seat_id_;
    OrbitState         state_;
    PhysicalProperties props_;
    SatelliteMetrics   metrics_;
};
