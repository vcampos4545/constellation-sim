#include "orbit/Satellite.h"

Satellite::Satellite(int id, int plane_id, int seat_id,
                     const OrbitState&       initial_state,
                     const PhysicalProperties& props)
    : id_(id), plane_id_(plane_id), seat_id_(seat_id),
      state_(initial_state), props_(props)
{
}

double Satellite::altitude_m(double earth_radius_m) const {
    return state_.position.norm() - earth_radius_m;
}
