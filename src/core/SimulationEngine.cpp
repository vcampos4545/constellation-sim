#include "core/SimulationEngine.h"
#include "physics/Gravity.h"
#include "physics/J2Perturbation.h"
#include "physics/ZonalHarmonics.h"
#include "physics/AtmosphericDrag.h"
#include "physics/SolarRadiationPressure.h"
#include "physics/ThirdBodyGravity.h"
#include "environment/MoonModel.h"
#include "environment/EclipseModel.h"
#include "environment/SunModel.h"
#include "orbit/OrbitalElements.h"
#include "core/math/Constants.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <mutex>

// Serialize all simulation progress output so multi-threaded MC runs
// don't interleave partial lines on the terminal.
static std::mutex g_console_mutex;

SimulationEngine::SimulationEngine(const SimConfig& cfg)
    : cfg_(cfg),
      constellation_(cfg.constellation_type == "custom"
                     ? Constellation::createCustom(cfg.explicit_satellites)
                     : Constellation::createWalker(cfg.constellation)),
      metrics_(cfg.metrics, cfg)
{
    buildPropagator();

    // Apply satellite physical properties.
    for (auto* sat : constellation_.satellites()) {
        const_cast<PhysicalProperties&>(sat->properties()) = cfg.satellite;
        sat_info_.push_back({sat->id(), sat->planeId(), sat->seatId()});
    }
}

void SimulationEngine::buildPropagator() {
    const auto& phy = cfg_.physics;
    const double mu  = Constants::GM_EARTH;
    const double re  = Constants::EARTH_RADIUS_M;
    const double j2  = Constants::J2;

    if (phy.gravity) {
        auto g = std::make_unique<Gravity>(mu);
        propagator_.addForceModel(std::move(g));
    }
    if (phy.j2) {
        if (phy.j3 || phy.j4) {
            // Use the combined model when higher-order terms are enabled
            auto zh = std::make_unique<ZonalHarmonics>(mu, re, j2,
                phy.j3 ? Constants::J3 : 0.0,
                phy.j4 ? Constants::J4 : 0.0);
            propagator_.addForceModel(std::move(zh));
        } else {
            auto j = std::make_unique<J2Perturbation>(mu, j2, re);
            propagator_.addForceModel(std::move(j));
        }
    }
    if (phy.drag) {
        auto d = std::make_unique<AtmosphericDrag>(re);
        propagator_.addForceModel(std::move(d));
    }
    if (phy.srp) {
        auto s = std::make_unique<SolarRadiationPressure>(cfg_.epoch_jd, re);
        propagator_.addForceModel(std::move(s));
    }
    if (phy.moon_gravity) {
        propagator_.addForceModel(
            std::make_unique<ThirdBodyGravity>(ThirdBodyType::Moon, cfg_.epoch_jd));
    }
    if (phy.sun_gravity) {
        propagator_.addForceModel(
            std::make_unique<ThirdBodyGravity>(ThirdBodyType::Sun, cfg_.epoch_jd));
    }
}

void SimulationEngine::broadcastFrame(double time_s) {
    if (!frame_cb_) return;

    FrameData frame;
    frame.time_s      = time_s;
    frame.sun_dir_eci  = SunModel::direction_eci(time_s, cfg_.epoch_jd);
    frame.moon_dir_eci = MoonModel::direction_eci(time_s, cfg_.epoch_jd);

    const auto& sats = constellation_.satellites();
    frame.positions.reserve(sats.size());
    frame.velocities.reserve(sats.size());
    frame.in_eclipse.reserve(sats.size());

    for (const auto* sat : sats) {
        frame.positions.push_back(sat->state().position);
        frame.velocities.push_back(sat->state().velocity);
        frame.in_eclipse.push_back(
            EclipseModel::inEclipse(sat->state().position, frame.sun_dir_eci));
    }

    (*frame_cb_)(frame);
}

void SimulationEngine::sampleTrajectory(double time_s) {
    const double mu = Constants::GM_EARTH;
    const auto& sats = constellation_.satellites();
    for (int i = 0; i < static_cast<int>(sats.size()); ++i) {
        const OrbitalElements elems =
            OrbitalElements::fromStateVector(sats[i]->state(), mu);
        OrbitalSnapshot snap;
        snap.time_s          = time_s;
        snap.sat_id          = i;
        snap.sma_km          = elems.sma / 1000.0;
        snap.eccentricity    = elems.ecc;
        snap.inclination_deg = elems.inc  * Constants::RAD2DEG;
        snap.raan_deg        = elems.raan * Constants::RAD2DEG;
        snap.aop_deg         = elems.aop  * Constants::RAD2DEG;
        snap.true_anomaly_deg = elems.ta  * Constants::RAD2DEG;
        snap.altitude_km     = (elems.sma / 1000.0) - Constants::EARTH_RADIUS_KM;
        traj_snapshots_.push_back(snap);
    }
}

ConstellationResult SimulationEngine::run(int run_id) {
    const double dt     = cfg_.timestep_s;
    const double t_end  = cfg_.duration_s();
    auto& sats          = constellation_.satellites();

    // (Progress reporting removed to keep MC output clean)
    [[maybe_unused]] const double report_interval = t_end / 10.0;
    [[maybe_unused]] double next_report = report_interval;

    {
        std::lock_guard lock(g_console_mutex);
        std::cout << "[" << cfg_.run_name << "] propagating "
                  << constellation_.totalSatellites() << " satellites for "
                  << cfg_.duration_days << " days\n";
    }

    const bool do_traj = (cfg_.trajectory_sample_interval_s > 0.0);
    if (do_traj) {
        traj_snapshots_.clear();
        traj_snapshots_.reserve(
            static_cast<std::size_t>(t_end / cfg_.trajectory_sample_interval_s + 2)
            * sats.size());
        traj_next_sample_s_ = 0.0;
        sampleTrajectory(0.0);
        traj_next_sample_s_ = cfg_.trajectory_sample_interval_s;
    }

    metrics_.update(sats, 0.0);
    broadcastFrame(0.0);

    for (double t = 0.0; t < t_end; t += dt) {
        for (auto* sat : sats) {
            propagator_.step(sat->state(), sat->properties(), dt);
        }
        metrics_.update(sats, t + dt);
        broadcastFrame(t + dt);

        if (do_traj && (t + dt) >= traj_next_sample_s_) {
            sampleTrajectory(t + dt);
            traj_next_sample_s_ += cfg_.trajectory_sample_interval_s;
        }

        if (t + dt >= next_report) {
            next_report += report_interval;
        }
    }

    {
        std::lock_guard lock(g_console_mutex);
        std::cout << "[" << cfg_.run_name << "] done\n";
    }
    return metrics_.finalize(run_id, cfg_);
}

const std::vector<SatelliteResult>& SimulationEngine::satelliteResults() const {
    return metrics_.satelliteResults();
}

const std::vector<GroundTargetResult>& SimulationEngine::groundTargetResults() const {
    return metrics_.groundTargetResults();
}

const std::vector<SimulationEngine::SatelliteInfo>& SimulationEngine::satelliteInfo() const {
    return sat_info_;
}

std::pair<ConstellationResult, std::vector<SimulationEngine::FrameData>>
SimulationEngine::runAndCapture(int run_id) {
    std::vector<FrameData> frames;
    // Reserve capacity: duration / timestep + small headroom
    frames.reserve(static_cast<std::size_t>(cfg_.duration_s() / cfg_.timestep_s) + 2);

    setFrameCallback([&frames](const FrameData& f) { frames.push_back(f); });
    ConstellationResult cr = run(run_id);
    frame_cb_.reset();  // detach callback so the engine can be reused

    return {std::move(cr), std::move(frames)};
}
