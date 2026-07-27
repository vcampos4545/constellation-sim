#pragma once

#include "core/ConfigLoader.h"
#include "orbit/Propagator.h"
#include "constellation/Constellation.h"
#include "metrics/MetricsCollector.h"
#include "metrics/ConjunctionScreener.h"
#include <functional>
#include <memory>
#include <optional>

// Single-simulation execution engine.
//
// The engine owns:
//   - A Propagator (shared force models applied to all satellites)
//   - A Constellation (satellite state)
//   - A MetricsCollector (incremental metrics accumulation)
//
// Visualization is decoupled via an optional frame callback that the
// renderer subscribes to. If no callback is registered the engine runs
// fully headless with no synchronization overhead.
class SimulationEngine {
public:
    // Per-frame state snapshot pushed to the renderer when viz is enabled.
    struct FrameData {
        double              time_s;
        std::vector<Vec3>          positions;       // ECI [m]
        std::vector<Vec3>          velocities;      // ECI [m/s]
        std::vector<bool>          in_eclipse;
        Vec3                       sun_dir_eci;
        Vec3                       moon_dir_eci;
    };

    // Static per-satellite metadata (same every frame; populated in constructor).
    struct SatelliteInfo {
        int id{0};
        int plane_id{0};
        int seat_id{0};
    };

    // Time-series snapshot of Keplerian elements for one satellite at one instant.
    struct OrbitalSnapshot {
        double time_s{0};
        int    sat_id{0};
        double sma_km{0};
        double eccentricity{0};
        double inclination_deg{0};
        double raan_deg{0};
        double aop_deg{0};
        double true_anomaly_deg{0};
        double altitude_km{0};
    };

    // Raw position/velocity sample, captured for CCSDS OEM ephemeris export.
    // sat_id here is the satellite's index in constellation order (matching
    // OrbitalSnapshot's convention), not necessarily Satellite::id().
    struct EphemerisSample {
        double time_s{0};
        int    sat_id{0};
        Vec3   position;
        Vec3   velocity;
    };

    using FrameCallback = std::function<void(const FrameData&)>;

    explicit SimulationEngine(const SimConfig& cfg);

    // Register a viz callback (called at each timestep when visualization is on).
    void setFrameCallback(FrameCallback cb) { frame_cb_ = std::move(cb); }

    // Execute the full simulation. Returns the constellation-level result.
    ConstellationResult run(int run_id = 0);

    // Run headless and capture every frame into a vector for visualization playback.
    // Returns the ConstellationResult and all captured FrameData objects.
    std::pair<ConstellationResult, std::vector<FrameData>>
    runAndCapture(int run_id = 0);

    // Access results after run().
    const std::vector<SatelliteResult>&    satelliteResults()    const;
    const std::vector<GroundTargetResult>& groundTargetResults() const;
    const std::vector<SatelliteInfo>&      satelliteInfo()       const;
    const std::vector<OrbitalSnapshot>&    trajectorySnapshots() const { return traj_snapshots_; }
    const std::vector<PassEvent>&          passEvents()          const { return metrics_.passEvents(); }
    const std::vector<EphemerisSample>&    ephemerisSamples()    const { return ephem_samples_; }

    // Empty if metrics.conjunction.enabled was false.
    std::vector<ConjunctionScreener::Event> conjunctionEvents() const;

private:
    SimConfig        cfg_;
    Propagator       propagator_;
    Constellation    constellation_;
    MetricsCollector metrics_;

    std::optional<FrameCallback> frame_cb_;
    std::vector<SatelliteInfo>   sat_info_;        // populated in constructor
    std::vector<OrbitalSnapshot> traj_snapshots_;  // populated during run() if enabled
    double                       traj_next_sample_s_{0.0};
    std::vector<EphemerisSample> ephem_samples_;   // populated during run() if export_oem enabled

    std::unique_ptr<ConjunctionScreener> conjunction_screener_;
    double conjunction_next_sample_s_{0.0};

    std::vector<double> nominal_sma_m_;      // per-satellite station-keeping target, from initial state
    double sk_next_check_s_{0.0};

    void buildPropagator();
    void broadcastFrame(double time_s);
    void sampleTrajectory(double time_s);
    void sampleEphemeris(double time_s);
    void applyStationKeeping(double time_s);
};
