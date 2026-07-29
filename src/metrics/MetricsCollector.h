#pragma once

#include "core/ConfigLoader.h"
#include "orbit/Satellite.h"
#include "environment/EclipseModel.h"
#include "environment/SunModel.h"
#include "environment/EarthModel.h"
#include "environment/AtmosphereModel.h"
#include <vector>
#include <string>
#include <unordered_map>

// ---------------------------------------------------------------------------
// Satellite-level result (one per satellite per simulation run)
// ---------------------------------------------------------------------------
struct SatelliteResult {
    int    run_id{0};
    int    sat_id{0};
    int    plane_id{0};
    int    seat_id{0};
    double time_in_sunlight_pct{0.0};  // [%]
    double time_in_eclipse_pct{0.0};   // [%]
    double max_eclipse_duration_s{0.0}; // longest single continuous eclipse [s] -- worst-case power/thermal stress
    double avg_solar_flux_wm2{0.0};    // mean incident solar flux [W/m^2], averaged over all time (0 while eclipsed)
    double avg_solar_power_w{0.0};     // mean estimated array power [W] (idealized sun-tracking array, see PhysicalProperties)
    double avg_beta_angle_deg{0.0};    // mean beta angle (sun vs. orbit plane) [deg]
    double min_beta_angle_deg{0.0};    // [deg]
    double max_beta_angle_deg{0.0};    // [deg]
    double avg_drag_accel_ms2{0.0};    // average drag acceleration [m/s^2]
    double total_drag_dv_ms{0.0};      // accumulated drag ΔV [m/s]
    double stationkeeping_dv_ms{0.0};  // SK ΔV actually spent (or drag-ΔV estimate if SK disabled) [m/s]
    double annual_sk_dv_ms_per_year{0.0}; // stationkeeping_dv_ms extrapolated to a 1-year budget
    int    sk_maneuver_count{0};       // number of reboost burns performed (0 if SK disabled)
    double avg_altitude_km{0.0};       // [km]
    double min_altitude_km{1e9};       // [km]
    double orbital_lifetime_days{0.0}; // rough estimate
};

// ---------------------------------------------------------------------------
// Constellation-level result (one per simulation run)
// ---------------------------------------------------------------------------
struct ConstellationResult {
    int    run_id{0};
    std::string run_name;

    // Constellation parameters (echoed from config)
    double altitude_km{0.0};
    double inclination_deg{0.0};
    int    total_satellites{0};
    int    planes{0};
    int    sats_per_plane{0};

    // Coverage
    double coverage_pct{0.0};          // % of Earth covered (time-averaged)
    double revisit_time_avg_s{0.0};    // average revisit gap [s]
    double revisit_time_max_s{0.0};    // maximum revisit gap [s]

    // Ground access
    double avg_pass_duration_s{0.0};
    double avg_elevation_deg{0.0};

    // Inter-satellite
    double avg_nearest_neighbor_km{0.0};

    // Link budget (only populated if metrics.link_budget.enabled)
    double avg_datarate_mbps{0.0};   // mean achievable rate across ground targets, while visible

    double avg_solar_power_w{0.0};   // mean estimated array power across the fleet [W]

    // Fleet averages
    double avg_drag_dv_ms{0.0};
    double avg_sk_dv_ms{0.0};
    double avg_annual_sk_dv_ms_per_year{0.0};
    double avg_sunlit_pct{0.0};
    double avg_altitude_km{0.0};
    double min_altitude_km{0.0};

    // Deployment
    double deployment_dv_per_sat_ms{0.0};
};

// ---------------------------------------------------------------------------
// Per-ground-target access statistics (one per target per simulation run)
// ---------------------------------------------------------------------------
struct GroundTargetResult {
    std::string name;
    double      lat_deg{0.0};
    double      lon_deg{0.0};

    // Access = satellite elevation above min_elevation_deg (regardless of eclipse)
    double visible_pct{0.0};         // % sim time with ≥1 sat in view
    double coverage_time_s{0.0};     // absolute visible time [s]

    // Illumination = satellite in view AND in sunlight (can reflect)
    double illuminated_pct{0.0};     // % sim time with ≥1 sunlit sat in view
    double illuminated_time_s{0.0};  // absolute illuminated time [s]

    double avg_elevation_deg{0.0};   // mean best-elevation during visible intervals
    double max_elevation_deg{0.0};   // peak elevation seen [deg]
    double avg_pass_duration_s{0.0}; // mean continuous pass duration [s]
    int    pass_count{0};

    // Link budget (only populated if metrics.link_budget.enabled)
    double avg_datarate_mbps{0.0};   // mean achievable rate (best visible satellite) while in view
    double min_datarate_mbps{0.0};   // worst-case rate during any visible sample
    double peak_datarate_mbps{0.0};  // best-case rate during any visible sample
};

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Per-pass record: one AOS→LOS event for a (ground-target, satellite) pair.
// ---------------------------------------------------------------------------
struct PassEvent {
    std::string target_name;
    int         sat_id{0};
    double      aos_s{0.0};      // Acquisition of Signal [s since epoch]
    double      los_s{0.0};      // Loss of Signal [s since epoch]
    double      duration_s{0.0}; // LOS − AOS
    double      max_elev_deg{0.0};
};

// MetricsCollector: updated each simulation timestep
// ---------------------------------------------------------------------------
class MetricsCollector {
public:
    explicit MetricsCollector(const MetricsConfig& cfg, const SimConfig& sim_cfg);

    // Called at each timestep with the current satellite states.
    void update(const std::vector<Satellite*>& satellites, double time_s);

    // Called once at end of simulation to finalize all metrics.
    ConstellationResult finalize(int run_id, const SimConfig& cfg);

    const std::vector<SatelliteResult>&     satelliteResults()    const { return sat_results_; }
    const std::vector<GroundTargetResult>&  groundTargetResults() const { return gt_results_; }
    const std::vector<PassEvent>&           passEvents()          const { return pass_events_; }

private:
    MetricsConfig cfg_;
    double        epoch_jd_;
    double        total_time_s_{0.0};
    int           update_count_{0};

    // Per-satellite accumulation state
    struct SatAccum {
        double sunlit_s{0.0};
        double eclipse_s{0.0};
        double drag_dv_ms{0.0};
        double drag_accel_sum{0.0};
        int    drag_samples{0};
        double alt_sum_km{0.0};
        double min_alt_km{1e9};
        int    alt_samples{0};
        double sk_dv_latest{0.0};   // mirrors Satellite::metrics().accumulated_sk_dv_ms
        int    sk_maneuvers_latest{0};

        // Eclipse interval tracking (distinct from the sunlit_s/eclipse_s
        // totals above, which only capture the aggregate fraction).
        bool   in_eclipse{false};
        double eclipse_start_s{0.0};
        double max_eclipse_duration_s{0.0};

        // Solar flux / power
        double flux_sum_wm2{0.0};
        double power_sum_w{0.0};

        // Beta angle
        double beta_sum_deg{0.0};
        double beta_min_deg{1e9};
        double beta_max_deg{-1e9};
        int    beta_samples{0};
    };
    std::vector<SatAccum> sat_accum_;
    int num_satellites_{0};

    // Coverage grid
    struct GridPoint {
        Vec3   pos_ecef;   // fixed ECEF position
        double last_coverage_time_s{-1e9};
        double revisit_sum_s{0.0};
        double max_revisit_s{0.0};
        int    revisit_count{0};
        bool   currently_covered{false};
        double coverage_time_s{0.0};
    };
    std::vector<GridPoint> grid_points_;
    double sample_timer_{0.0};
    int    coverage_samples_{0};
    double coverage_acc_{0.0};

    // Inter-satellite spacing, sampled at the same (coverage) cadence: mean
    // per-satellite nearest-neighbor distance, averaged across satellites
    // and then across samples. O(N^2) per sample -- fine at coverage's
    // already-reduced cadence, but keep sample_interval_s coarse for very
    // large constellations.
    double nn_sum_km_{0.0};
    int    nn_samples_{0};

    void initGrid(double resolution_deg);
    void updateCoverage(const std::vector<Satellite*>& sats, double time_s);
    void updateSatMetrics(const std::vector<Satellite*>& sats, double dt);

    std::vector<SatelliteResult>    sat_results_;
    std::vector<GroundTargetResult> gt_results_;

    // Per-ground-target accumulation
    struct GroundTargetAccum {
        Vec3   pos_ecef;
        double lat_deg{0.0};
        double lon_deg{0.0};
        std::string name;

        double visible_s{0.0};
        double illuminated_s{0.0};
        double elev_sum_deg{0.0};
        double max_elev_deg{0.0};
        int    visible_samples{0};

        bool   in_pass{false};
        double pass_start_s{0.0};
        double pass_dur_sum_s{0.0};
        int    pass_count{0};

        double datarate_sum_mbps{0.0};
        double min_datarate_mbps{1e18};
        double peak_datarate_mbps{0.0};
        int    datarate_samples{0};

        // Per-satellite pass state (index = sat index in constellation).
        // Populated on first update when num_satellites_ is known.
        struct SatPassState {
            bool   in_pass{false};
            double aos_s{0.0};
            double max_elev_deg{-90.0};
        };
        std::vector<SatPassState> sat_passes;
    };
    std::vector<GroundTargetAccum> gt_accum_;
    std::vector<PassEvent>         pass_events_;
};
