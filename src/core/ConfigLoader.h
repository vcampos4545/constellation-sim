#pragma once

#include "environment/LinkBudget.h"
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// ---------------------------------------------------------------------------
// Physical and simulation configuration structs
// All internal values stored in SI units. The loader converts from user-facing
// km / degrees to meters / radians as appropriate.
// ---------------------------------------------------------------------------

struct PhysicalProperties {
    double mass_kg          = 260.0;
    double drag_coefficient = 2.2;
    double drag_area_m2     = 2.0;    // cross-sectional area for drag
    double reflectivity     = 1.3;    // Cr (solar radiation pressure coefficient)
    double srp_area_m2      = 2.0;    // area for SRP (may differ from drag area)

    // Solar array, for the power-generation estimate (see MetricsCollector).
    // Attitude isn't modeled in this project, so power assumes an idealized
    // always-sun-tracking array (cos(incidence) = 1 whenever sunlit) -- a
    // best-case bound, not a pointing-loss-aware estimate.
    double solar_array_area_m2   = 4.0;   // total array area
    double solar_array_efficiency = 0.30; // cell efficiency (0-1), ~0.30 typical for modern triple-junction GaAs
};

struct PhysicsConfig {
    bool gravity      = true;
    bool j2           = true;
    bool j3           = false;  // J3 zonal harmonic (pear-shape)
    bool j4           = false;  // J4 zonal harmonic
    bool drag         = true;
    bool srp          = true;
    bool moon_gravity = false;
    bool sun_gravity  = false;
};

struct WalkerConfig {
    double altitude_km        = 550.0;
    double inclination_deg    = 53.0;
    int    total_satellites   = 1584;
    int    planes             = 72;
    int    phasing_factor     = 13;   // F in Walker T/P/F notation
    double eccentricity       = 0.0;
    double arg_of_perigee_deg = 0.0;
};

// Per-satellite Keplerian specification for fully-custom constellations
// (used when constellation.type == "custom" in the JSON).
struct SatelliteSpec {
    int    plane_id{0};
    int    seat_id{0};
    double sma_km{7000.0};
    double eccentricity{0.0};
    double inclination_deg{0.0};
    double raan_deg{0.0};
    double arg_of_perigee_deg{0.0};
    double true_anomaly_deg{0.0};
};

// Active altitude station-keeping. When enabled, satellites are re-boosted
// back toward their nominal SMA whenever drag decay exceeds the deadband,
// and the ΔV actually spent is reported (see src/orbit/StationKeeping.h).
struct StationKeepingConfig {
    bool   enabled              = false;
    double altitude_deadband_km = 5.0;    // max allowed decay before a reboost burn
    double check_interval_s     = 3600.0; // how often to evaluate/correct
};

struct CoverageConfig {
    bool   enabled             = true;
    double grid_resolution_deg = 5.0;  // latitude/longitude grid spacing
    double min_elevation_deg   = 10.0;
    double sample_interval_s   = 300.0;
};

// Pairwise closest-approach screening within the constellation.
// O(N^2) per sample after a one-time radius-band pre-filter, so
// sample_interval_s is intentionally decoupled from the physics timestep --
// keep it coarse for large constellations.
struct ConjunctionConfig {
    bool   enabled           = false;
    double threshold_km      = 5.0;    // only pairs ever closer than this are reported
    double sample_interval_s = 60.0;   // screening cadence
};

struct MetricsConfig {
    CoverageConfig     coverage;
    ConjunctionConfig  conjunction;
    LinkBudget::Config link_budget;
    bool sunlight  = true;
    bool drag      = true;
    bool delta_v   = true;
    bool revisit   = true;
    bool links     = false;
};

// A ground station / solar farm target for access analysis.
struct GroundTarget {
    std::string name;
    double      lat_deg  = 0.0;
    double      lon_deg  = 0.0;
    std::string description;
};

struct SimConfig {
    std::string name          = "Unnamed Simulation";
    double duration_days      = 1.0;
    double timestep_s         = 60.0;
    double epoch_jd           = 2451545.0;  // J2000.0

    // "walker"  → use constellation (WalkerConfig) to generate satellites
    // "custom"  → use explicit_satellites (vector<SatelliteSpec>) directly
    std::string                constellation_type{"walker"};
    WalkerConfig               constellation;
    std::vector<SatelliteSpec> explicit_satellites;

    PhysicalProperties satellite;
    PhysicsConfig      physics;
    MetricsConfig      metrics;
    StationKeepingConfig station_keeping;

    std::vector<GroundTarget> ground_targets;

    std::string output_directory = "output";
    std::string run_name         = "run";

    // If > 0, orbital elements (RAAN, inclination, etc.) are snapshotted at this
    // interval and written to trajectory.csv. Set to 0 to disable.
    double trajectory_sample_interval_s = 0.0;

    // If true, write a CCSDS OEM ephemeris file per satellite (position/velocity
    // history), sampled at trajectory_sample_interval_s (which must be > 0).
    // oem_satellite_ids restricts export to specific satellite indices; empty
    // means "all satellites" (only recommended for small constellations).
    bool             export_oem = false;
    std::vector<int> oem_satellite_ids;

    double duration_s() const { return duration_days * 86400.0; }
};

// ---------------------------------------------------------------------------
// Monte Carlo configuration
// ---------------------------------------------------------------------------

struct MCParameterRange {
    std::string          name;
    std::vector<double>  values;
};

struct MCConfig {
    std::string name          = "Unnamed Experiment";
    int         runs          = 0;     // 0 = auto (product of all parameter counts)
    std::string sampling      = "grid"; // "grid", "random", or "pareto"
    int         threads       = 0;     // 0 = hardware_concurrency

    SimConfig                  base_config;
    std::vector<MCParameterRange> parameters;

    // Used only when sampling == "pareto". Each entry is "maximize:<field>"
    // or "minimize:<field>", where <field> names a ConstellationResult
    // member (e.g. "coverage_pct", "total_satellites", "avg_annual_sk_dv_ms_per_year").
    // The full parameter grid is still run; the Pareto-optimal subset (by
    // these objectives) is written to pareto_frontier.csv.
    std::vector<std::string> pareto_objectives;

    std::string output_directory  = "output";
    std::string experiment_name   = "experiment";
};

// ---------------------------------------------------------------------------
// Loader
// ---------------------------------------------------------------------------

class ConfigLoader {
public:
    // Load a single-simulation config from a JSON file.
    static SimConfig loadSimConfig(const std::string& path);

    // Load a Monte Carlo experiment config from a JSON file.
    static MCConfig  loadMCConfig(const std::string& path);

    // Apply a named parameter override to a SimConfig.
    // Used by the Monte Carlo engine to build per-run configs.
    static void applyParameter(SimConfig& cfg,
                               const std::string& name,
                               double value);

    static double epochStringToJD(const std::string& epoch_str);

private:
    static SimConfig          parseSimConfig(const nlohmann::json& j);
    static WalkerConfig       parseWalker(const nlohmann::json& j);
    static PhysicalProperties parseSatellite(const nlohmann::json& j);
    static PhysicsConfig      parsePhysics(const nlohmann::json& j);
    static MetricsConfig      parseMetrics(const nlohmann::json& j);
    static StationKeepingConfig parseStationKeeping(const nlohmann::json& j);
};
