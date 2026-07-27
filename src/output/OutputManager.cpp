#include "output/OutputManager.h"
#include "output/OemWriter.h"
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <map>

namespace fs = std::filesystem;

OutputManager::OutputManager(const std::string& base_dir,
                             const std::string& experiment_name) {
    exp_dir_ = base_dir + "/" + experiment_name;
    ensureDir(exp_dir_);
    summary_path_ = exp_dir_ + "/experiment_summary.csv";
    summary_writer_ = std::make_unique<CsvWriter>(summary_path_);
}

void OutputManager::ensureDir(const std::string& path) {
    fs::create_directories(path);
}

std::string OutputManager::runDirName(int run_id) {
    std::ostringstream oss;
    oss << "run_" << std::setw(4) << std::setfill('0') << run_id;
    return oss.str();
}

std::string OutputManager::runDir(int run_id) const {
    return exp_dir_ + "/" + runDirName(run_id);
}

void OutputManager::writeSummaryRow(CsvWriter& w, const ConstellationResult& cr) {
    w.writeRowV(cr.run_id, cr.run_name,
                cr.altitude_km, cr.inclination_deg,
                cr.total_satellites, cr.planes, cr.sats_per_plane,
                cr.coverage_pct,
                cr.revisit_time_avg_s,
                cr.revisit_time_max_s,
                cr.avg_drag_dv_ms,
                cr.avg_sk_dv_ms,
                cr.avg_annual_sk_dv_ms_per_year,
                cr.avg_sunlit_pct,
                cr.avg_altitude_km,
                cr.min_altitude_km,
                cr.deployment_dv_per_sat_ms,
                cr.avg_datarate_mbps);
}

void OutputManager::writeRun(int run_id,
                              const ConstellationResult& cr,
                              const std::vector<SatelliteResult>& sat_results,
                              const std::vector<GroundTargetResult>& gt_results) {
    std::lock_guard lock(mutex_);

    // Write experiment-level summary header once
    if (!summary_header_written_) {
        summary_writer_->writeHeader({
            "run_id","run_name",
            "altitude_km","inclination_deg",
            "total_satellites","planes","sats_per_plane",
            "coverage_pct",
            "revisit_time_avg_s","revisit_time_max_s",
            "avg_drag_dv_ms","avg_sk_dv_ms","avg_annual_sk_dv_ms_per_year",
            "avg_sunlit_pct","avg_altitude_km","min_altitude_km",
            "deployment_dv_per_sat_ms","avg_datarate_mbps"
        });
        summary_header_written_ = true;
    }

    writeSummaryRow(*summary_writer_, cr);
    summary_writer_->flush();

    // Per-run directory and files
    const std::string rdir = runDir(run_id);
    ensureDir(rdir);

    // Run summary (single-row CSV)
    {
        CsvWriter w(rdir + "/summary.csv");
        w.writeHeader({
            "run_id","run_name",
            "altitude_km","inclination_deg",
            "total_satellites","planes","sats_per_plane",
            "coverage_pct","revisit_time_avg_s","revisit_time_max_s",
            "avg_drag_dv_ms","avg_sk_dv_ms","avg_annual_sk_dv_ms_per_year",
            "avg_sunlit_pct","avg_altitude_km","min_altitude_km",
            "deployment_dv_per_sat_ms","avg_datarate_mbps"
        });
        writeSummaryRow(w, cr);
    }

    // Satellite results
    writeSatelliteCsv(rdir + "/satellites.csv", sat_results);

    // Ground target results (only written if any targets were configured)
    if (!gt_results.empty())
        writeGroundTargetCsv(rdir + "/ground_targets.csv", gt_results);
}

void OutputManager::writeSatelliteCsv(const std::string& path,
                                       const std::vector<SatelliteResult>& sats) {
    CsvWriter w(path);
    w.writeHeader({
        "run_id","satellite_id","plane_id","seat_id",
        "time_in_sunlight_pct","time_in_eclipse_pct",
        "avg_drag_accel_ms2","total_drag_dv_ms",
        "stationkeeping_dv_ms","annual_sk_dv_ms_per_year","sk_maneuver_count",
        "avg_altitude_km","min_altitude_km",
        "orbital_lifetime_days"
    });
    for (const auto& s : sats) {
        w.writeRowV(s.run_id, s.sat_id, s.plane_id, s.seat_id,
                    s.time_in_sunlight_pct, s.time_in_eclipse_pct,
                    s.avg_drag_accel_ms2, s.total_drag_dv_ms,
                    s.stationkeeping_dv_ms, s.annual_sk_dv_ms_per_year, s.sk_maneuver_count,
                    s.avg_altitude_km,
                    s.min_altitude_km, s.orbital_lifetime_days);
    }
}

void OutputManager::writeGroundTargetCsv(const std::string& path,
                                          const std::vector<GroundTargetResult>& gts) {
    CsvWriter w(path);
    w.writeHeader({
        "name","lat_deg","lon_deg",
        "visible_pct","illuminated_pct",
        "avg_elevation_deg","max_elevation_deg",
        "avg_pass_duration_s","pass_count",
        "coverage_time_s","illuminated_time_s",
        "avg_datarate_mbps","min_datarate_mbps","peak_datarate_mbps"
    });
    for (const auto& g : gts) {
        w.writeRowV(g.name, g.lat_deg, g.lon_deg,
                    g.visible_pct, g.illuminated_pct,
                    g.avg_elevation_deg, g.max_elevation_deg,
                    g.avg_pass_duration_s, g.pass_count,
                    g.coverage_time_s, g.illuminated_time_s,
                    g.avg_datarate_mbps, g.min_datarate_mbps, g.peak_datarate_mbps);
    }
}

void OutputManager::writePassEvents(int run_id, const std::vector<PassEvent>& events)
{
    if (events.empty()) return;
    const std::string path = runDir(run_id) + "/pass_events.csv";
    CsvWriter w(path);
    w.writeHeader({"target","sat_id",
                   "aos_s","los_s","duration_s","max_elev_deg",
                   "aos_day","aos_hh","aos_mm","aos_ss",
                   "los_day","los_hh","los_mm","los_ss"});
    for (const auto& e : events) {
        auto decompose = [](double t_s, int& day, int& hh, int& mm, int& ss) {
            day = static_cast<int>(t_s / 86400.0);
            hh  = static_cast<int>(std::fmod(t_s, 86400.0) / 3600.0);
            mm  = static_cast<int>(std::fmod(t_s, 3600.0)  / 60.0);
            ss  = static_cast<int>(std::fmod(t_s, 60.0));
        };
        int ad, ah, am, as_, ld, lh, lm, ls;
        decompose(e.aos_s, ad, ah, am, as_);
        decompose(e.los_s, ld, lh, lm, ls);
        w.writeRowV(e.target_name, e.sat_id,
                    e.aos_s, e.los_s, e.duration_s, e.max_elev_deg,
                    ad, ah, am, as_,
                    ld, lh, lm, ls);
    }
}

void OutputManager::writeTrajectory(
    int run_id,
    const std::vector<SimulationEngine::OrbitalSnapshot>& snapshots)
{
    if (snapshots.empty()) return;
    const std::string path = runDir(run_id) + "/trajectory.csv";
    CsvWriter w(path);
    w.writeHeader({"time_s","sat_id","sma_km","eccentricity",
                   "inclination_deg","raan_deg","aop_deg",
                   "true_anomaly_deg","altitude_km"});
    for (const auto& s : snapshots) {
        w.writeRowV(s.time_s, s.sat_id, s.sma_km, s.eccentricity,
                    s.inclination_deg, s.raan_deg, s.aop_deg,
                    s.true_anomaly_deg, s.altitude_km);
    }
}

void OutputManager::writeOem(int run_id,
                             const std::vector<SimulationEngine::EphemerisSample>& samples,
                             double epoch_jd,
                             const std::string& object_name_prefix) {
    if (samples.empty()) return;

    std::map<int, std::vector<OemWriter::EphemerisPoint>> by_sat;
    for (const auto& s : samples) {
        by_sat[s.sat_id].push_back({s.time_s, s.position, s.velocity});
    }

    const std::string oem_dir = runDir(run_id) + "/oem";
    ensureDir(oem_dir);

    for (const auto& [sat_id, points] : by_sat) {
        OemWriter::Metadata meta;
        meta.object_name = object_name_prefix + "-" + std::to_string(sat_id);
        meta.object_id   = std::to_string(sat_id);
        meta.epoch_jd    = epoch_jd;

        char fname[64];
        std::snprintf(fname, sizeof(fname), "/sat_%04d.oem", sat_id);
        OemWriter::write(oem_dir + fname, meta, points);
    }
}

void OutputManager::writeConjunctions(int run_id,
                                      const std::vector<ConjunctionScreener::Event>& events) {
    if (events.empty()) return;
    const std::string path = runDir(run_id) + "/conjunctions.csv";
    CsvWriter w(path);
    w.writeHeader({"sat_i", "sat_j", "time_of_min_dist_s", "min_distance_km"});
    for (const auto& e : events) {
        w.writeRowV(e.sat_i, e.sat_j, e.time_of_min_dist_s, e.min_distance_km);
    }
}

void OutputManager::writeParetoFrontier(const std::vector<ConstellationResult>& frontier,
                                        const std::vector<std::string>& objectives) {
    const std::string path = exp_dir_ + "/pareto_frontier.csv";
    CsvWriter w(path);

    std::ostringstream obj_note;
    for (size_t i = 0; i < objectives.size(); ++i) {
        if (i > 0) obj_note << "; ";
        obj_note << objectives[i];
    }
    w.writeRow({"# objectives: " + obj_note.str()});

    w.writeHeader({
        "run_id","run_name",
        "altitude_km","inclination_deg",
        "total_satellites","planes","sats_per_plane",
        "coverage_pct","revisit_time_avg_s","revisit_time_max_s",
        "avg_drag_dv_ms","avg_sk_dv_ms","avg_annual_sk_dv_ms_per_year",
        "avg_sunlit_pct","avg_altitude_km","min_altitude_km",
        "deployment_dv_per_sat_ms","avg_datarate_mbps"
    });
    for (const auto& cr : frontier) writeSummaryRow(w, cr);
}

void OutputManager::finalize() {
    std::lock_guard lock(mutex_);
    summary_writer_->flush();
}
