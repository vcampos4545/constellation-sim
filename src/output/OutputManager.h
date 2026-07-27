#pragma once

#include "metrics/MetricsCollector.h"
#include "metrics/ConjunctionScreener.h"
#include "core/SimulationEngine.h"
#include "output/CsvWriter.h"
#include <string>
#include <filesystem>
#include <memory>
#include <mutex>

// OutputManager creates the per-run directory structure and writes CSV files.
//
// Directory layout:
//   <output_dir>/<experiment_name>/
//     run_0001/
//       summary.csv       — constellation-level result (one row)
//       satellites.csv    — per-satellite results
//     run_0002/ ...
//     experiment_summary.csv  — all runs combined (one row per run)
class OutputManager {
public:
    explicit OutputManager(const std::string& base_dir,
                           const std::string& experiment_name);

    // Write results for a single run. Thread-safe.
    void writeRun(int run_id,
                  const ConstellationResult& cr,
                  const std::vector<SatelliteResult>& sat_results,
                  const std::vector<GroundTargetResult>& gt_results = {});

    // Write orbital element trajectory snapshots (RAAN, inclination, etc.) to CSV.
    void writeTrajectory(int run_id,
                         const std::vector<SimulationEngine::OrbitalSnapshot>& snapshots);

    // Write per-pass AOS/LOS table to CSV. Only written when ground targets are configured.
    void writePassEvents(int run_id,
                         const std::vector<PassEvent>& events);

    // Write one CCSDS OEM ephemeris file per satellite present in `samples`,
    // into <run_dir>/oem/sat_<id>.oem.
    void writeOem(int run_id,
                 const std::vector<SimulationEngine::EphemerisSample>& samples,
                 double epoch_jd,
                 const std::string& object_name_prefix = "SAT");

    // Write pairwise conjunction events (sorted closest-first) to CSV.
    void writeConjunctions(int run_id,
                           const std::vector<ConjunctionScreener::Event>& events);

    // Write the non-dominated subset of a Monte Carlo sweep to
    // <experiment_dir>/pareto_frontier.csv (same columns as experiment_summary.csv).
    void writeParetoFrontier(const std::vector<ConstellationResult>& frontier,
                             const std::vector<std::string>& objectives);

    // Finalize: flush the experiment summary CSV.
    void finalize();

    std::string runDir(int run_id) const;
    std::string experimentDir() const { return exp_dir_; }

private:
    std::string exp_dir_;
    std::string summary_path_;
    std::mutex  mutex_;

    std::unique_ptr<CsvWriter> summary_writer_;
    bool summary_header_written_{false};

    static std::string runDirName(int run_id);
    void ensureDir(const std::string& path);
    void writeSummaryRow(CsvWriter& w, const ConstellationResult& cr);
    void writeSatelliteCsv(const std::string& path,
                           const std::vector<SatelliteResult>& sats);
    void writeGroundTargetCsv(const std::string& path,
                               const std::vector<GroundTargetResult>& gts);
};
