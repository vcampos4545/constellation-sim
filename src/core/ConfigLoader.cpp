#include "core/ConfigLoader.h"
#include "core/CityData.h"
#include <fstream>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cctype>

using json = nlohmann::json;

static double getOrDefault(const json& j, const std::string& key, double def) {
    return j.contains(key) ? j.at(key).get<double>() : def;
}
static int getOrDefaultI(const json& j, const std::string& key, int def) {
    return j.contains(key) ? j.at(key).get<int>() : def;
}
static bool getOrDefaultB(const json& j, const std::string& key, bool def) {
    return j.contains(key) ? j.at(key).get<bool>() : def;
}
static std::string getOrDefaultS(const json& j, const std::string& key, const std::string& def) {
    return j.contains(key) ? j.at(key).get<std::string>() : def;
}

// Convert calendar date/time to Julian Date (Meeus Ch. 7).
static double calendarToJD(int year, int month, int day,
                            int hour = 0, int minute = 0, double second = 0.0) {
    if (month <= 2) { year -= 1; month += 12; }
    int A = year / 100;
    int B = 2 - A + A / 4;
    double day_frac = day + (hour + minute / 60.0 + second / 3600.0) / 24.0;
    return std::floor(365.25  * (year  + 4716))
         + std::floor(30.6001 * (month + 1))
         + day_frac + B - 1524.5;
}

double ConfigLoader::epochStringToJD(const std::string& str) {
    std::string up(str);
    for (auto& c : up) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

    if (up == "J2000" || up == "J2000.0") return 2451545.0;

    if (up == "TODAY" || up == "NOW") {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm utc{};
#ifdef _WIN32
        gmtime_s(&utc, &t);
#else
        gmtime_r(&t, &utc);
#endif
        return calendarToJD(utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                            utc.tm_hour, utc.tm_min, utc.tm_sec);
    }

    // ISO 8601: "YYYY-MM-DD" or "YYYY-MM-DDTHH:MM:SS" or "YYYY-MM-DD HH:MM:SS"
    try {
        int year = 0, month = 0, day = 0, hour = 0, minute = 0;
        double second = 0.0;
        char sep = 0;
        std::istringstream ss(str);
        if (!(ss >> year >> sep >> month >> sep >> day))
            throw std::runtime_error("bad date");
        char peek = static_cast<char>(ss.peek());
        if (peek == 'T' || peek == ' ') {
            ss >> sep >> hour >> sep >> minute >> sep >> second;
        }
        return calendarToJD(year, month, day, hour, minute, second);
    } catch (...) {
        throw std::runtime_error("Cannot parse epoch string: '" + str + "'. "
            "Use ISO 8601 (e.g. \"2025-01-01T00:00:00\"), \"J2000\", or \"today\".");
    }
}

WalkerConfig ConfigLoader::parseWalker(const json& j) {
    WalkerConfig c;
    c.altitude_km        = getOrDefault(j, "altitude_km",        c.altitude_km);
    c.inclination_deg    = getOrDefault(j, "inclination_deg",    c.inclination_deg);
    c.total_satellites   = getOrDefaultI(j, "total_satellites",  c.total_satellites);
    c.planes             = getOrDefaultI(j, "planes",            c.planes);
    c.phasing_factor     = getOrDefaultI(j, "phasing_factor",    c.phasing_factor);
    c.eccentricity       = getOrDefault(j, "eccentricity",       c.eccentricity);
    c.arg_of_perigee_deg = getOrDefault(j, "arg_of_perigee_deg", c.arg_of_perigee_deg);

    // sats_per_plane is an alternative way to specify
    if (j.contains("sats_per_plane") && !j.contains("total_satellites")) {
        int spp = j.at("sats_per_plane").get<int>();
        c.total_satellites = c.planes * spp;
    }
    return c;
}

PhysicalProperties ConfigLoader::parseSatellite(const json& j) {
    PhysicalProperties p;
    p.mass_kg          = getOrDefault(j, "mass_kg",          p.mass_kg);
    p.drag_coefficient = getOrDefault(j, "drag_coefficient", p.drag_coefficient);
    p.drag_area_m2     = getOrDefault(j, "drag_area_m2",     p.drag_area_m2);
    p.reflectivity     = getOrDefault(j, "reflectivity",     p.reflectivity);
    p.srp_area_m2      = getOrDefault(j, "srp_area_m2",      p.srp_area_m2);
    p.solar_array_area_m2    = getOrDefault(j, "solar_array_area_m2",    p.solar_array_area_m2);
    p.solar_array_efficiency = getOrDefault(j, "solar_array_efficiency", p.solar_array_efficiency);
    return p;
}

StationKeepingConfig ConfigLoader::parseStationKeeping(const json& j) {
    StationKeepingConfig cfg;
    cfg.enabled              = getOrDefaultB(j, "enabled",              cfg.enabled);
    cfg.altitude_deadband_km = getOrDefault(j,  "altitude_deadband_km", cfg.altitude_deadband_km);
    cfg.check_interval_s     = getOrDefault(j,  "check_interval_s",     cfg.check_interval_s);
    return cfg;
}

PhysicsConfig ConfigLoader::parsePhysics(const json& j) {
    PhysicsConfig p;
    p.gravity      = getOrDefaultB(j, "gravity",      p.gravity);
    p.j2           = getOrDefaultB(j, "j2",           p.j2);
    p.j3           = getOrDefaultB(j, "j3",           p.j3);
    p.j4           = getOrDefaultB(j, "j4",           p.j4);
    p.drag         = getOrDefaultB(j, "drag",         p.drag);
    p.srp          = getOrDefaultB(j, "srp",          p.srp);
    p.moon_gravity = getOrDefaultB(j, "moon",         p.moon_gravity);
    p.sun_gravity  = getOrDefaultB(j, "sun_gravity",  p.sun_gravity);
    return p;
}

MetricsConfig ConfigLoader::parseMetrics(const json& j) {
    MetricsConfig m;
    if (j.contains("coverage")) {
        const auto& cj = j.at("coverage");
        if (cj.is_object()) {
            m.coverage.enabled             = getOrDefaultB(cj, "enabled",             m.coverage.enabled);
            m.coverage.grid_resolution_deg = getOrDefault(cj, "grid_resolution_deg", m.coverage.grid_resolution_deg);
            m.coverage.min_elevation_deg   = getOrDefault(cj, "min_elevation_deg",   m.coverage.min_elevation_deg);
            m.coverage.sample_interval_s   = getOrDefault(cj, "sample_interval_s",   m.coverage.sample_interval_s);
        } else {
            m.coverage.enabled = cj.get<bool>();
        }
    }
    m.sunlight = getOrDefaultB(j, "sunlight", m.sunlight);
    m.drag     = getOrDefaultB(j, "drag",     m.drag);
    m.delta_v  = getOrDefaultB(j, "delta_v",  m.delta_v);
    m.revisit  = getOrDefaultB(j, "revisit",  m.revisit);
    m.links    = getOrDefaultB(j, "links",    m.links);

    if (j.contains("conjunction")) {
        const auto& cj = j.at("conjunction");
        m.conjunction.enabled           = getOrDefaultB(cj, "enabled",           m.conjunction.enabled);
        m.conjunction.threshold_km      = getOrDefault(cj,  "threshold_km",      m.conjunction.threshold_km);
        m.conjunction.sample_interval_s = getOrDefault(cj,  "sample_interval_s", m.conjunction.sample_interval_s);
    }
    if (j.contains("link_budget")) {
        const auto& lj = j.at("link_budget");
        m.link_budget.enabled          = getOrDefaultB(lj, "enabled",          m.link_budget.enabled);
        m.link_budget.frequency_ghz    = getOrDefault(lj,  "frequency_ghz",    m.link_budget.frequency_ghz);
        m.link_budget.eirp_dbw         = getOrDefault(lj,  "eirp_dbw",         m.link_budget.eirp_dbw);
        m.link_budget.gt_dbk           = getOrDefault(lj,  "gt_dbk",           m.link_budget.gt_dbk);
        m.link_budget.bandwidth_mhz    = getOrDefault(lj,  "bandwidth_mhz",    m.link_budget.bandwidth_mhz);
        m.link_budget.other_losses_db  = getOrDefault(lj,  "other_losses_db",  m.link_budget.other_losses_db);
    }
    return m;
}

SimConfig ConfigLoader::parseSimConfig(const json& j) {
    SimConfig cfg;

    if (j.contains("simulation")) {
        const auto& s = j.at("simulation");
        cfg.name          = getOrDefaultS(s, "name",         cfg.name);
        cfg.duration_days = getOrDefault(s,  "duration_days", cfg.duration_days);
        cfg.timestep_s    = getOrDefault(s,  "timestep_s",    cfg.timestep_s);
        if (s.contains("epoch")) {
            cfg.epoch_jd = ConfigLoader::epochStringToJD(
                s.at("epoch").get<std::string>());
        } else {
            cfg.epoch_jd = getOrDefault(s, "epoch_jd", cfg.epoch_jd);
        }
    }
    if (j.contains("constellation")) {
        const auto& cj = j.at("constellation");
        cfg.constellation_type = getOrDefaultS(cj, "type", "walker");

        if (cfg.constellation_type == "custom") {
            if (cj.contains("satellites")) {
                for (const auto& sj : cj.at("satellites")) {
                    SatelliteSpec s;
                    s.plane_id           = getOrDefaultI(sj, "plane",              0);
                    s.seat_id            = getOrDefaultI(sj, "seat",               0);
                    s.sma_km             = getOrDefault(sj,  "sma_km",             7000.0);
                    s.eccentricity       = getOrDefault(sj,  "eccentricity",       0.0);
                    s.inclination_deg    = getOrDefault(sj,  "inclination_deg",    0.0);
                    s.raan_deg           = getOrDefault(sj,  "raan_deg",           0.0);
                    s.arg_of_perigee_deg = getOrDefault(sj,  "arg_of_perigee_deg", 0.0);
                    s.true_anomaly_deg   = getOrDefault(sj,  "true_anomaly_deg",   0.0);
                    cfg.explicit_satellites.push_back(s);
                }
            }
            // Populate WalkerConfig metadata fields so MetricsCollector output is meaningful.
            cfg.constellation.total_satellites = static_cast<int>(cfg.explicit_satellites.size());
            int max_plane = 0;
            for (const auto& s : cfg.explicit_satellites)
                if (s.plane_id > max_plane) max_plane = s.plane_id;
            cfg.constellation.planes         = max_plane + 1;
            if (!cfg.explicit_satellites.empty()) {
                cfg.constellation.inclination_deg = cfg.explicit_satellites[0].inclination_deg;
                cfg.constellation.altitude_km     = cfg.explicit_satellites[0].sma_km - 6378.137;
                cfg.constellation.eccentricity    = cfg.explicit_satellites[0].eccentricity;
                cfg.constellation.arg_of_perigee_deg =
                    cfg.explicit_satellites[0].arg_of_perigee_deg;
            }
        } else {
            cfg.constellation = parseWalker(cj);
        }
    }
    if (j.contains("satellite"))       cfg.satellite     = parseSatellite(j.at("satellite"));
    if (j.contains("forces"))         cfg.physics       = parsePhysics(j.at("forces"));
    if (j.contains("metrics"))        cfg.metrics       = parseMetrics(j.at("metrics"));
    if (j.contains("station_keeping")) cfg.station_keeping = parseStationKeeping(j.at("station_keeping"));

    if (j.contains("ground_targets")) {
        for (const auto& t : j.at("ground_targets")) {
            GroundTarget gt;
            if (t.contains("city")) {
                const std::string city_name = t.at("city").get<std::string>();
                const CityData::City* city = CityData::find(city_name);
                if (!city) {
                    throw std::runtime_error("Unknown city in ground_targets: '" + city_name +
                        "' (see resources/data/major_cities.json for known names)");
                }
                gt.name    = getOrDefaultS(t, "name", city->name);
                gt.lat_deg = city->lat_deg;
                gt.lon_deg = city->lon_deg;
            } else {
                gt.name    = getOrDefaultS(t, "name",    "");
                gt.lat_deg = getOrDefault(t,  "lat_deg", 0.0);
                gt.lon_deg = getOrDefault(t,  "lon_deg", 0.0);
            }
            gt.description = getOrDefaultS(t, "description", "");
            cfg.ground_targets.push_back(std::move(gt));
        }
    }

    if (j.contains("output")) {
        const auto& o = j.at("output");
        cfg.output_directory              = getOrDefaultS(o, "directory",  cfg.output_directory);
        cfg.run_name                      = getOrDefaultS(o, "run_name",   cfg.run_name);
        cfg.trajectory_sample_interval_s  = getOrDefault(o,  "trajectory_sample_interval_s", 0.0);
        cfg.export_oem                    = getOrDefaultB(o, "export_oem", cfg.export_oem);
        if (o.contains("oem_satellite_ids")) {
            cfg.oem_satellite_ids.clear();
            for (const auto& id : o.at("oem_satellite_ids"))
                cfg.oem_satellite_ids.push_back(id.get<int>());
        }
    }
    return cfg;
}

SimConfig ConfigLoader::loadSimConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open config file: " + path);
    json j;
    file >> j;
    return parseSimConfig(j);
}

MCConfig ConfigLoader::loadMCConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open MC config file: " + path);
    json j;
    file >> j;

    MCConfig mc;
    if (j.contains("experiment")) {
        const auto& e = j.at("experiment");
        mc.name     = getOrDefaultS(e, "name",     mc.name);
        mc.runs     = getOrDefaultI(e, "runs",     mc.runs);
        mc.sampling = getOrDefaultS(e, "sampling", mc.sampling);
        mc.threads  = getOrDefaultI(e, "threads",  mc.threads);
    }

    if (j.contains("base_config")) mc.base_config = parseSimConfig(j.at("base_config"));

    if (j.contains("sweep")) {
        for (auto& [key, val] : j.at("sweep").items()) {
            MCParameterRange range;
            range.name = key;
            if (val.is_array()) {
                for (auto& v : val) range.values.push_back(v.get<double>());
            }
            mc.parameters.push_back(std::move(range));
        }
    }

    if (j.contains("pareto_objectives")) {
        for (const auto& v : j.at("pareto_objectives"))
            mc.pareto_objectives.push_back(v.get<std::string>());
    }

    if (j.contains("output")) {
        const auto& o = j.at("output");
        mc.output_directory = getOrDefaultS(o, "directory",       mc.output_directory);
        mc.experiment_name  = getOrDefaultS(o, "experiment_name", mc.experiment_name);
    }
    return mc;
}

void ConfigLoader::applyParameter(SimConfig& cfg, const std::string& name, double value) {
    // Maps flat parameter names used in MC sweeps to struct fields.
    if      (name == "altitude_km")       cfg.constellation.altitude_km        = value;
    else if (name == "inclination_deg")   cfg.constellation.inclination_deg    = value;
    else if (name == "planes")            cfg.constellation.planes             = static_cast<int>(value);
    else if (name == "sats_per_plane") {
        cfg.constellation.total_satellites = cfg.constellation.planes * static_cast<int>(value);
    }
    else if (name == "total_satellites")  cfg.constellation.total_satellites   = static_cast<int>(value);
    else if (name == "phasing_factor")    cfg.constellation.phasing_factor     = static_cast<int>(value);
    else if (name == "mass_kg")           cfg.satellite.mass_kg                = value;
    else if (name == "drag_coefficient")  cfg.satellite.drag_coefficient       = value;
    else if (name == "drag_area_m2")      cfg.satellite.drag_area_m2           = value;
    else if (name == "timestep_s")        cfg.timestep_s                       = value;
    else if (name == "duration_days")     cfg.duration_days                    = value;
    else if (name == "min_elevation_deg") cfg.metrics.coverage.min_elevation_deg = value;
}
