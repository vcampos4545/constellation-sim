#include "output/OemWriter.h"
#include "core/TimeUtils.h"
#include <fstream>
#include <stdexcept>
#include <cstdio>

namespace OemWriter {

void write(const std::string& path,
          const Metadata& meta,
          const std::vector<EphemerisPoint>& points) {
    if (points.empty()) return;

    std::ofstream f(path);
    if (!f.is_open())
        throw std::runtime_error("OemWriter: cannot open " + path);

    const std::string start_time = TimeUtils::jdToISO8601(meta.epoch_jd + points.front().time_s / 86400.0);
    const std::string stop_time  = TimeUtils::jdToISO8601(meta.epoch_jd + points.back().time_s  / 86400.0);
    const std::string creation   = TimeUtils::jdToISO8601(meta.epoch_jd);

    f << "CCSDS_OEM_VERS = 2.0\n";
    f << "CREATION_DATE = " << creation << "\n";
    f << "ORIGINATOR = ConstellationSim\n";
    f << "\n";
    f << "META_START\n";
    f << "OBJECT_NAME = " << (meta.object_name.empty() ? "UNKNOWN" : meta.object_name) << "\n";
    f << "OBJECT_ID = " << (meta.object_id.empty() ? "0000-000A" : meta.object_id) << "\n";
    f << "CENTER_NAME = " << meta.center_name << "\n";
    f << "REF_FRAME = " << meta.ref_frame << "\n";
    f << "TIME_SYSTEM = UTC\n";
    f << "START_TIME = " << start_time << "\n";
    f << "STOP_TIME = " << stop_time << "\n";
    f << "META_STOP\n";
    f << "\n";

    char buf[256];
    for (const auto& p : points) {
        const std::string ts = TimeUtils::jdToISO8601(meta.epoch_jd + p.time_s / 86400.0);
        // Position/velocity in km, km/s per the OEM standard.
        std::snprintf(buf, sizeof(buf), "%s %.8f %.8f %.8f %.8f %.8f %.8f",
                      ts.c_str(),
                      p.position_m.x / 1000.0, p.position_m.y / 1000.0, p.position_m.z / 1000.0,
                      p.velocity_ms.x / 1000.0, p.velocity_ms.y / 1000.0, p.velocity_ms.z / 1000.0);
        f << buf << "\n";
    }
}

} // namespace OemWriter
