#include "core/CityData.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cctype>

namespace CityData {

namespace {
std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

std::vector<City> loadCities() {
    std::vector<City> cities;
    const std::string path = "resources/data/major_cities.json";
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Warning: could not open " << path
                  << " -- city-name ground targets will not resolve.\n";
        return cities;
    }

    nlohmann::json j;
    try {
        file >> j;
    } catch (const std::exception& e) {
        std::cerr << "Warning: failed to parse " << path << ": " << e.what() << "\n";
        return cities;
    }

    for (const auto& entry : j) {
        City c;
        c.name    = entry.value("name", "");
        c.lat_deg = entry.value("lat_deg", 0.0);
        c.lon_deg = entry.value("lon_deg", 0.0);
        if (!c.name.empty()) cities.push_back(std::move(c));
    }
    return cities;
}
} // namespace

const std::vector<City>& all() {
    static const std::vector<City> cities = loadCities();
    return cities;
}

const City* find(const std::string& name) {
    const std::string needle = toLower(name);
    for (const auto& c : all()) {
        if (toLower(c.name) == needle) return &c;
    }
    return nullptr;
}

} // namespace CityData
