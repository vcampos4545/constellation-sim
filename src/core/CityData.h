#pragma once
#include <string>
#include <vector>

// Lookup table of major world cities (lat/lon), loaded from
// resources/data/major_cities.json. Lets a scenario reference a ground
// target by city name instead of hand-typing coordinates.
namespace CityData {

struct City {
    std::string name;
    double      lat_deg{0.0};
    double      lon_deg{0.0};
};

// Loads and caches resources/data/major_cities.json on first call. Returns
// an empty list (and prints a warning once) if the file can't be found --
// callers should treat that as "no cities available", not a hard error.
const std::vector<City>& all();

// Case-insensitive lookup by name. Returns nullptr if not found.
const City* find(const std::string& name);

} // namespace CityData
