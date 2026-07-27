#include "orbit/TLE.h"
#include "core/math/Constants.h"
#include <cmath>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

// Standard Gregorian calendar date -> Julian Date (Meeus Ch. 7).
// Duplicated locally (also present in ConfigLoader.cpp) to keep TLE parsing
// self-contained and independent of the JSON config loader.
double calendarToJD(int year, int month, int day) {
    if (month <= 2) { year -= 1; month += 12; }
    int A = year / 100;
    int B = 2 - A + A / 4;
    return std::floor(365.25 * (year + 4716))
         + std::floor(30.6001 * (month + 1))
         + day + B - 1524.5;
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// Parse a TLE-style "assumed decimal point" exponential field, e.g. the
// BSTAR / second-derivative-of-mean-motion columns: " 66816-4" -> 0.66816e-4.
// Field is exactly 8 characters: [sign][5 digits][exp sign][exp digit].
double parseAssumedDecimalExp(const std::string& field) {
    if (field.size() < 8) throw std::runtime_error("TLE: malformed exponential field: '" + field + "'");

    const char mantissa_sign = field[0];
    const std::string digits = field.substr(1, 5);
    const char exp_sign = field[6];
    const char exp_digit = field[7];

    for (char c : digits) {
        if (!std::isdigit(static_cast<unsigned char>(c)))
            throw std::runtime_error("TLE: malformed exponential mantissa: '" + field + "'");
    }
    if (!std::isdigit(static_cast<unsigned char>(exp_digit)))
        throw std::runtime_error("TLE: malformed exponential field: '" + field + "'");

    double mantissa = 0.0;
    if (!digits.empty() && digits.find_first_not_of('0') != std::string::npos) {
        mantissa = std::stod("0." + digits);
    }
    const double sign = (mantissa_sign == '-') ? -1.0 : 1.0;
    const int exponent = (exp_digit - '0') * ((exp_sign == '-') ? -1 : 1);
    return sign * mantissa * std::pow(10.0, exponent);
}

double parseDecimal(const std::string& field) {
    std::string s = trim(field);
    if (s.empty()) return 0.0;
    // Some fields use a leading '.' with no leading zero ("-.00073094");
    // std::stod handles that directly.
    return std::stod(s);
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line)) {
        // Strip trailing CR if the file has CRLF endings.
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!trim(line).empty()) lines.push_back(line);
    }
    return lines;
}

} // namespace

TLE TLE::parse(const std::string& line1, const std::string& line2, const std::string& name) {
    // Column 69 (checksum) is not required/validated -- only columns up to
    // 68 are actually parsed, so accept lines that omit the checksum digit.
    if (line1.size() < 68 || line2.size() < 68)
        throw std::runtime_error("TLE: lines must be at least 68 columns wide");
    if (line1[0] != '1' || line2[0] != '2')
        throw std::runtime_error("TLE: expected line numbers '1' and '2'");

    TLE t;
    t.name = trim(name);
    t.satellite_number = std::stoi(trim(line1.substr(2, 5)));

    // --- Epoch ---
    const int yy = std::stoi(trim(line1.substr(18, 2)));
    const int year = (yy < 57) ? (2000 + yy) : (1900 + yy);
    const double doy = parseDecimal(line1.substr(20, 12));
    t.epoch_jd = calendarToJD(year, 1, 1) + (doy - 1.0);

    // --- Line 1 drag / mean-motion derivative terms ---
    t.ndot_over_2  = parseDecimal(line1.substr(33, 10)) * Constants::TWO_PI / (1440.0 * 1440.0);
    t.nddot_over_6 = parseAssumedDecimalExp(line1.substr(44, 8)) * Constants::TWO_PI / (1440.0 * 1440.0 * 1440.0);
    t.bstar        = parseAssumedDecimalExp(line1.substr(53, 8));
    t.element_set_number = std::stoi(trim(line1.substr(64, 4)));

    // --- Line 2 elements ---
    t.inclination_rad   = parseDecimal(line2.substr(8, 8))  * Constants::DEG2RAD;
    t.raan_rad           = parseDecimal(line2.substr(17, 8)) * Constants::DEG2RAD;

    const std::string ecc_digits = trim(line2.substr(26, 7));
    t.eccentricity = ecc_digits.empty() ? 0.0 : std::stod("0." + ecc_digits);

    t.arg_perigee_rad    = parseDecimal(line2.substr(34, 8)) * Constants::DEG2RAD;
    t.mean_anomaly_rad   = parseDecimal(line2.substr(43, 8)) * Constants::DEG2RAD;

    const double mean_motion_rev_day = parseDecimal(line2.substr(52, 11));
    t.mean_motion_rad_min = mean_motion_rev_day * Constants::TWO_PI / 1440.0;

    t.rev_number_at_epoch = std::stoi(trim(line2.substr(63, 5)));

    return t;
}

TLE TLE::parseBlock(const std::string& text) {
    const auto lines = splitLines(text);
    if (lines.size() == 2) {
        return parse(lines[0], lines[1]);
    }
    if (lines.size() >= 3) {
        return parse(lines[1], lines[2], lines[0]);
    }
    throw std::runtime_error("TLE: expected 2 or 3 non-empty lines");
}

double TLE::period_s() const {
    return Constants::TWO_PI / mean_motion_rad_min * 60.0;
}
