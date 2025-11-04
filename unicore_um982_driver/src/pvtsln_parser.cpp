#include "unicore_um982_driver/pvtsln_data.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace unicore_um982_driver
{
namespace
{

std::string trim(const std::string &input)
{
    const auto first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

std::vector<std::string> splitString(const std::string &str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter))
    {
        tokens.push_back(trim(token));
    }

    return tokens;
}

double parseDouble(const std::string &value)
{
    const std::string trimmed = trim(value);
    if (trimmed.empty())
    {
        return 0.0;
    }
    return std::stod(trimmed);
}

int32_t parseInt32(const std::string &value)
{
    const std::string trimmed = trim(value);
    if (trimmed.empty())
    {
        return 0;
    }
    return static_cast<int32_t>(std::stol(trimmed, nullptr, 10));
}

uint32_t parseUInt32(const std::string &value)
{
    std::string trimmed = trim(value);
    if (trimmed.empty())
    {
        return 0U;
    }

    int base = 10;
    if (trimmed.rfind("0x", 0) == 0 || trimmed.rfind("0X", 0) == 0)
    {
        trimmed = trimmed.substr(2);
        base = 16;
    }
    else if (trimmed.find_first_of("ABCDEFabcdef") != std::string::npos)
    {
        base = 16;
    }

    return static_cast<uint32_t>(std::stoul(trimmed, nullptr, base));
}

uint16_t parseUInt16(const std::string &value)
{
    const std::string trimmed = trim(value);
    if (trimmed.empty())
    {
        return 0U;
    }
    return static_cast<uint16_t>(std::stoul(trimmed, nullptr, 10));
}

uint8_t parseUInt8(const std::string &value)
{
    const std::string trimmed = trim(value);
    if (trimmed.empty())
    {
        return 0U;
    }
    return static_cast<uint8_t>(std::stoul(trimmed, nullptr, 10));
}

constexpr size_t kRequiredBodyFields = 35;  // 34 data fields + PRN count

}  // namespace

bool parsePVTSLN(const std::string &line, PVTSLNData &data, std::string *error_message)
{
    data = PVTSLNData();
    data.is_valid = false;

    auto setError = [&](const std::string &message) {
        if (error_message)
        {
            *error_message = message;
        }
        std::cerr << message << std::endl;
    };

    if (line.find("PVTSLN") == std::string::npos)
    {
        return false;
    }

    try
    {
        std::string msg = trim(line);

        // Remove checksum if present and store it
        size_t checksum_pos = msg.find('*');
        if (checksum_pos != std::string::npos)
        {
            std::string checksum_str = msg.substr(checksum_pos + 1);
            data.crc = parseUInt32(checksum_str);
            msg = msg.substr(0, checksum_pos);
        }
        else
        {
            data.crc = 0U;
        }

        // Remove leading # if present
        if (!msg.empty() && msg.front() == '#')
        {
            msg.erase(0, 1);
        }

        // Split the message into header and body parts
        size_t semicolon_pos = msg.find(';');
        if (semicolon_pos == std::string::npos)
        {
            setError("No semicolon found in PVTSLN message");
            return false;
        }

        std::string header_part = msg.substr(0, semicolon_pos);
        std::string body_part = msg.substr(semicolon_pos + 1);

        // Parse header part (comma-separated)
        std::vector<std::string> header_fields = splitString(header_part, ',');
        if (header_fields.size() < 10)
        {
            setError("Insufficient header fields in PVTSLN message");
            return false;
        }

        data.message_id = header_fields[0];
        data.sequence_number = parseInt32(header_fields[1]);
        data.port_id = header_fields[2];
        data.time_status = header_fields[3];
        data.week = parseInt32(header_fields[4]);
        data.time_of_week = parseDouble(header_fields[5]);
        data.receiver_status = parseUInt32(header_fields[6]);
        data.reserved = parseUInt32(header_fields[7]);
        data.solution_status = parseInt32(header_fields[8]);
        data.position_type = parseInt32(header_fields[9]);

        // Parse body part (comma-separated)
        std::vector<std::string> body_fields = splitString(body_part, ',');
        if (body_fields.size() < kRequiredBodyFields)
        {
            setError("Insufficient body fields in PVTSLN message, got " + std::to_string(body_fields.size()));
            return false;
        }

        size_t field_idx = 0;

        data.bestpos_type = body_fields[field_idx++];
        data.bestpos_height = parseDouble(body_fields[field_idx++]);
        data.bestpos_latitude = parseDouble(body_fields[field_idx++]);
        data.bestpos_longitude = parseDouble(body_fields[field_idx++]);
        data.bestpos_height_std = parseDouble(body_fields[field_idx++]);
        data.bestpos_latitude_std = parseDouble(body_fields[field_idx++]);
        data.bestpos_longitude_std = parseDouble(body_fields[field_idx++]);
        data.bestpos_diff_age = parseDouble(body_fields[field_idx++]);
        data.age_of_corrections = data.bestpos_diff_age;

        data.psrpos_type = body_fields[field_idx++];
        data.psrpos_height = parseDouble(body_fields[field_idx++]);
        data.psrpos_latitude = parseDouble(body_fields[field_idx++]);
        data.psrpos_longitude = parseDouble(body_fields[field_idx++]);

        data.undulation = parseDouble(body_fields[field_idx++]);

        data.bestpos_svs = parseUInt8(body_fields[field_idx++]);
        data.bestpos_solnsvs = parseUInt8(body_fields[field_idx++]);
        data.psrpos_svs = parseUInt8(body_fields[field_idx++]);
        data.psrpos_solnsvs = parseUInt8(body_fields[field_idx++]);

        data.psrvel_north = parseDouble(body_fields[field_idx++]);
        data.psrvel_east = parseDouble(body_fields[field_idx++]);
        data.psrvel_ground = parseDouble(body_fields[field_idx++]);

        data.heading_type = body_fields[field_idx++];
        data.heading_length = parseDouble(body_fields[field_idx++]);
        data.heading_degree = parseDouble(body_fields[field_idx++]);
        data.heading_pitch = parseDouble(body_fields[field_idx++]);

        data.heading_trackedsvs = parseUInt8(body_fields[field_idx++]);
        data.heading_solnsvs = parseUInt8(body_fields[field_idx++]);
        data.heading_ggl1 = parseUInt8(body_fields[field_idx++]);
        data.heading_ggl1l2 = parseUInt8(body_fields[field_idx++]);

        data.gdop = parseDouble(body_fields[field_idx++]);
        data.pdop = parseDouble(body_fields[field_idx++]);
        data.hdop = parseDouble(body_fields[field_idx++]);
        data.htdop = parseDouble(body_fields[field_idx++]);
        data.tdop = parseDouble(body_fields[field_idx++]);
        data.elevation_cutoff = parseDouble(body_fields[field_idx++]);

        data.prn_count = parseUInt16(body_fields[field_idx++]);

        data.prn_list.clear();
        const size_t remaining_fields = body_fields.size() - field_idx;
        const size_t prn_to_read = std::min(static_cast<size_t>(data.prn_count), remaining_fields);
        data.prn_list.reserve(prn_to_read);
        for (size_t i = 0; i < prn_to_read; ++i)
        {
            data.prn_list.push_back(parseUInt16(body_fields[field_idx++]));
        }

        // Populate convenience mirrors used by rest of the driver
        data.position_status = data.bestpos_type;
        data.latitude = data.bestpos_latitude;
        data.longitude = data.bestpos_longitude;
        data.altitude = data.bestpos_height;
        data.num_satellites_tracked = data.bestpos_svs;
        data.num_satellites_used = data.bestpos_solnsvs;
        data.sigma_latitude = data.bestpos_latitude_std;
        data.sigma_longitude = data.bestpos_longitude_std;
        data.sigma_altitude = data.bestpos_height_std;
        data.heading = data.heading_degree;
        data.velocity_north = data.psrvel_north;
        data.velocity_east = data.psrvel_east;
        data.velocity_up = 0.0;  // PVTSLN does not report vertical velocity
        data.timestamp = data.time_of_week;

        data.is_valid = true;
        if (error_message)
        {
            error_message->clear();
        }
        return true;
    }
    catch (const std::exception &e)
    {
        setError(std::string("Error parsing PVTSLN message: ") + e.what());
        data.is_valid = false;
        return false;
    }
}

}  // namespace unicore_um982_driver
