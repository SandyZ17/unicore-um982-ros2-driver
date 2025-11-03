#include "unicore_um982_driver/pvtsln_data.hpp"
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>

namespace unicore_um982_driver
{

std::vector<std::string> splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

bool parsePVTSLN(const std::string& line, PVTSLNData& data)
{
    // Reset data validity
    data.is_valid = false;
    
    // Check if line contains PVTSLN
    if (line.find("PVTSLN") == std::string::npos) {
        return false;
    }
    
    try {
        // Remove checksum if present (everything after *)
        std::string msg = line;
        size_t checksum_pos = msg.find('*');
        if (checksum_pos != std::string::npos) {
            msg = msg.substr(0, checksum_pos);
        }
        
        // Remove leading # if present
        if (!msg.empty() && msg[0] == '#') {
            msg = msg.substr(1);
        }
        
        // Split the message into header and body parts
        size_t semicolon_pos = msg.find(';');
        if (semicolon_pos == std::string::npos) {
            std::cerr << "No semicolon found in PVTSLN message" << std::endl;
            return false;
        }
        
        std::string header_part = msg.substr(0, semicolon_pos);
        std::string body_part = msg.substr(semicolon_pos + 1);
        
        // Parse header part (comma-separated)
        std::vector<std::string> header_fields = splitString(header_part, ',');
        if (header_fields.size() < 9) {
            std::cerr << "Insufficient header fields in PVTSLN message" << std::endl;
            return false;
        }
        
        data.message_id = header_fields[0];
        data.sequence_num = std::stoi(header_fields[1]);
        data.gnss_mode = header_fields[2];
        data.time_status = header_fields[3];
        data.week = std::stoi(header_fields[4]);
        data.time_of_week = std::stod(header_fields[5]);
        
        // Parse body part (comma-separated)
        std::vector<std::string> body_fields = splitString(body_part, ',');
        if (body_fields.size() < 20) {
            std::cerr << "Insufficient body fields in PVTSLN message, got " << body_fields.size() << std::endl;
            return false;
        }
        
        // Parse the main fields based on observed message structure
        int field_idx = 0;
        
        // Primary position solution
        data.position_status = body_fields[field_idx++];          // 0: SINGLE
        data.heading = std::stod(body_fields[field_idx++]);       // 1: heading (deg)
        data.latitude = std::stod(body_fields[field_idx++]);      // 2: latitude (deg)
        data.longitude = std::stod(body_fields[field_idx++]);     // 3: longitude (deg)
        data.altitude = std::stod(body_fields[field_idx++]);      // 4: altitude (m)
        data.undulation = std::stod(body_fields[field_idx++]);    // 5: undulation (m)
        data.velocity_north = std::stod(body_fields[field_idx++]); // 6: vel_north (m/s)
        data.velocity_east = std::stod(body_fields[field_idx++]);  // 7: vel_east (m/s)
        
        // Skip dual antenna status (8)
        data.dual_antenna_status = body_fields[field_idx++];      // 8: dual antenna status
        data.dual_antenna_heading = std::stod(body_fields[field_idx++]);  // 9: dual antenna heading
        data.dual_antenna_latitude = std::stod(body_fields[field_idx++]); // 10: dual antenna lat
        data.dual_antenna_longitude = std::stod(body_fields[field_idx++]); // 11: dual antenna lon
        data.dual_antenna_altitude = std::stod(body_fields[field_idx++]);  // 12: dual antenna alt
        
        // Parse satellite counts and other data
        if (field_idx < body_fields.size()) {
            data.num_satellites_tracked = std::stoi(body_fields[field_idx++]);  // 13
        }
        if (field_idx < body_fields.size()) {
            data.num_satellites_used_l1 = std::stoi(body_fields[field_idx++]);  // 14
        }
        if (field_idx < body_fields.size()) {
            data.num_satellites_used_l2 = std::stoi(body_fields[field_idx++]);  // 15
        }
        if (field_idx < body_fields.size()) {
            field_idx++; // Skip field 16
        }
        
        // Parse standard deviations for position
        if (field_idx < body_fields.size()) {
            data.sigma_latitude = std::stod(body_fields[field_idx++]);   // 17
        }
        if (field_idx < body_fields.size()) {
            data.sigma_longitude = std::stod(body_fields[field_idx++]);  // 18
        }
        if (field_idx < body_fields.size()) {
            data.sigma_altitude = std::stod(body_fields[field_idx++]);   // 19
        }
        
        // Set timestamp to current time of week
        data.timestamp = data.time_of_week;
        
        // Mark as valid
        data.is_valid = true;
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing PVTSLN message: " << e.what() << std::endl;
        data.is_valid = false;
        return false;
    }
}

} // namespace unicore_um982_driver
