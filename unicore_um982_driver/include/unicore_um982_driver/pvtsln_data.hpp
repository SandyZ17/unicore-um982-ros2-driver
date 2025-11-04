#ifndef UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_
#define UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace unicore_um982_driver
{

struct PVTSLNData
{
    // Header information (Table 7-49/7-50)
    std::string message_id;          // Log identifier (e.g. PVTSLNA)
    int32_t sequence_number;         // Message sequence counter
    std::string port_id;             // Port or GNSS mode string
    std::string time_status;         // Receiver time status
    int32_t week;                    // GPS week number
    double time_of_week;             // Milliseconds into the GPS week
    uint32_t receiver_status;        // Receiver status bitfield
    uint32_t reserved;               // Reserved header field
    int32_t solution_status;         // Solution status code
    int32_t position_type;           // Position type code

    // BESTPOS-equivalent fields
    std::string bestpos_type;        // Position type string (SINGLE/RTK/...)
    double bestpos_height;           // Height above mean sea level (m)
    double bestpos_latitude;         // Latitude (deg)
    double bestpos_longitude;        // Longitude (deg)
    double bestpos_height_std;       // Height standard deviation (m)
    double bestpos_latitude_std;     // Latitude standard deviation (m)
    double bestpos_longitude_std;    // Longitude standard deviation (m)
    double bestpos_diff_age;         // Differential age (s)
    double age_of_corrections;       // Convenience alias for diff age

    // Pseudorange position solution
    std::string psrpos_type;         // Pseudorange position type string
    double psrpos_height;            // Height above mean sea level (m)
    double psrpos_latitude;          // Latitude (deg)
    double psrpos_longitude;         // Longitude (deg)

    double undulation;               // Geoid undulation (m)

    uint8_t bestpos_svs;             // Satellites tracked for bestpos
    uint8_t bestpos_solnsvs;         // Satellites used in bestpos solution
    uint8_t psrpos_svs;              // Satellites tracked for psrpos
    uint8_t psrpos_solnsvs;          // Satellites used in psrpos solution

    // Velocity and heading
    double psrvel_north;             // North velocity (m/s)
    double psrvel_east;              // East velocity (m/s)
    double psrvel_ground;            // Horizontal speed over ground (m/s)
    std::string heading_type;        // Heading solution status
    double heading_length;           // Baseline length (m)
    double heading_degree;           // Heading (deg)
    double heading_pitch;            // Pitch (deg)
    uint8_t heading_trackedsvs;      // Satellites tracked by master antenna
    uint8_t heading_solnsvs;         // Satellites used in heading solution
    uint8_t heading_ggl1;            // Satellites using L1 frequency
    uint8_t heading_ggl1l2;          // Satellites using L1/L2 frequencies

    // Dilution of precision and measurement settings
    double gdop;
    double pdop;
    double hdop;
    double htdop;
    double tdop;
    double elevation_cutoff;         // Elevation cutoff angle (deg)

    // Satellite PRN list
    uint16_t prn_count;              // Number of PRNs reported
    std::vector<uint16_t> prn_list;  // PRNs of tracked satellites

    // CRC value (if provided in ASCII log)
    uint32_t crc;

    // Convenience fields used elsewhere in the driver
    std::string position_status;     // Mirrors bestpos_type
    double latitude;                 // Mirrors bestpos_latitude
    double longitude;                // Mirrors bestpos_longitude
    double altitude;                 // Mirrors bestpos_height
    int num_satellites_tracked;      // Mirrors bestpos_svs
    int num_satellites_used;         // Mirrors bestpos_solnsvs
    double sigma_latitude;           // Mirrors bestpos_latitude_std
    double sigma_longitude;          // Mirrors bestpos_longitude_std
    double sigma_altitude;           // Mirrors bestpos_height_std
    double heading;                  // Mirrors heading_degree
    double velocity_north;           // Mirrors psrvel_north
    double velocity_east;            // Mirrors psrvel_east
    double velocity_up;              // Not provided -> set to 0

    // Derived meta-data
    double timestamp;
    bool is_valid;

    PVTSLNData();
};

inline PVTSLNData::PVTSLNData()
    : sequence_number(0)
    , week(0)
    , time_of_week(0.0)
    , receiver_status(0)
    , reserved(0)
    , solution_status(0)
    , position_type(0)
    , bestpos_height(0.0)
    , bestpos_latitude(0.0)
    , bestpos_longitude(0.0)
    , bestpos_height_std(0.0)
    , bestpos_latitude_std(0.0)
    , bestpos_longitude_std(0.0)
    , bestpos_diff_age(0.0)
    , age_of_corrections(0.0)
    , psrpos_height(0.0)
    , psrpos_latitude(0.0)
    , psrpos_longitude(0.0)
    , undulation(0.0)
    , bestpos_svs(0)
    , bestpos_solnsvs(0)
    , psrpos_svs(0)
    , psrpos_solnsvs(0)
    , psrvel_north(0.0)
    , psrvel_east(0.0)
    , psrvel_ground(0.0)
    , heading_length(0.0)
    , heading_degree(0.0)
    , heading_pitch(0.0)
    , heading_trackedsvs(0)
    , heading_solnsvs(0)
    , heading_ggl1(0)
    , heading_ggl1l2(0)
    , gdop(0.0)
    , pdop(0.0)
    , hdop(0.0)
    , htdop(0.0)
    , tdop(0.0)
    , elevation_cutoff(0.0)
    , prn_count(0)
    , crc(0)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , num_satellites_tracked(0)
    , num_satellites_used(0)
    , sigma_latitude(0.0)
    , sigma_longitude(0.0)
    , sigma_altitude(0.0)
    , heading(0.0)
    , velocity_north(0.0)
    , velocity_east(0.0)
    , velocity_up(0.0)
    , timestamp(0.0)
    , is_valid(false)
{
}

// Function to parse PVTSLN message
bool parsePVTSLN(const std::string& line, PVTSLNData& data, std::string* error_message = nullptr);

} // namespace unicore_um982_driver

#endif // UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_
