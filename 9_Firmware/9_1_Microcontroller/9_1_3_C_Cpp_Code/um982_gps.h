// um982_gps.h
#ifndef UM982_GPS_H
#define UM982_GPS_H

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

// UM982 NMEA Sentence Types
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GGA,     // Global Positioning System Fix Data
    NMEA_RMC,     // Recommended Minimum Specific GNSS Data
    NMEA_GSA,     // GNSS DOP and Active Satellites
    NMEA_GSV,     // GNSS Satellites in View
    NMEA_VTG,     // Course Over Ground and Ground Speed
    NMEA_GLL,     // Geographic Position - Latitude/Longitude
    NMEA_ZDA,     // Time and Date
    NMEA_GPTHS,     // Heading/Track from UM982
    NMEA_PASH     // Proprietary UM982 messages
} NMEA_SentenceType;

// UM982 RTK Status
typedef enum {
    RTK_FIX_INVALID = 0,
    RTK_SINGLE = 1,      // Single point positioning
    RTK_DGPS = 2,        // Differential GPS (SBAS)
    RTK_FLOAT = 4,       // Float RTK
    RTK_FIXED = 5,       // Fixed RTK (centimeter precision)
    RTK_DEAD_RECKON = 6  // Dead reckoning
} RTK_FixType;

// UM982 Navigation Status
typedef struct {
    // Position
    double latitude;           // Degrees (negative = South)
    double longitude;          // Degrees (negative = West)
    float altitude;            // Meters (MSL)
    float geoid_separation;    // Geoid separation (WGS84)
    
    // Time
    uint32_t utc_hour;
    uint32_t utc_minute;
    uint32_t utc_second;
    uint32_t utc_millisecond;
    uint32_t timestamp_ms;     // System timestamp when data was received
    
    // Course and Speed
    float heading;             // Degrees (0-360, True North)
    float ground_speed_knots;  // Speed over ground in knots
    float ground_speed_kmh;    // Speed over ground in km/h
    float ground_speed_ms;     // Speed over ground in m/s
    
    // Accuracy and Quality
    RTK_FixType fix_type;      // RTK fix type
    uint8_t sat_count;         // Number of satellites used
    float hdop;                // Horizontal Dilution of Precision
    float vdop;                // Vertical Dilution of Precision
    float pdop;                // Position Dilution of Precision
    
    // RTK specific
    float age_of_differential; // Age of differential corrections (seconds)
    uint32_t differential_ref_station_id;
    
    // Validity flags
    bool data_valid;
    bool heading_valid;
    bool position_valid;
    bool time_valid;
    
} UM982_Data_t;

// UM982 Handler Class
class UM982_GPS {
public:
    UM982_GPS(UART_HandleTypeDef* huart);
    ~UM982_GPS();
    
    // Initialization and Configuration
    bool init(bool enable_rtk, bool enable_heading, float baseline_m, byte update_rate);
    bool setUpdateRate(uint8_t hz);
    bool setRTKMode(bool enable);
    bool saveConfiguration();
    
    // Data Processing
    void process();            // Call in main loop to parse incoming data
    bool processByte(uint8_t data);
    bool isDataUpdated();
    
    // Data Access
    UM982_Data_t getData();
    double getLatitude() const { return data_.latitude; }
    double getLongitude() const { return data_.longitude; }
    float getHeading() const { return data_.heading; }
    float getAltitude() const { return data_.altitude; }
    RTK_FixType getFixType() const { return data_.fix_type; }
    uint8_t getSatelliteCount() const { return data_.sat_count; }
    
    // Status
    bool hasFix() const { return (data_.fix_type >= RTK_SINGLE); }
    bool hasRTKFix() const { return (data_.fix_type == RTK_FIXED); }
    bool isDataValid() const { return data_.data_valid; }
    
    // Utility
    static double parseNMEACoordinate(const char* coord, char hemisphere);
    static float parseNMEAFloat(const char* str);
    static uint32_t parseNMEAInt(const char* str);
    
private:
    UART_HandleTypeDef* uart_;
    UM982_Data_t data_;
    UM982_Data_t last_data_;
    UM982_Config_t config_;
    
    // NMEA Parsing
    char rx_buffer_[256];
    uint16_t buffer_index_;
    bool data_updated_;
    uint32_t last_update_ms_;
    
    // Sentence parsing
    NMEA_SentenceType identifySentence(const char* sentence);
    bool parseGGA(const char* sentence);
    bool parseRMC(const char* sentence);
    bool parseGSA(const char* sentence);
    bool parseVTG(const char* sentence);
    bool parseGPTHS(const char* sentence);  // UM982 proprietary heading sentence
    
    // Command generation
    void sendCommand(const char* command);
    
    // Helper methods
    void updateData();
    void resetBuffer();
};

#ifdef __cplusplus
}
#endif

#endif // UM982_GPS_H