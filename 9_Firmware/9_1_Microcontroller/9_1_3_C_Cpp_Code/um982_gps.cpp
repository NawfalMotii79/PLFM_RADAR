// um982_gps.cpp
#include "um982_gps.h"
#include <stdio.h>

// UM982 Default NMEA Commands
#define UM982_CMD_RESET "RESET"
#define UM982_CMD_SAVE_CONFIG "SAVECONFIG"
#define UM982_CMD_RATE_1HZ "GPTHS COM1 1"
#define UM982_CMD_RATE_5HZ "GPTHS COM1 0.2"
#define UM982_CMD_RATE_10HZ "GPTHS COM1 0.1"
#define UM982_CMD_RATE_20HZ "GPTHS COM1 0.05"
#define UM982_CMD_ENABLE_RTK "CONFIG RTK RELIABILITY 3 1"
#define UM982_CMD_DISABLE_RTK "CONFIG RTK DISABLE"
#define UM982_CMD_HEADING_FIXLENGTH "CONFIG HEADING FIXLENGTH"
#define UM982_CMD_HEADING_LENGTH "CONFIG HEADING LENGTH %.1f"

// NMEA Sentence Starters
#define NMEA_GGA_START "GGA"
#define NMEA_RMC_START "RMC"
#define NMEA_GSA_START "GSA"
#define NMEA_VTG_START "VTG"
#define NMEA_GPTHS_START "GNTHS" // UM982 Heading/Track  | "GNTHS" is correct, The Output Message is: $GNTHS,341.3065,A*1F

// Constructor
UM982_GPS::UM982_GPS(UART_HandleTypeDef *huart)
    : uart_(huart), buffer_index_(0), data_updated_(false), last_update_ms_(0)
{

    if (uart_ == nullptr)
    {
        return;
    }

    memset(&data_, 0, sizeof(UM982_Data_t));
    memset(&last_data_, 0, sizeof(UM982_Data_t));
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

UM982_GPS::~UM982_GPS()
{
    // Cleanup if needed
}

// Initialization
bool UM982_GPS::init(bool enable_rtk, bool enable_heading, float baseline_m, byte update_rate)
{
    // Clear buffer
    resetBuffer();

    // Wait for UM982 to stabilize
    HAL_Delay(1000);

    if (enable_rtk)
    {
        sendCommand(UM982_CMD_ENABLE_RTK);
        HAL_Delay(50);
    }
    else
    {
        sendCommand(UM982_CMD_DISABLE_RTK);
        HAL_Delay(50);
    }

    if (enable_heading)
    {
        sendCommand(UM982_CMD_HEADING_FIXLENGTH);
        HAL_Delay(50);

        char len_cmd[64];
        snprintf(len_cmd, sizeof(len_cmd), UM982_CMD_HEADING_LENGTH, baseline_m);
        sendCommand(len_cmd);
        HAL_Delay(50);
    }

    switch (update_rate)
    {
    case 1:
        sendCommand(UM982_CMD_RATE_1HZ);
        break;
    case 5:
        sendCommand(UM982_CMD_RATE_5HZ);
        break;
    case 10:
        sendCommand(UM982_CMD_RATE_10HZ);
        break;
    case 20:
        sendCommand(UM982_CMD_RATE_20HZ);
        break;
    default:
        sendCommand(UM982_CMD_RATE_10HZ);
        break;
    }
    HAL_Delay(100);

    sendCommand(UM982_CMD_SAVE_CONFIG);
    HAL_Delay(300);

    return true;
}

// Set update rate
bool UM982_GPS::setUpdateRate(uint8_t hz)
{
    const char *cmd;

    switch (hz)
    {
    case 1:
        cmd = UM982_CMD_RATE_1HZ;
        break;
    case 5:
        cmd = UM982_CMD_RATE_5HZ;
        break;
    case 10:
        cmd = UM982_CMD_RATE_10HZ;
        break;
    case 20:
        cmd = UM982_CMD_RATE_20HZ;
        break;
    default:
        cmd = UM982_CMD_RATE_10HZ;
        break;
    }

    sendCommand(cmd);
    HAL_Delay(100);
    return true;
}

// Set RTK mode
bool UM982_GPS::setRTKMode(bool enable)
{
    if (enable)
    {
        sendCommand(UM982_CMD_ENABLE_RTK);
    }
    else
    {
        sendCommand(UM982_CMD_DISABLE_RTK);
    }

    HAL_Delay(100);
    return true;
}

// Save configuration to flash
bool UM982_GPS::saveConfiguration()
{
    sendCommand(UM982_CMD_SAVE_CONFIG);
    HAL_Delay(500);
    return true;
}

// Send command to UM982
void UM982_GPS::sendCommand(const char *command)
{
    if (command == nullptr || uart_ == nullptr)
        return;
    if (strlen(command) > 240)
        return;

    HAL_UART_Transmit(uart_, (uint8_t *)command, strlen(command), 100);
}

// Process incoming data (call in main loop)
void UM982_GPS::process()
{
    uint8_t data;

    // Read all available bytes
    while (HAL_UART_Receive(uart_, &data, 1, 0) == HAL_OK)
    {
        processByte(data);
    }
}

bool UM982_GPS::processByte(uint8_t data)
{
    // Start of new sentence
    if (data == '$')
    {
        resetBuffer();
        rx_buffer_[buffer_index_++] = data;
        return true;
    }

    // End of sentence (accept both \r\n and \n only)
    if ((data == '\n' || data == '\r') && buffer_index_ > 0)
    {
        if (data == '\n')
        {
            rx_buffer_[buffer_index_] = '\0';

            const char *sentence = rx_buffer_ + 1; // skip '$'
            NMEA_SentenceType type = identifySentence(sentence);

            // Only parse if checksum is valid
            if (verifyNMEAChecksum(rx_buffer_))
            {
                switch (type)
                {
                case NMEA_GGA:
                    parseGGA(sentence);
                    break;
                case NMEA_RMC:
                    parseRMC(sentence);
                    break;
                case NMEA_GSA:
                    parseGSA(sentence);
                    break;
                case NMEA_VTG:
                    parseVTG(sentence);
                    break;
                case NMEA_GPTHS:
                    parseGPTHS(sentence);
                    break;
                default:
                    break;
                }
            }

            resetBuffer();
        }
        return true;
    }

    // Store byte if buffer not full
    if (buffer_index_ < sizeof(rx_buffer_) - 1)
    {
        rx_buffer_[buffer_index_++] = data;
    }

    return true;
}

bool UM982_GPS::verifyNMEAChecksum(const char *sentence)
{
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk || asterisk[3] != '\0')
        return false; // must have *XX

    uint8_t calculated = 0;
    for (const char *p = sentence + 1; p < asterisk; ++p)
    {
        calculated ^= *p;
    }

    uint8_t received = (uint8_t)strtol(asterisk + 1, NULL, 16);
    return calculated == received;
}

// Identify NMEA sentence type
NMEA_SentenceType UM982_GPS::identifySentence(const char *sentence)
{
    if (strncmp(sentence, NMEA_GGA_START, 3) == 0)
        return NMEA_GGA;
    if (strncmp(sentence, NMEA_RMC_START, 3) == 0)
        return NMEA_RMC;
    if (strncmp(sentence, NMEA_GSA_START, 3) == 0)
        return NMEA_GSA;
    if (strncmp(sentence, NMEA_VTG_START, 3) == 0)
        return NMEA_VTG;
    if (strncmp(sentence, NMEA_GPTHS_START, 5) == 0)
        return NMEA_GPTHS;
    return NMEA_UNKNOWN;
}

// Parse GGA sentence (time, position, fix type)
bool UM982_GPS::parseGGA(const char *sentence)
{
    char buffer[32];
    int field = 0;
    const char *ptr = sentence;

    while (*ptr && *ptr != '*')
    {
        if (*ptr == ',' || *ptr == '\r')
        {
            field++;
            ptr++;
            continue;
        }

        // Extract field based on position
        int i = 0;
        while (*ptr && *ptr != ',' && *ptr != '\r' && i < 31)
        {
            buffer[i++] = *ptr++;
        }
        buffer[i] = '\0';

        switch (field)
        {
        case 0: // Skip "GGA"
            break;
        case 1: // UTC Time (HHMMSS.SS)
            if (strlen(buffer) >= 6)
            {
                data_.utc_hour = (buffer[0] - '0') * 10 + (buffer[1] - '0');
                data_.utc_minute = (buffer[2] - '0') * 10 + (buffer[3] - '0');
                data_.utc_second = (buffer[4] - '0') * 10 + (buffer[5] - '0');
            }
            break;
        case 2: // Latitude
        {
            char lat_str[16];
            strcpy(lat_str, buffer);
            // Next field is hemisphere
            while (*ptr && (*ptr == ',' || *ptr == '\r'))
                ptr++;
            char hem = *ptr;
            data_.latitude = parseNMEACoordinate(lat_str, hem);
        }
        break;
        case 3: // Skip hemisphere (already processed)
            break;
        case 4: // Longitude
        {
            char lon_str[16];
            strcpy(lon_str, buffer);
            while (*ptr && (*ptr == ',' || *ptr == '\r'))
                ptr++;
            char hem = *ptr;
            data_.longitude = parseNMEACoordinate(lon_str, hem);
        }
        break;
        case 5: // Skip hemisphere
            break;
        case 6: // Fix quality
            data_.fix_type = (RTK_FixType)parseNMEAInt(buffer);
            break;
        case 7: // Number of satellites
            data_.sat_count = parseNMEAInt(buffer);
            break;
        case 8: // HDOP
            data_.hdop = parseNMEAFloat(buffer);
            break;
        case 9: // Altitude
            data_.altitude = parseNMEAFloat(buffer);
            break;
        default:
            break;
        }

        // Skip to next field
        while (*ptr && *ptr != ',')
            ptr++;
    }

    data_.position_valid = (data_.fix_type >= RTK_SINGLE);
    data_.time_valid = true;
    updateData();

    return true;
}

// Parse RMC sentence (recommended minimum - includes heading)
bool UM982_GPS::parseRMC(const char *sentence)
{
    char buffer[32];
    int field = 0;
    const char *ptr = sentence;

    while (*ptr && *ptr != '*')
    {
        if (*ptr == ',' || *ptr == '\r')
        {
            field++;
            ptr++;
            continue;
        }

        int i = 0;
        while (*ptr && *ptr != ',' && *ptr != '\r' && i < 31)
        {
            buffer[i++] = *ptr++;
        }
        buffer[i] = '\0';

        switch (field)
        {
        case 0: // Skip "RMC"
            break;
        case 1: // UTC Time
            if (strlen(buffer) >= 6)
            {
                data_.utc_hour = (buffer[0] - '0') * 10 + (buffer[1] - '0');
                data_.utc_minute = (buffer[2] - '0') * 10 + (buffer[3] - '0');
                data_.utc_second = (buffer[4] - '0') * 10 + (buffer[5] - '0');
            }
            break;
        case 2: // Status (A=valid, V=invalid)
            data_.data_valid = (buffer[0] == 'A');
            break;
        case 3: // Latitude
        {
            char lat_str[16];
            strcpy(lat_str, buffer);
            while (*ptr && (*ptr == ',' || *ptr == '\r'))
                ptr++;
            char hem = *ptr;
            data_.latitude = parseNMEACoordinate(lat_str, hem);
        }
        break;
        case 4: // Skip hemisphere
            break;
        case 5: // Longitude
        {
            char lon_str[16];
            strcpy(lon_str, buffer);
            while (*ptr && (*ptr == ',' || *ptr == '\r'))
                ptr++;
            char hem = *ptr;
            data_.longitude = parseNMEACoordinate(lon_str, hem);
        }
        break;
        case 6: // Skip hemisphere
            break;
        case 7: // Speed over ground (knots)
            data_.ground_speed_knots = parseNMEAFloat(buffer);
            data_.ground_speed_kmh = data_.ground_speed_knots * 1.852f;
            data_.ground_speed_ms = data_.ground_speed_knots * 0.514444f;
            break;
        case 8: // Track angle (heading)
            data_.heading = parseNMEAFloat(buffer);
            data_.heading_valid = true;
            break;
        case 9: // Date (DDMMYY)
            if (strlen(buffer) >= 6)
            {
                // Date is available but we may not need it
            }
            break;
        default:
            break;
        }

        while (*ptr && *ptr != ',')
            ptr++;
    }

    data_.position_valid = data_.data_valid;
    updateData();

    return true;
}

// Parse GSA sentence (DOP and active satellites)
bool UM982_GPS::parseGSA(const char *sentence)
{
    char buffer[32];
    int field = 0;
    const char *ptr = sentence;

    while (*ptr && *ptr != '*')
    {
        if (*ptr == ',' || *ptr == '\r')
        {
            field++;
            ptr++;
            continue;
        }

        int i = 0;
        while (*ptr && *ptr != ',' && *ptr != '\r' && i < 31)
        {
            buffer[i++] = *ptr++;
        }
        buffer[i] = '\0';

        switch (field)
        {
        case 0: // Skip "GSA"
            break;
        case 1: // Mode (M=manual, A=auto)
            break;
        case 2: // Fix type (1=no fix, 2=2D, 3=3D)
            break;
        case 15: // PDOP
            data_.pdop = parseNMEAFloat(buffer);
            break;
        case 16: // HDOP
            data_.hdop = parseNMEAFloat(buffer);
            break;
        case 17: // VDOP
            data_.vdop = parseNMEAFloat(buffer);
            break;
        default:
            break;
        }

        while (*ptr && *ptr != ',')
            ptr++;
    }

    return true;
}

// Parse VTG sentence (course and speed)
bool UM982_GPS::parseVTG(const char *sentence)
{
    char buffer[32];
    int field = 0;
    const char *ptr = sentence;

    while (*ptr && *ptr != '*')
    {
        if (*ptr == ',' || *ptr == '\r')
        {
            field++;
            ptr++;
            continue;
        }

        int i = 0;
        while (*ptr && *ptr != ',' && *ptr != '\r' && i < 31)
        {
            buffer[i++] = *ptr++;
        }
        buffer[i] = '\0';

        switch (field)
        {
        case 0: // Skip "VTG"
            break;
        case 1: // True track (heading)
            data_.heading = parseNMEAFloat(buffer);
            data_.heading_valid = true;
            break;
        case 2: // Skip "T"
            break;
        case 3: // Magnetic track (skip)
            break;
        case 4: // Skip "M"
            break;
        case 5: // Speed (knots)
            data_.ground_speed_knots = parseNMEAFloat(buffer);
            data_.ground_speed_kmh = data_.ground_speed_knots * 1.852f;
            data_.ground_speed_ms = data_.ground_speed_knots * 0.514444f;
            break;
        default:
            break;
        }

        while (*ptr && *ptr != ',')
            ptr++;
    }

    return true;
}

// Parse GPTHS sentence (UM982 proprietary heading)
bool UM982_GPS::parseGPTHS(const char *sentence)
{
    char buffer[32];
    int field = 0;
    const char *ptr = sentence;

    while (*ptr && *ptr != '*')
    {
        if (*ptr == ',' || *ptr == '\r')
        {
            field++;
            ptr++;
            continue;
        }

        int i = 0;
        while (*ptr && *ptr != ',' && *ptr != '\r' && i < 31)
        {
            buffer[i++] = *ptr++;
        }
        buffer[i] = '\0';

        switch (field)
        {
        case 0: // Skip "GNTHS"
            break;
        case 1: // Heading (True North)
            data_.heading = parseNMEAFloat(buffer);
            data_.heading_valid = true;
            break;
        case 2: // Status (A/V)
            data_.data_valid = (buffer[0] == 'A');
            break;
        default:
            break;
        }

        while (*ptr && *ptr != ',')
            ptr++;
    }

    updateData();
    return true;
}

// Parse NMEA coordinate (DDMM.MMMMM format)
double UM982_GPS::parseNMEACoordinate(const char *coord, char hemisphere)
{
    if (coord == nullptr || strlen(coord) < 4)
        return 0.0;

    // Find decimal point position
    int dot_pos = 0;
    while (coord[dot_pos] != '.' && dot_pos < (int)strlen(coord))
    {
        dot_pos++;
    }

    // Degrees are the first 2 digits before decimal
    int degrees = 0;
    if (dot_pos >= 2)
    {
        degrees = (coord[0] - '0') * 10 + (coord[1] - '0');
    }

    // Minutes are the rest
    double minutes = atof(coord + 2);

    double decimal = degrees + minutes / 60.0;

    // Apply hemisphere (S or W = negative)
    if (hemisphere == 'S' || hemisphere == 'W')
    {
        decimal = -decimal;
    }

    return decimal;
}

// Parse NMEA float value
float UM982_GPS::parseNMEAFloat(const char *str)
{
    if (str == NULL || strlen(str) == 0)
        return 0.0f;
    return atof(str);
}

// Parse NMEA integer value
uint32_t UM982_GPS::parseNMEAInt(const char *str)
{
    if (str == NULL || strlen(str) == 0)
        return 0;
    return atoi(str);
}

// Update data and mark as updated
void UM982_GPS::updateData()
{
    data_.timestamp_ms = HAL_GetTick();

    // Check if data has changed significantly
    if (fabs(data_.latitude - last_data_.latitude) > 1e-6 ||
        fabs(data_.longitude - last_data_.longitude) > 1e-6 ||
        fabs(data_.heading - last_data_.heading) > 0.1f)
    {

        data_updated_ = true;
        last_update_ms_ = data_.timestamp_ms;
        memcpy(&last_data_, &data_, sizeof(UM982_Data_t));
    }
}

// Check if data has been updated
bool UM982_GPS::isDataUpdated()
{
    bool updated = data_updated_;
    data_updated_ = false;
    return updated;
}

// Get complete data structure
UM982_Data_t UM982_GPS::getData()
{
    return data_;
}

// Reset receive buffer
void UM982_GPS::resetBuffer()
{
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    buffer_index_ = 0;
}