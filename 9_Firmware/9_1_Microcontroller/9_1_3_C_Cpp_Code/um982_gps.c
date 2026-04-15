/*******************************************************************************
 * um982_gps.c -- UM982 dual-antenna GNSS receiver driver implementation
 *
 * See um982_gps.h for API documentation.
 * Command syntax per Unicore N4 Command Reference EN R1.14.
 ******************************************************************************/
#include "um982_gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ========================= Internal helpers ========================== */

/**
 * Advance to the next comma-delimited field in an NMEA sentence.
 * Returns pointer to the start of the next field (after the comma),
 * or NULL if no more commas found before end-of-string or '*'.
 *
 * Handles empty fields (consecutive commas) correctly by returning
 * a pointer to the character after the comma (which may be another comma).
 */
static const char *next_field(const char *p)
{
    if (p == NULL) return NULL;
    while (*p != '\0' && *p != ',' && *p != '*') {
        p++;
    }
    if (*p == ',') return p + 1;
    return NULL;  /* End of sentence or checksum marker */
}

/**
 * Get the length of the current field (up to next comma, '*', or '\0').
 */
static int field_len(const char *p)
{
    int len = 0;
    if (p == NULL) return 0;
    while (p[len] != '\0' && p[len] != ',' && p[len] != '*') {
        len++;
    }
    return len;
}

/**
 * Check if a field is non-empty (has at least one character before delimiter).
 */
static bool field_valid(const char *p)
{
    return p != NULL && field_len(p) > 0;
}

/**
 * Parse a floating-point value from a field, returning 0.0 if empty.
 */
static double field_to_double(const char *p)
{
    if (!field_valid(p)) return 0.0;
    return strtod(p, NULL);
}

static float field_to_float(const char *p)
{
    return (float)field_to_double(p);
}

static int field_to_int(const char *p)
{
    if (!field_valid(p)) return 0;
    return (int)strtol(p, NULL, 10);
}

/* ========================= Checksum ================================== */

bool um982_verify_checksum(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') return false;

    const char *p = sentence + 1;  /* Skip '$' */
    uint8_t computed = 0;

    while (*p != '\0' && *p != '*') {
        computed ^= (uint8_t)*p;
        p++;
    }

    if (*p != '*') return false;  /* No checksum marker found */
    p++;  /* Skip '*' */

    /* Parse 2-char hex checksum */
    if (p[0] == '\0' || p[1] == '\0') return false;

    char hex_str[3] = { p[0], p[1], '\0' };
    unsigned long expected = strtoul(hex_str, NULL, 16);

    return computed == (uint8_t)expected;
}

/* ========================= Coordinate parsing ======================== */

double um982_parse_coord(const char *field, char hemisphere)
{
    if (field == NULL || field[0] == '\0') return NAN;

    /* Find the decimal point to determine degree digit count.
     * Latitude:  ddmm.mmmm  (dot at index 4, degrees = 2)
     * Longitude: dddmm.mmmm (dot at index 5, degrees = 3)
     * General:   degree_digits = dot_position - 2
     */
    const char *dot = strchr(field, '.');
    if (dot == NULL) return NAN;

    int dot_pos = (int)(dot - field);
    int deg_digits = dot_pos - 2;

    if (deg_digits < 1 || deg_digits > 3) return NAN;

    /* Extract degree portion */
    double degrees = 0.0;
    for (int i = 0; i < deg_digits; i++) {
        if (field[i] < '0' || field[i] > '9') return NAN;
        degrees = degrees * 10.0 + (field[i] - '0');
    }

    /* Extract minutes portion (everything from deg_digits onward) */
    double minutes = strtod(field + deg_digits, NULL);
    if (minutes < 0.0 || minutes >= 60.0) return NAN;

    double result = degrees + minutes / 60.0;

    /* Apply hemisphere sign */
    if (hemisphere == 'S' || hemisphere == 'W') {
        result = -result;
    }

    return result;
}

/* ========================= Sentence parsers ========================== */

/**
 * Identify the NMEA sentence type by skipping the 2-char talker ID
 * and comparing the 3-letter formatter.
 *
 * "$GNGGA,..." -> talker="GN", formatter="GGA"
 * "$GPTHS,..." -> talker="GP", formatter="THS"
 *
 * Returns pointer to the formatter (3 chars at sentence+3), or NULL
 * if sentence is too short.
 */
static const char *get_formatter(const char *sentence)
{
    /* sentence starts with '$', followed by 2-char talker + 3-char formatter */
    if (sentence == NULL || strlen(sentence) < 6) return NULL;
    return sentence + 3;  /* Skip "$XX" -> points to formatter */
}

/**
 * Parse GGA sentence — position and fix quality.
 *
 * Format: $--GGA,time,lat,N/S,lon,E/W,quality,numSat,hdop,alt,M,geoidSep,M,dgpsAge,refID*XX
 *         field:  1    2   3    4   5     6      7     8    9 10    11    12   13     14
 */
static void parse_gga(UM982_GPS_t *gps, const char *sentence)
{
    /* Skip to first field (after "$XXGGA,") */
    const char *f = strchr(sentence, ',');
    if (f == NULL) return;
    f++;  /* f -> field 1 (time) */

    /* Field 1: UTC time — skip for now */
    const char *f2 = next_field(f);   /* lat */
    const char *f3 = next_field(f2);  /* N/S */
    const char *f4 = next_field(f3);  /* lon */
    const char *f5 = next_field(f4);  /* E/W */
    const char *f6 = next_field(f5);  /* quality */
    const char *f7 = next_field(f6);  /* numSat */
    const char *f8 = next_field(f7);  /* hdop */
    const char *f9 = next_field(f8);  /* altitude */
    const char *f10 = next_field(f9); /* M */
    const char *f11 = next_field(f10); /* geoid sep */

    /* Parse fix quality first — if 0, position is meaningless */
    gps->fix_quality = (uint8_t)field_to_int(f6);

    /* Parse coordinates */
    if (field_valid(f2) && field_valid(f3)) {
        char hem = field_valid(f3) ? *f3 : 'N';
        double lat = um982_parse_coord(f2, hem);
        if (!isnan(lat)) gps->latitude = lat;
    }

    if (field_valid(f4) && field_valid(f5)) {
        char hem = field_valid(f5) ? *f5 : 'E';
        double lon = um982_parse_coord(f4, hem);
        if (!isnan(lon)) gps->longitude = lon;
    }

    /* Number of satellites */
    gps->num_satellites = (uint8_t)field_to_int(f7);

    /* HDOP */
    if (field_valid(f8)) {
        gps->hdop = field_to_float(f8);
    }

    /* Altitude */
    if (field_valid(f9)) {
        gps->altitude = field_to_float(f9);
    }

    /* Geoid separation */
    if (field_valid(f11)) {
        gps->geoid_sep = field_to_float(f11);
    }

    gps->last_gga_tick = HAL_GetTick();
}

/**
 * Parse RMC sentence — recommended minimum (position, speed, date).
 *
 * Format: $--RMC,time,status,lat,N/S,lon,E/W,speed,course,date,magVar,E/W,mode*XX
 *         field:  1     2      3   4   5   6    7      8     9    10    11   12
 */
static void parse_rmc(UM982_GPS_t *gps, const char *sentence)
{
    const char *f = strchr(sentence, ',');
    if (f == NULL) return;
    f++;  /* f -> field 1 (time) */

    const char *f2 = next_field(f);   /* status */
    const char *f3 = next_field(f2);  /* lat */
    const char *f4 = next_field(f3);  /* N/S */
    const char *f5 = next_field(f4);  /* lon */
    const char *f6 = next_field(f5);  /* E/W */
    const char *f7 = next_field(f6);  /* speed knots */
    const char *f8 = next_field(f7);  /* course true */

    /* Status */
    if (field_valid(f2)) {
        gps->rmc_status = *f2;
    }

    /* Position (only if status = A for valid) */
    if (field_valid(f2) && *f2 == 'A') {
        if (field_valid(f3) && field_valid(f4)) {
            double lat = um982_parse_coord(f3, *f4);
            if (!isnan(lat)) gps->latitude = lat;
        }
        if (field_valid(f5) && field_valid(f6)) {
            double lon = um982_parse_coord(f5, *f6);
            if (!isnan(lon)) gps->longitude = lon;
        }
    }

    /* Speed (knots) */
    if (field_valid(f7)) {
        gps->speed_knots = field_to_float(f7);
    }

    /* Course */
    if (field_valid(f8)) {
        gps->course_true = field_to_float(f8);
    }

    gps->last_rmc_tick = HAL_GetTick();
}

/**
 * Parse THS sentence — true heading and status (UM982-specific).
 *
 * Format: $--THS,heading,mode*XX
 *         field:    1      2
 */
static void parse_ths(UM982_GPS_t *gps, const char *sentence)
{
    const char *f = strchr(sentence, ',');
    if (f == NULL) return;
    f++;  /* f -> field 1 (heading) */

    const char *f2 = next_field(f);  /* mode */

    /* Heading */
    if (field_valid(f)) {
        gps->heading = field_to_float(f);
    } else {
        gps->heading = NAN;
    }

    /* Mode */
    if (field_valid(f2)) {
        gps->heading_mode = *f2;
    } else {
        gps->heading_mode = 'V';  /* Not valid if missing */
    }

    gps->last_ths_tick = HAL_GetTick();
}

/**
 * Parse VTG sentence — course and speed over ground.
 *
 * Format: $--VTG,courseTrue,T,courseMag,M,speedKnots,N,speedKmh,K,mode*XX
 *         field:     1      2     3      4     5      6    7     8   9
 */
static void parse_vtg(UM982_GPS_t *gps, const char *sentence)
{
    const char *f = strchr(sentence, ',');
    if (f == NULL) return;
    f++;  /* f -> field 1 (course true) */

    const char *f2 = next_field(f);   /* T */
    const char *f3 = next_field(f2);  /* course mag */
    const char *f4 = next_field(f3);  /* M */
    const char *f5 = next_field(f4);  /* speed knots */
    const char *f6 = next_field(f5);  /* N */
    const char *f7 = next_field(f6);  /* speed km/h */

    /* Course true */
    if (field_valid(f)) {
        gps->course_true = field_to_float(f);
    }

    /* Speed knots */
    if (field_valid(f5)) {
        gps->speed_knots = field_to_float(f5);
    }

    /* Speed km/h */
    if (field_valid(f7)) {
        gps->speed_kmh = field_to_float(f7);
    }

    gps->last_vtg_tick = HAL_GetTick();
}

/* ========================= Sentence dispatch ========================= */

void um982_parse_sentence(UM982_GPS_t *gps, const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') return;

    /* Verify checksum before parsing */
    if (!um982_verify_checksum(sentence)) return;

    /* Check for VERSIONA response (starts with '#', not '$') -- handled separately */
    /* Actually VERSIONA starts with '#', so it won't enter here. We check in feed(). */

    /* Identify sentence type */
    const char *fmt = get_formatter(sentence);
    if (fmt == NULL) return;

    if (strncmp(fmt, "GGA", 3) == 0) {
        parse_gga(gps, sentence);
    } else if (strncmp(fmt, "RMC", 3) == 0) {
        parse_rmc(gps, sentence);
    } else if (strncmp(fmt, "THS", 3) == 0) {
        parse_ths(gps, sentence);
    } else if (strncmp(fmt, "VTG", 3) == 0) {
        parse_vtg(gps, sentence);
    }
    /* Other sentences silently ignored */
}

/* ========================= Command interface ========================= */

bool um982_send_command(UM982_GPS_t *gps, const char *cmd)
{
    if (gps == NULL || gps->huart == NULL || cmd == NULL) return false;

    /* Build command with \r\n termination */
    char buf[UM982_CMD_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    if (len <= 0 || (size_t)len >= sizeof(buf)) return false;

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        gps->huart, (const uint8_t *)buf, (uint16_t)len, 100);

    return status == HAL_OK;
}

/* ========================= Line assembly + feed ====================== */

/**
 * Process a completed line from the line buffer.
 */
static void process_line(UM982_GPS_t *gps, const char *line)
{
    if (line == NULL || line[0] == '\0') return;

    /* NMEA sentence starts with '$' */
    if (line[0] == '$') {
        um982_parse_sentence(gps, line);
        return;
    }

    /* Unicore proprietary response starts with '#' (e.g. #VERSIONA) */
    if (line[0] == '#') {
        if (strncmp(line + 1, "VERSIONA", 8) == 0) {
            gps->version_received = true;
        }
        return;
    }
}

void um982_feed(UM982_GPS_t *gps, const uint8_t *data, uint16_t len)
{
    if (gps == NULL || data == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t ch = data[i];

        /* End of line: process if we have content */
        if (ch == '\n' || ch == '\r') {
            if (gps->line_len > 0 && !gps->line_overflow) {
                gps->line_buf[gps->line_len] = '\0';
                process_line(gps, gps->line_buf);
            }
            gps->line_len = 0;
            gps->line_overflow = false;
            continue;
        }

        /* Accumulate into line buffer */
        if (gps->line_len < UM982_LINE_BUF_SIZE - 1) {
            gps->line_buf[gps->line_len++] = (char)ch;
        } else {
            gps->line_overflow = true;
        }
    }
}

/* ========================= UART process (production) ================= */

void um982_process(UM982_GPS_t *gps)
{
    if (gps == NULL || gps->huart == NULL) return;

    /* Read all available bytes from the UART one at a time.
     * At 115200 baud (~11.5 KB/s) and a typical main-loop period of ~10 ms,
     * we expect ~115 bytes per call — negligible overhead on a 168 MHz STM32.
     *
     * Note: batch reads (HAL_UART_Receive with Size > 1 and Timeout = 0) are
     * NOT safe here because the HAL consumes bytes from the data register as
     * it reads them.  If fewer than Size bytes are available, the consumed
     * bytes are lost (HAL_TIMEOUT is returned and the caller has no way to
     * know how many bytes were actually placed into the buffer). */
    uint8_t ch;
    while (HAL_UART_Receive(gps->huart, &ch, 1, 0) == HAL_OK) {
        um982_feed(gps, &ch, 1);
    }
}

/* ========================= Validity checks =========================== */

bool um982_is_heading_valid(const UM982_GPS_t *gps)
{
    if (gps == NULL) return false;
    if (isnan(gps->heading)) return false;

    /* Mode must be Autonomous or Differential */
    if (gps->heading_mode != 'A' && gps->heading_mode != 'D') return false;

    /* Check age */
    uint32_t age = HAL_GetTick() - gps->last_ths_tick;
    return age < UM982_HEADING_TIMEOUT_MS;
}

bool um982_is_position_valid(const UM982_GPS_t *gps)
{
    if (gps == NULL) return false;
    if (gps->fix_quality == UM982_FIX_NONE) return false;

    /* Check age — use GGA timestamp */
    uint32_t age = HAL_GetTick() - gps->last_gga_tick;
    return age < UM982_POSITION_TIMEOUT_MS;
}

uint32_t um982_heading_age(const UM982_GPS_t *gps)
{
    if (gps == NULL) return UINT32_MAX;
    return HAL_GetTick() - gps->last_ths_tick;
}

uint32_t um982_position_age(const UM982_GPS_t *gps)
{
    if (gps == NULL) return UINT32_MAX;
    return HAL_GetTick() - gps->last_gga_tick;
}

/* ========================= Initialization ============================ */

bool um982_init(UM982_GPS_t *gps, UART_HandleTypeDef *huart,
                float baseline_cm, float tolerance_cm)
{
    if (gps == NULL || huart == NULL) return false;

    /* Zero-init entire structure */
    memset(gps, 0, sizeof(UM982_GPS_t));

    gps->huart = huart;
    gps->heading = NAN;
    gps->heading_mode = 'V';
    gps->rmc_status = 'V';
    gps->speed_knots = 0.0f;
    gps->speed_kmh = 0.0f;
    gps->course_true = 0.0f;

    /* Step 1: Stop all current output to get a clean slate */
    um982_send_command(gps, "UNLOG");
    HAL_Delay(100);

    /* Step 2: Configure heading mode
     * Per N4 Reference 4.18: CONFIG HEADING FIXLENGTH (default mode)
     * "The distance between ANT1 and ANT2 is fixed. They move synchronously." */
    um982_send_command(gps, "CONFIG HEADING FIXLENGTH");
    HAL_Delay(50);

    /* Step 3: Set baseline length if specified
     * Per N4 Reference: CONFIG HEADING LENGTH <cm> <tolerance_cm>
     * "parameter1: Fixed baseline length (cm), valid range >= 0"
     * "parameter2: Tolerable error margin (cm), valid range > 0" */
    if (baseline_cm > 0.0f) {
        char cmd[64];
        if (tolerance_cm > 0.0f) {
            snprintf(cmd, sizeof(cmd), "CONFIG HEADING LENGTH %.0f %.0f",
                     baseline_cm, tolerance_cm);
        } else {
            snprintf(cmd, sizeof(cmd), "CONFIG HEADING LENGTH %.0f",
                     baseline_cm);
        }
        um982_send_command(gps, cmd);
        HAL_Delay(50);
    }

    /* Step 4: Enable NMEA output sentences on COM1.
     * Per N4 Reference: "When requesting NMEA messages, users should add GP
     * before each command name"
     *
     * IMPORTANT: "COM1" here refers to the UM982's serial port 1 (pins 17/18).
     * The MCU UART (huart5) must be physically wired to UM982 COM1.  If the
     * board routes to COM2 or COM3 instead, change "COM1" to "COM2"/"COM3"
     * in all the commands below.  Verify against the schematic. */
    um982_send_command(gps, "GPGGA COM1 1");     /* GGA at 1 Hz */
    HAL_Delay(50);
    um982_send_command(gps, "GPRMC COM1 1");     /* RMC at 1 Hz */
    HAL_Delay(50);
    um982_send_command(gps, "GPTHS COM1 0.2");   /* THS at 5 Hz (heading primary) */
    HAL_Delay(50);

    /* Step 5: Skip SAVECONFIG -- NMEA config is re-sent every boot anyway.
     * Saving to NVM on every power cycle would wear flash.  If persistent
     * config is needed, call um982_send_command(gps, "SAVECONFIG") once
     * during commissioning. */

    /* Step 6: Query version to verify communication */
    gps->version_received = false;
    um982_send_command(gps, "VERSIONA");

    /* Wait for VERSIONA response (non-blocking poll) */
    uint32_t start = HAL_GetTick();
    while (!gps->version_received &&
           (HAL_GetTick() - start) < UM982_INIT_TIMEOUT_MS) {
        um982_process(gps);
        HAL_Delay(10);
    }

    gps->initialized = gps->version_received;
    return gps->initialized;
}
