/*******************************************************************************
 * um982_gps.h -- UM982 dual-antenna GNSS receiver driver
 *
 * Parses NMEA sentences (GGA, RMC, THS, VTG) from the Unicore UM982 module
 * and provides position, heading, and velocity data.
 *
 * Design principles:
 *   - Non-blocking: process() reads available UART bytes without waiting
 *   - Correct NMEA parsing: proper tokenizer handles empty fields
 *   - Longitude handles 3-digit degrees (dddmm.mmmm) via decimal-point detection
 *   - Checksum verified on every sentence
 *   - Command syntax verified against Unicore N4 Command Reference EN R1.14
 *
 * Hardware: UM982 on UART5 @ 115200 baud, dual-antenna heading mode
 ******************************************************************************/
#ifndef UM982_GPS_H
#define UM982_GPS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward-declare the HAL UART handle type.  The real definition comes from
 * stm32f7xx_hal.h (production) or stm32_hal_mock.h (tests). */
#ifndef STM32_HAL_MOCK_H
#include "stm32f7xx_hal.h"
#else
/* Already included via mock -- nothing to do */
#endif

/* ========================= Constants ================================= */

#define UM982_RX_BUF_SIZE    512   /* Ring buffer for incoming UART bytes */
#define UM982_LINE_BUF_SIZE   96   /* Max NMEA sentence (82 chars + margin) */
#define UM982_CMD_BUF_SIZE   128   /* Outgoing command buffer */
#define UM982_INIT_TIMEOUT_MS 3000 /* Timeout waiting for VERSIONA response */

/* Fix quality values (from GGA field 6) */
#define UM982_FIX_NONE       0
#define UM982_FIX_GPS        1
#define UM982_FIX_DGPS       2
#define UM982_FIX_RTK_FIXED  4
#define UM982_FIX_RTK_FLOAT  5

/* Validity timeout defaults (ms) */
#define UM982_HEADING_TIMEOUT_MS 2000
#define UM982_POSITION_TIMEOUT_MS 5000

/* ========================= Data Types ================================ */

typedef struct {
    /* Position */
    double   latitude;       /* Decimal degrees, positive = North */
    double   longitude;      /* Decimal degrees, positive = East */
    float    altitude;       /* Meters above MSL */
    float    geoid_sep;      /* Geoid separation (meters) */

    /* Heading (from dual-antenna THS) */
    float    heading;        /* True heading 0-360 degrees, NAN if invalid */
    char     heading_mode;   /* A=autonomous, D=diff, E=est, M=manual, S=sim, V=invalid */

    /* Velocity */
    float    speed_knots;    /* Speed over ground (knots) */
    float    speed_kmh;      /* Speed over ground (km/h) */
    float    course_true;    /* Course over ground (degrees true) */

    /* Quality */
    uint8_t  fix_quality;    /* 0=none, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float */
    uint8_t  num_satellites; /* Satellites used in fix */
    float    hdop;           /* Horizontal dilution of precision */

    /* RMC status */
    char     rmc_status;     /* A=valid, V=warning */

    /* Timestamps (HAL_GetTick() at last update) */
    uint32_t last_gga_tick;
    uint32_t last_rmc_tick;
    uint32_t last_ths_tick;
    uint32_t last_vtg_tick;

    /* Init state */
    bool     initialized;
    bool     version_received;

    /* ---- Internal parser state (not for external use) ---- */

    /* Ring buffer */
    uint8_t  rx_buf[UM982_RX_BUF_SIZE];
    uint16_t rx_head;        /* Write index */
    uint16_t rx_tail;        /* Read index */

    /* Line assembler */
    char     line_buf[UM982_LINE_BUF_SIZE];
    uint8_t  line_len;
    bool     line_overflow;  /* Current line exceeded buffer */

    /* UART handle */
    UART_HandleTypeDef *huart;

} UM982_GPS_t;

/* ========================= Public API ================================ */

/**
 * Initialize the UM982_GPS_t structure and configure the module.
 *
 * Sends: UNLOG, GPGGA, GPRMC, GPTHS, CONFIG HEADING, SAVECONFIG
 * Queries VERSIONA to verify communication.
 *
 * @param gps          Pointer to UM982_GPS_t instance
 * @param huart        UART handle (e.g. &huart5)
 * @param baseline_cm  Distance between antennas in cm (0 = use module default)
 * @param tolerance_cm Baseline tolerance in cm (0 = use module default)
 * @return true if VERSIONA response received within timeout
 */
bool um982_init(UM982_GPS_t *gps, UART_HandleTypeDef *huart,
                float baseline_cm, float tolerance_cm);

/**
 * Process available UART data. Call from main loop — non-blocking.
 *
 * Reads all available bytes from UART, assembles lines, and dispatches
 * complete NMEA sentences to the appropriate parser.
 *
 * @param gps  Pointer to UM982_GPS_t instance
 */
void um982_process(UM982_GPS_t *gps);

/**
 * Feed raw bytes directly into the parser (useful for testing).
 * In production, um982_process() calls this internally after UART read.
 *
 * @param gps   Pointer to UM982_GPS_t instance
 * @param data  Pointer to byte array
 * @param len   Number of bytes
 */
void um982_feed(UM982_GPS_t *gps, const uint8_t *data, uint16_t len);

/* ---- Getters ---- */

static inline float    um982_get_heading(const UM982_GPS_t *gps)     { return gps->heading; }
static inline double   um982_get_latitude(const UM982_GPS_t *gps)    { return gps->latitude; }
static inline double   um982_get_longitude(const UM982_GPS_t *gps)   { return gps->longitude; }
static inline float    um982_get_altitude(const UM982_GPS_t *gps)    { return gps->altitude; }
static inline uint8_t  um982_get_fix_quality(const UM982_GPS_t *gps) { return gps->fix_quality; }
static inline uint8_t  um982_get_num_sats(const UM982_GPS_t *gps)    { return gps->num_satellites; }
static inline float    um982_get_hdop(const UM982_GPS_t *gps)        { return gps->hdop; }
static inline float    um982_get_speed_knots(const UM982_GPS_t *gps) { return gps->speed_knots; }
static inline float    um982_get_speed_kmh(const UM982_GPS_t *gps)   { return gps->speed_kmh; }
static inline float    um982_get_course(const UM982_GPS_t *gps)      { return gps->course_true; }

/**
 * Check if heading is valid (mode A or D, and within timeout).
 */
bool um982_is_heading_valid(const UM982_GPS_t *gps);

/**
 * Check if position is valid (fix_quality > 0, and within timeout).
 */
bool um982_is_position_valid(const UM982_GPS_t *gps);

/**
 * Get age of last heading update in milliseconds.
 */
uint32_t um982_heading_age(const UM982_GPS_t *gps);

/**
 * Get age of last position update in milliseconds.
 */
uint32_t um982_position_age(const UM982_GPS_t *gps);

/* ========================= Internal (exposed for testing) ============ */

/**
 * Verify NMEA checksum. Returns true if valid.
 * Sentence must start with '$' and contain '*XX' before termination.
 */
bool um982_verify_checksum(const char *sentence);

/**
 * Parse a complete NMEA line (with $ prefix and *XX checksum).
 * Dispatches to GGA/RMC/THS/VTG parsers as appropriate.
 */
void um982_parse_sentence(UM982_GPS_t *gps, const char *sentence);

/**
 * Parse NMEA coordinate string to decimal degrees.
 * Works for both latitude (ddmm.mmmm) and longitude (dddmm.mmmm)
 * by detecting the decimal point position.
 *
 * @param field     NMEA coordinate field (e.g. "4404.14036" or "12118.85961")
 * @param hemisphere 'N', 'S', 'E', or 'W'
 * @return Decimal degrees (negative for S/W), or NAN on parse error
 */
double um982_parse_coord(const char *field, char hemisphere);

/**
 * Send a command to the UM982. Appends \r\n automatically.
 * @return true if UART transmit succeeded
 */
bool um982_send_command(UM982_GPS_t *gps, const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* UM982_GPS_H */
