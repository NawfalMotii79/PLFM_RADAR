/*******************************************************************************
 * test_um982_gps.c -- Unit tests for UM982 GPS driver
 *
 * Tests NMEA parsing, checksum validation, coordinate parsing, init sequence,
 * and validity tracking. Uses the mock HAL infrastructure for UART.
 *
 * Build: see Makefile target test_um982_gps
 * Run:   ./test_um982_gps
 ******************************************************************************/
#include "stm32_hal_mock.h"
#include "../9_1_3_C_Cpp_Code/um982_gps.h"
#include "../9_1_3_C_Cpp_Code/um982_gps.c"  /* Include .c directly for white-box testing */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

/* ========================= Test helpers ============================== */

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    do { printf("  [TEST] %-55s ", name); } while(0)

#define PASS() \
    do { printf("PASS\n"); tests_passed++; } while(0)

#define FAIL(msg) \
    do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)

#define ASSERT_TRUE(expr, msg) \
    do { if (!(expr)) { FAIL(msg); return; } } while(0)

#define ASSERT_FALSE(expr, msg) \
    do { if (expr) { FAIL(msg); return; } } while(0)

#define ASSERT_EQ_INT(a, b, msg) \
    do { if ((a) != (b)) { \
        char _buf[256]; \
        snprintf(_buf, sizeof(_buf), "%s (got %d, expected %d)", msg, (int)(a), (int)(b)); \
        FAIL(_buf); return; \
    } } while(0)

#define ASSERT_NEAR(a, b, tol, msg) \
    do { if (fabs((double)(a) - (double)(b)) > (tol)) { \
        char _buf[256]; \
        snprintf(_buf, sizeof(_buf), "%s (got %.8f, expected %.8f)", msg, (double)(a), (double)(b)); \
        FAIL(_buf); return; \
    } } while(0)

#define ASSERT_NAN(val, msg) \
    do { if (!isnan(val)) { FAIL(msg); return; } } while(0)

static UM982_GPS_t gps;

static void reset_gps(void)
{
    spy_reset();
    memset(&gps, 0, sizeof(gps));
    gps.huart = &huart5;
    gps.heading = NAN;
    gps.heading_mode = 'V';
    gps.rmc_status = 'V';
}

/* ========================= Checksum tests ============================ */

static void test_checksum_valid(void)
{
    TEST("checksum: valid GGA");
    ASSERT_TRUE(um982_verify_checksum(
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47"),
        "should be valid");
    PASS();
}

static void test_checksum_valid_ths(void)
{
    TEST("checksum: valid THS");
    ASSERT_TRUE(um982_verify_checksum("$GNTHS,341.3344,A*1F"),
        "should be valid");
    PASS();
}

static void test_checksum_invalid(void)
{
    TEST("checksum: invalid (wrong value)");
    ASSERT_FALSE(um982_verify_checksum("$GNTHS,341.3344,A*FF"),
        "should be invalid");
    PASS();
}

static void test_checksum_missing_star(void)
{
    TEST("checksum: missing * marker");
    ASSERT_FALSE(um982_verify_checksum("$GNTHS,341.3344,A"),
        "should be invalid");
    PASS();
}

static void test_checksum_null(void)
{
    TEST("checksum: NULL input");
    ASSERT_FALSE(um982_verify_checksum(NULL), "should be false");
    PASS();
}

static void test_checksum_no_dollar(void)
{
    TEST("checksum: missing $ prefix");
    ASSERT_FALSE(um982_verify_checksum("GNTHS,341.3344,A*1F"),
        "should be invalid without $");
    PASS();
}

/* ========================= Coordinate parsing tests ================== */

static void test_coord_latitude_north(void)
{
    TEST("coord: latitude 4404.14036 N");
    double lat = um982_parse_coord("4404.14036", 'N');
    /* 44 + 04.14036/60 = 44.069006 */
    ASSERT_NEAR(lat, 44.069006, 0.000001, "latitude");
    PASS();
}

static void test_coord_latitude_south(void)
{
    TEST("coord: latitude 3358.92500 S (negative)");
    double lat = um982_parse_coord("3358.92500", 'S');
    ASSERT_TRUE(lat < 0.0, "should be negative for S");
    ASSERT_NEAR(lat, -(33.0 + 58.925/60.0), 0.000001, "latitude");
    PASS();
}

static void test_coord_longitude_3digit(void)
{
    TEST("coord: longitude 12118.85961 W (3-digit degrees)");
    double lon = um982_parse_coord("12118.85961", 'W');
    /* 121 + 18.85961/60 = 121.314327 */
    ASSERT_TRUE(lon < 0.0, "should be negative for W");
    ASSERT_NEAR(lon, -(121.0 + 18.85961/60.0), 0.000001, "longitude");
    PASS();
}

static void test_coord_longitude_east(void)
{
    TEST("coord: longitude 11614.19729 E");
    double lon = um982_parse_coord("11614.19729", 'E');
    ASSERT_TRUE(lon > 0.0, "should be positive for E");
    ASSERT_NEAR(lon, 116.0 + 14.19729/60.0, 0.000001, "longitude");
    PASS();
}

static void test_coord_empty(void)
{
    TEST("coord: empty string returns NAN");
    ASSERT_NAN(um982_parse_coord("", 'N'), "should be NAN");
    PASS();
}

static void test_coord_null(void)
{
    TEST("coord: NULL returns NAN");
    ASSERT_NAN(um982_parse_coord(NULL, 'N'), "should be NAN");
    PASS();
}

static void test_coord_no_dot(void)
{
    TEST("coord: no decimal point returns NAN");
    ASSERT_NAN(um982_parse_coord("440414036", 'N'), "should be NAN");
    PASS();
}

/* ========================= GGA parsing tests ========================= */

static void test_parse_gga_full(void)
{
    TEST("GGA: full sentence with all fields");
    reset_gps();
    mock_set_tick(1000);

    um982_parse_sentence(&gps,
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47");

    ASSERT_NEAR(gps.latitude, 44.069006, 0.0001, "latitude");
    ASSERT_NEAR(gps.longitude, -(121.0 + 18.85961/60.0), 0.0001, "longitude");
    ASSERT_EQ_INT(gps.fix_quality, 1, "fix quality");
    ASSERT_EQ_INT(gps.num_satellites, 12, "num sats");
    ASSERT_NEAR(gps.hdop, 0.98, 0.01, "hdop");
    ASSERT_NEAR(gps.altitude, 1113.0, 0.1, "altitude");
    ASSERT_NEAR(gps.geoid_sep, -21.3, 0.1, "geoid sep");
    PASS();
}

static void test_parse_gga_rtk_fixed(void)
{
    TEST("GGA: RTK fixed (quality=4)");
    reset_gps();

    um982_parse_sentence(&gps,
        "$GNGGA,023634.00,4004.73871635,N,11614.19729418,E,4,28,0.7,61.0988,M,-8.4923,M,,*5D");

    ASSERT_EQ_INT(gps.fix_quality, 4, "RTK fixed");
    ASSERT_EQ_INT(gps.num_satellites, 28, "num sats");
    ASSERT_NEAR(gps.latitude, 40.0 + 4.73871635/60.0, 0.0000001, "latitude");
    ASSERT_NEAR(gps.longitude, 116.0 + 14.19729418/60.0, 0.0000001, "longitude");
    PASS();
}

static void test_parse_gga_no_fix(void)
{
    TEST("GGA: no fix (quality=0)");
    reset_gps();

    /* Compute checksum for this sentence */
    um982_parse_sentence(&gps,
        "$GNGGA,235959.00,,,,,0,00,99.99,,,,,,*79");

    ASSERT_EQ_INT(gps.fix_quality, 0, "no fix");
    PASS();
}

/* ========================= RMC parsing tests ========================= */

static void test_parse_rmc_valid(void)
{
    TEST("RMC: valid position and speed");
    reset_gps();
    mock_set_tick(2000);

    um982_parse_sentence(&gps,
        "$GNRMC,001031.00,A,4404.13993,N,12118.86023,W,0.146,,100117,,,A*7B");

    ASSERT_EQ_INT(gps.rmc_status, 'A', "status");
    ASSERT_NEAR(gps.latitude, 44.0 + 4.13993/60.0, 0.0001, "latitude");
    ASSERT_NEAR(gps.longitude, -(121.0 + 18.86023/60.0), 0.0001, "longitude");
    ASSERT_NEAR(gps.speed_knots, 0.146, 0.001, "speed");
    PASS();
}

static void test_parse_rmc_void(void)
{
    TEST("RMC: void status (no valid fix)");
    reset_gps();
    gps.latitude = 12.34;  /* Pre-set to check it doesn't get overwritten */

    um982_parse_sentence(&gps,
        "$GNRMC,235959.00,V,,,,,,,100117,,,N*64");

    ASSERT_EQ_INT(gps.rmc_status, 'V', "void status");
    ASSERT_NEAR(gps.latitude, 12.34, 0.001, "lat should not change on void");
    PASS();
}

/* ========================= THS parsing tests ========================= */

static void test_parse_ths_autonomous(void)
{
    TEST("THS: autonomous heading 341.3344");
    reset_gps();
    mock_set_tick(3000);

    um982_parse_sentence(&gps, "$GNTHS,341.3344,A*1F");

    ASSERT_NEAR(gps.heading, 341.3344, 0.001, "heading");
    ASSERT_EQ_INT(gps.heading_mode, 'A', "mode");
    PASS();
}

static void test_parse_ths_not_valid(void)
{
    TEST("THS: not valid mode");
    reset_gps();

    um982_parse_sentence(&gps, "$GNTHS,,V*10");

    ASSERT_NAN(gps.heading, "heading should be NAN when empty");
    ASSERT_EQ_INT(gps.heading_mode, 'V', "mode V");
    PASS();
}

static void test_parse_ths_zero(void)
{
    TEST("THS: heading exactly 0.0000");
    reset_gps();

    um982_parse_sentence(&gps, "$GNTHS,0.0000,A*19");

    ASSERT_NEAR(gps.heading, 0.0, 0.001, "heading zero");
    ASSERT_EQ_INT(gps.heading_mode, 'A', "mode A");
    PASS();
}

static void test_parse_ths_360_boundary(void)
{
    TEST("THS: heading near 360");
    reset_gps();

    um982_parse_sentence(&gps, "$GNTHS,359.9999,D*13");

    ASSERT_NEAR(gps.heading, 359.9999, 0.001, "heading near 360");
    ASSERT_EQ_INT(gps.heading_mode, 'D', "mode D");
    PASS();
}

/* ========================= VTG parsing tests ========================= */

static void test_parse_vtg(void)
{
    TEST("VTG: course and speed");
    reset_gps();

    um982_parse_sentence(&gps,
        "$GPVTG,220.86,T,,M,2.550,N,4.724,K,A*34");

    ASSERT_NEAR(gps.course_true, 220.86, 0.01, "course");
    ASSERT_NEAR(gps.speed_knots, 2.550, 0.001, "speed knots");
    ASSERT_NEAR(gps.speed_kmh, 4.724, 0.001, "speed kmh");
    PASS();
}

/* ========================= Talker ID tests =========================== */

static void test_talker_gp(void)
{
    TEST("talker: GP prefix parses correctly");
    reset_gps();

    um982_parse_sentence(&gps, "$GPTHS,123.4567,A*07");

    ASSERT_NEAR(gps.heading, 123.4567, 0.001, "heading with GP");
    PASS();
}

static void test_talker_gl(void)
{
    TEST("talker: GL prefix parses correctly");
    reset_gps();

    um982_parse_sentence(&gps, "$GLTHS,123.4567,A*1B");

    ASSERT_NEAR(gps.heading, 123.4567, 0.001, "heading with GL");
    PASS();
}

/* ========================= Feed / line assembly tests ================ */

static void test_feed_single_sentence(void)
{
    TEST("feed: single complete sentence with CRLF");
    reset_gps();
    mock_set_tick(5000);

    const char *data = "$GNTHS,341.3344,A*1F\r\n";
    um982_feed(&gps, (const uint8_t *)data, (uint16_t)strlen(data));

    ASSERT_NEAR(gps.heading, 341.3344, 0.001, "heading");
    PASS();
}

static void test_feed_multiple_sentences(void)
{
    TEST("feed: multiple sentences in one chunk");
    reset_gps();
    mock_set_tick(5000);

    const char *data =
        "$GNTHS,100.0000,A*18\r\n"
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47\r\n";
    um982_feed(&gps, (const uint8_t *)data, (uint16_t)strlen(data));

    ASSERT_NEAR(gps.heading, 100.0, 0.01, "heading from THS");
    ASSERT_EQ_INT(gps.fix_quality, 1, "fix from GGA");
    PASS();
}

static void test_feed_partial_then_complete(void)
{
    TEST("feed: partial bytes then complete");
    reset_gps();
    mock_set_tick(5000);

    const char *part1 = "$GNTHS,200.";
    const char *part2 = "5000,A*1E\r\n";
    um982_feed(&gps, (const uint8_t *)part1, (uint16_t)strlen(part1));
    /* Heading should not be set yet */
    ASSERT_NAN(gps.heading, "should be NAN before complete");

    um982_feed(&gps, (const uint8_t *)part2, (uint16_t)strlen(part2));
    ASSERT_NEAR(gps.heading, 200.5, 0.01, "heading after complete");
    PASS();
}

static void test_feed_bad_checksum_rejected(void)
{
    TEST("feed: bad checksum sentence is rejected");
    reset_gps();
    mock_set_tick(5000);

    const char *data = "$GNTHS,999.0000,A*FF\r\n";
    um982_feed(&gps, (const uint8_t *)data, (uint16_t)strlen(data));

    ASSERT_NAN(gps.heading, "heading should remain NAN");
    PASS();
}

static void test_feed_versiona_response(void)
{
    TEST("feed: VERSIONA response sets flag");
    reset_gps();

    const char *data = "#VERSIONA,79,GPS,FINE,2326,378237000,15434,0,18,889;\"UM982\"\r\n";
    um982_feed(&gps, (const uint8_t *)data, (uint16_t)strlen(data));

    ASSERT_TRUE(gps.version_received, "version_received should be true");
    PASS();
}

/* ========================= Validity / age tests ====================== */

static void test_heading_valid_within_timeout(void)
{
    TEST("validity: heading valid within timeout");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps, "$GNTHS,341.3344,A*1F");

    /* Still at tick 10000 */
    ASSERT_TRUE(um982_is_heading_valid(&gps), "should be valid");
    PASS();
}

static void test_heading_invalid_after_timeout(void)
{
    TEST("validity: heading invalid after 2s timeout");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps, "$GNTHS,341.3344,A*1F");

    /* Advance past timeout */
    mock_set_tick(12500);
    ASSERT_FALSE(um982_is_heading_valid(&gps), "should be invalid after 2.5s");
    PASS();
}

static void test_heading_invalid_mode_v(void)
{
    TEST("validity: heading invalid with mode V");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps, "$GNTHS,,V*10");

    ASSERT_FALSE(um982_is_heading_valid(&gps), "mode V is invalid");
    PASS();
}

static void test_position_valid(void)
{
    TEST("validity: position valid with fix quality 1");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps,
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47");

    ASSERT_TRUE(um982_is_position_valid(&gps), "should be valid");
    PASS();
}

static void test_position_invalid_no_fix(void)
{
    TEST("validity: position invalid with no fix");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps,
        "$GNGGA,235959.00,,,,,0,00,99.99,,,,,,*79");

    ASSERT_FALSE(um982_is_position_valid(&gps), "no fix = invalid");
    PASS();
}

static void test_heading_age(void)
{
    TEST("age: heading age computed correctly");
    reset_gps();
    mock_set_tick(10000);

    um982_parse_sentence(&gps, "$GNTHS,341.3344,A*1F");

    mock_set_tick(10500);
    uint32_t age = um982_heading_age(&gps);
    ASSERT_EQ_INT(age, 500, "age should be 500ms");
    PASS();
}

/* ========================= Send command tests ======================== */

static void test_send_command_appends_crlf(void)
{
    TEST("send_command: appends \\r\\n");
    reset_gps();

    um982_send_command(&gps, "GPGGA COM1 1");

    /* Check that TX buffer contains "GPGGA COM1 1\r\n" */
    const char *expected = "GPGGA COM1 1\r\n";
    ASSERT_TRUE(mock_uart_tx_len == strlen(expected), "TX length");
    ASSERT_TRUE(memcmp(mock_uart_tx_buf, expected, strlen(expected)) == 0,
        "TX content should be 'GPGGA COM1 1\\r\\n'");
    PASS();
}

static void test_send_command_null_safety(void)
{
    TEST("send_command: NULL gps returns false");
    ASSERT_FALSE(um982_send_command(NULL, "RESET"), "should return false");
    PASS();
}

/* ========================= Init sequence tests ======================= */

static void test_init_sends_correct_commands(void)
{
    TEST("init: sends correct command sequence");
    spy_reset();
    mock_uart_tx_clear();

    /* Pre-load VERSIONA response so init succeeds */
    const char *ver_resp = "#VERSIONA,79,GPS,FINE,2326,378237000,15434,0,18,889;\"UM982\"\r\n";
    mock_uart_rx_load(&huart5, (const uint8_t *)ver_resp, (uint16_t)strlen(ver_resp));

    UM982_GPS_t init_gps;
    bool ok = um982_init(&init_gps, &huart5, 50.0f, 3.0f);

    ASSERT_TRUE(ok, "init should succeed");
    ASSERT_TRUE(init_gps.initialized, "should be initialized");

    /* Verify TX buffer contains expected commands */
    const char *tx = (const char *)mock_uart_tx_buf;
    ASSERT_TRUE(strstr(tx, "UNLOG\r\n") != NULL, "should send UNLOG");
    ASSERT_TRUE(strstr(tx, "CONFIG HEADING FIXLENGTH\r\n") != NULL, "should send CONFIG HEADING");
    ASSERT_TRUE(strstr(tx, "CONFIG HEADING LENGTH 50 3\r\n") != NULL, "should send LENGTH");
    ASSERT_TRUE(strstr(tx, "GPGGA COM1 1\r\n") != NULL, "should enable GGA");
    ASSERT_TRUE(strstr(tx, "GPRMC COM1 1\r\n") != NULL, "should enable RMC");
    ASSERT_TRUE(strstr(tx, "GPTHS COM1 0.2\r\n") != NULL, "should enable THS at 5Hz");
    ASSERT_TRUE(strstr(tx, "SAVECONFIG\r\n") == NULL, "should NOT save config (NVM wear)");
    ASSERT_TRUE(strstr(tx, "VERSIONA\r\n") != NULL, "should query version");

    /* Verify command order: UNLOG should come before GPGGA */
    const char *unlog_pos = strstr(tx, "UNLOG\r\n");
    const char *gpgga_pos = strstr(tx, "GPGGA COM1 1\r\n");
    ASSERT_TRUE(unlog_pos < gpgga_pos, "UNLOG should precede GPGGA");

    PASS();
}

static void test_init_no_baseline(void)
{
    TEST("init: baseline=0 skips LENGTH command");
    spy_reset();
    mock_uart_tx_clear();

    const char *ver_resp = "#VERSIONA,79,GPS,FINE,2326,378237000,15434,0,18,889;\"UM982\"\r\n";
    mock_uart_rx_load(&huart5, (const uint8_t *)ver_resp, (uint16_t)strlen(ver_resp));

    UM982_GPS_t init_gps;
    um982_init(&init_gps, &huart5, 0.0f, 0.0f);

    const char *tx = (const char *)mock_uart_tx_buf;
    ASSERT_TRUE(strstr(tx, "CONFIG HEADING LENGTH") == NULL, "should NOT send LENGTH");
    PASS();
}

static void test_init_fails_no_version(void)
{
    TEST("init: fails if no VERSIONA response");
    spy_reset();
    mock_uart_tx_clear();

    /* Don't load any RX data — init should timeout */
    UM982_GPS_t init_gps;
    bool ok = um982_init(&init_gps, &huart5, 50.0f, 3.0f);

    ASSERT_FALSE(ok, "init should fail without version response");
    ASSERT_FALSE(init_gps.initialized, "should not be initialized");
    PASS();
}

/* ========================= Edge case tests =========================== */

static void test_empty_fields_handled(void)
{
    TEST("edge: GGA with empty lat/lon fields");
    reset_gps();
    gps.latitude = 99.99;
    gps.longitude = 99.99;

    /* GGA with empty position fields (no fix) */
    um982_parse_sentence(&gps,
        "$GNGGA,235959.00,,,,,0,00,99.99,,,,,,*79");

    ASSERT_EQ_INT(gps.fix_quality, 0, "no fix");
    /* Latitude/longitude should not be updated (fields are empty) */
    ASSERT_NEAR(gps.latitude, 99.99, 0.01, "lat unchanged");
    ASSERT_NEAR(gps.longitude, 99.99, 0.01, "lon unchanged");
    PASS();
}

static void test_sentence_too_short(void)
{
    TEST("edge: sentence too short to have formatter");
    reset_gps();
    /* Should not crash */
    um982_parse_sentence(&gps, "$GN");
    um982_parse_sentence(&gps, "$");
    um982_parse_sentence(&gps, "");
    um982_parse_sentence(&gps, NULL);
    PASS();
}

static void test_line_overflow(void)
{
    TEST("edge: oversized line is dropped");
    reset_gps();

    /* Create a line longer than UM982_LINE_BUF_SIZE */
    char big[200];
    memset(big, 'X', sizeof(big));
    big[0] = '$';
    big[198] = '\n';
    big[199] = '\0';

    um982_feed(&gps, (const uint8_t *)big, 199);
    /* Should not crash, heading should still be NAN */
    ASSERT_NAN(gps.heading, "no valid data from overflow");
    PASS();
}

static void test_process_via_mock_uart(void)
{
    TEST("process: reads from mock UART RX buffer");
    reset_gps();
    mock_set_tick(5000);

    /* Load data into mock UART RX */
    const char *data = "$GNTHS,275.1234,D*18\r\n";
    mock_uart_rx_load(&huart5, (const uint8_t *)data, (uint16_t)strlen(data));

    /* Call process() which reads from UART */
    um982_process(&gps);

    ASSERT_NEAR(gps.heading, 275.1234, 0.001, "heading via process()");
    ASSERT_EQ_INT(gps.heading_mode, 'D', "mode D");
    PASS();
}

/* ========================= PR #68 bug regression tests =============== */

/* These tests specifically verify the bugs found in the reverted PR #68 */

static void test_regression_sentence_id_with_gn_prefix(void)
{
    TEST("regression: GN-prefixed GGA is correctly identified");
    reset_gps();

    /* PR #68 bug: strncmp(sentence, "GGA", 3) compared "GNG" vs "GGA" — never matched.
     * Our fix: skip 2-char talker ID, compare at sentence+3. */
    um982_parse_sentence(&gps,
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47");

    ASSERT_EQ_INT(gps.fix_quality, 1, "GGA should parse with GN prefix");
    ASSERT_NEAR(gps.latitude, 44.069006, 0.001, "latitude should be parsed");
    PASS();
}

static void test_regression_longitude_3digit_degrees(void)
{
    TEST("regression: 3-digit longitude degrees parsed correctly");
    reset_gps();

    /* PR #68 bug: hardcoded 2-digit degrees for longitude.
     * 12118.85961 should be 121° 18.85961' = 121.314327° */
    um982_parse_sentence(&gps,
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47");

    ASSERT_NEAR(gps.longitude, -(121.0 + 18.85961/60.0), 0.0001,
        "longitude 121° should not be parsed as 12°");
    ASSERT_TRUE(gps.longitude < -100.0, "longitude should be > 100 degrees");
    PASS();
}

static void test_regression_hemisphere_no_ptr_corrupt(void)
{
    TEST("regression: hemisphere parsing doesn't corrupt field pointer");
    reset_gps();

    /* PR #68 bug: GGA/RMC hemisphere cases manually advanced ptr,
     * desynchronizing from field counter. Our parser uses proper tokenizer. */
    um982_parse_sentence(&gps,
        "$GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47");

    /* After lat/lon, remaining fields should be correct */
    ASSERT_EQ_INT(gps.num_satellites, 12, "sats after hemisphere");
    ASSERT_NEAR(gps.hdop, 0.98, 0.01, "hdop after hemisphere");
    ASSERT_NEAR(gps.altitude, 1113.0, 0.1, "altitude after hemisphere");
    PASS();
}

static void test_regression_rmc_also_parsed(void)
{
    TEST("regression: RMC sentence is actually parsed (not dead code)");
    reset_gps();

    /* PR #68 bug: identifySentence never matched GGA/RMC, so position
     * parsing was dead code. */
    um982_parse_sentence(&gps,
        "$GNRMC,001031.00,A,4404.13993,N,12118.86023,W,0.146,,100117,,,A*7B");

    ASSERT_TRUE(gps.latitude > 44.0, "RMC lat should be parsed");
    ASSERT_TRUE(gps.longitude < -121.0, "RMC lon should be parsed");
    ASSERT_NEAR(gps.speed_knots, 0.146, 0.001, "RMC speed");
    PASS();
}

/* ========================= Main ====================================== */

int main(void)
{
    printf("=== UM982 GPS Driver Tests ===\n\n");

    printf("--- Checksum ---\n");
    test_checksum_valid();
    test_checksum_valid_ths();
    test_checksum_invalid();
    test_checksum_missing_star();
    test_checksum_null();
    test_checksum_no_dollar();

    printf("\n--- Coordinate Parsing ---\n");
    test_coord_latitude_north();
    test_coord_latitude_south();
    test_coord_longitude_3digit();
    test_coord_longitude_east();
    test_coord_empty();
    test_coord_null();
    test_coord_no_dot();

    printf("\n--- GGA Parsing ---\n");
    test_parse_gga_full();
    test_parse_gga_rtk_fixed();
    test_parse_gga_no_fix();

    printf("\n--- RMC Parsing ---\n");
    test_parse_rmc_valid();
    test_parse_rmc_void();

    printf("\n--- THS Parsing ---\n");
    test_parse_ths_autonomous();
    test_parse_ths_not_valid();
    test_parse_ths_zero();
    test_parse_ths_360_boundary();

    printf("\n--- VTG Parsing ---\n");
    test_parse_vtg();

    printf("\n--- Talker IDs ---\n");
    test_talker_gp();
    test_talker_gl();

    printf("\n--- Feed / Line Assembly ---\n");
    test_feed_single_sentence();
    test_feed_multiple_sentences();
    test_feed_partial_then_complete();
    test_feed_bad_checksum_rejected();
    test_feed_versiona_response();

    printf("\n--- Validity / Age ---\n");
    test_heading_valid_within_timeout();
    test_heading_invalid_after_timeout();
    test_heading_invalid_mode_v();
    test_position_valid();
    test_position_invalid_no_fix();
    test_heading_age();

    printf("\n--- Send Command ---\n");
    test_send_command_appends_crlf();
    test_send_command_null_safety();

    printf("\n--- Init Sequence ---\n");
    test_init_sends_correct_commands();
    test_init_no_baseline();
    test_init_fails_no_version();

    printf("\n--- Edge Cases ---\n");
    test_empty_fields_handled();
    test_sentence_too_short();
    test_line_overflow();
    test_process_via_mock_uart();

    printf("\n--- PR #68 Regression ---\n");
    test_regression_sentence_id_with_gn_prefix();
    test_regression_longitude_3digit_degrees();
    test_regression_hemisphere_no_ptr_corrupt();
    test_regression_rmc_also_parsed();

    printf("\n===============================================\n");
    printf("  Results: %d passed, %d failed (of %d total)\n",
           tests_passed, tests_failed, tests_passed + tests_failed);
    printf("===============================================\n");

    return tests_failed > 0 ? 1 : 0;
}
