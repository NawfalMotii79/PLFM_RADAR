// test_agc_outer_loop.cpp -- C++ unit tests for ADAR1000_AGC outer-loop AGC
//
// Tests the STM32 outer-loop AGC class that adjusts ADAR1000 VGA gain based
// on the FPGA's saturation flag.  Uses the existing HAL mock/spy framework.
//
// Build: c++ -std=c++17 ... (see Makefile TESTS_WITH_CXX rule)

#include <cassert>
#include <cstdio>
#include <cstring>

// Shim headers override real STM32/diag headers
#include "stm32_hal_mock.h"
#include "ADAR1000_AGC.h"
#include "ADAR1000_Manager.h"

// ---------------------------------------------------------------------------
// Linker symbols required by ADAR1000_Manager.cpp (pulled in via main.h shim)
// ---------------------------------------------------------------------------
uint8_t GUI_start_flag_received = 0;
uint8_t USB_Buffer[64] = {0};
extern "C" void Error_Handler(void) {}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int tests_passed = 0;
static int tests_total  = 0;

#define RUN_TEST(fn)                                                           \
    do {                                                                        \
        tests_total++;                                                          \
        printf("  [%2d] %-55s ", tests_total, #fn);                            \
        fn();                                                                   \
        tests_passed++;                                                         \
        printf("PASS\n");                                                       \
    } while (0)

// ---------------------------------------------------------------------------
// Test 1: Default construction matches design spec
// ---------------------------------------------------------------------------
static void test_defaults()
{
    ADAR1000_AGC agc;

    assert(agc.agc_base_gain == 30);  // kDefaultRxVgaGain
    assert(agc.gain_step_down == 4);
    assert(agc.gain_step_up == 1);
    assert(agc.min_gain == 0);
    assert(agc.max_gain == 127);
    assert(agc.holdoff_frames == 4);
    assert(agc.enabled == false);  // disabled by default — FPGA DIG_6 is source of truth
    assert(agc.holdoff_counter == 0);
    assert(agc.last_saturated == false);
    assert(agc.saturation_event_count == 0);

    // All cal offsets zero
    for (int i = 0; i < AGC_TOTAL_CHANNELS; ++i) {
        assert(agc.cal_offset[i] == 0);
    }
}

// ---------------------------------------------------------------------------
// Test 2: Saturation reduces gain by step_down
// ---------------------------------------------------------------------------
static void test_saturation_reduces_gain()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    uint8_t initial = agc.agc_base_gain;  // 30

    agc.update(true);  // saturation

    assert(agc.agc_base_gain == initial - agc.gain_step_down);  // 26
    assert(agc.last_saturated == true);
    assert(agc.holdoff_counter == 0);
}

// ---------------------------------------------------------------------------
// Test 3: Holdoff prevents premature gain-up
// ---------------------------------------------------------------------------
static void test_holdoff_prevents_early_gain_up()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.update(true);  // saturate once -> gain = 26
    uint8_t after_sat = agc.agc_base_gain;

    // Feed (holdoff_frames - 1) clear frames — should NOT increase gain
    for (uint8_t i = 0; i < agc.holdoff_frames - 1; ++i) {
        agc.update(false);
        assert(agc.agc_base_gain == after_sat);
    }

    // holdoff_counter should be holdoff_frames - 1
    assert(agc.holdoff_counter == agc.holdoff_frames - 1);
}

// ---------------------------------------------------------------------------
// Test 4: Recovery after holdoff period
// ---------------------------------------------------------------------------
static void test_recovery_after_holdoff()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.update(true);  // saturate -> gain = 26
    uint8_t after_sat = agc.agc_base_gain;

    // Feed exactly holdoff_frames clear frames
    for (uint8_t i = 0; i < agc.holdoff_frames; ++i) {
        agc.update(false);
    }

    assert(agc.agc_base_gain == after_sat + agc.gain_step_up);  // 27
    assert(agc.holdoff_counter == 0);  // reset after recovery
}

// ---------------------------------------------------------------------------
// Test 5: Min gain clamping
// ---------------------------------------------------------------------------
static void test_min_gain_clamp()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.min_gain = 10;
    agc.agc_base_gain = 12;
    agc.gain_step_down = 4;

    agc.update(true);  // 12 - 4 = 8, but min = 10
    assert(agc.agc_base_gain == 10);

    agc.update(true);  // already at min
    assert(agc.agc_base_gain == 10);
}

// ---------------------------------------------------------------------------
// Test 6: Max gain clamping
// ---------------------------------------------------------------------------
static void test_max_gain_clamp()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.max_gain = 32;
    agc.agc_base_gain = 31;
    agc.gain_step_up = 2;
    agc.holdoff_frames = 1;  // immediate recovery

    agc.update(false);  // 31 + 2 = 33, but max = 32
    assert(agc.agc_base_gain == 32);

    agc.update(false);  // already at max
    assert(agc.agc_base_gain == 32);
}

// ---------------------------------------------------------------------------
// Test 7: Per-channel calibration offsets
// ---------------------------------------------------------------------------
static void test_calibration_offsets()
{
    ADAR1000_AGC agc;
    agc.agc_base_gain = 30;
    agc.min_gain = 0;
    agc.max_gain = 60;

    agc.cal_offset[0]  =  5;   // 30 + 5  = 35
    agc.cal_offset[1]  = -10;  // 30 - 10 = 20
    agc.cal_offset[15] =  40;  // 30 + 40 = 60 (clamped to max)

    assert(agc.effectiveGain(0) == 35);
    assert(agc.effectiveGain(1) == 20);
    assert(agc.effectiveGain(15) == 60);  // clamped to max_gain

    // Negative clamp
    agc.cal_offset[2] = -50;   // 30 - 50 = -20, clamped to min_gain = 0
    assert(agc.effectiveGain(2) == 0);

    // Out-of-range index returns min_gain
    assert(agc.effectiveGain(16) == agc.min_gain);
}

// ---------------------------------------------------------------------------
// Test 8: Disabled AGC is a no-op
// ---------------------------------------------------------------------------
static void test_disabled_noop()
{
    ADAR1000_AGC agc;
    agc.enabled = false;
    uint8_t original = agc.agc_base_gain;

    agc.update(true);   // should be ignored
    assert(agc.agc_base_gain == original);
    assert(agc.last_saturated == false);  // not updated when disabled
    assert(agc.saturation_event_count == 0);

    agc.update(false);  // also ignored
    assert(agc.agc_base_gain == original);
}

// ---------------------------------------------------------------------------
// Test 9: applyGain() produces correct SPI writes
// ---------------------------------------------------------------------------
static void test_apply_gain_spi()
{
    spy_reset();

    ADAR1000Manager mgr;  // creates 4 devices
    ADAR1000_AGC agc;
    agc.agc_base_gain = 42;

    agc.applyGain(mgr);

    // Each channel: adarSetRxVgaGain -> adarWrite(gain) + adarWrite(LOAD_WORKING)
    // Each adarWrite: CS_low (GPIO_WRITE) + SPI_TRANSMIT + CS_high (GPIO_WRITE)
    // = 3 spy records per adarWrite
    // = 6 spy records per channel
    // = 16 channels * 6 = 96 total spy records

    // Verify SPI transmit count: 2 SPI calls per channel * 16 channels = 32
    int spi_count = spy_count_type(SPY_SPI_TRANSMIT);
    assert(spi_count == 32);

    // Verify GPIO write count: 4 GPIO writes per channel (CS low + CS high for each of 2 adarWrite calls)
    int gpio_writes = spy_count_type(SPY_GPIO_WRITE);
    assert(gpio_writes == 64);  // 16 ch * 2 adarWrite * 2 GPIO each
}

// ---------------------------------------------------------------------------
// Test 10: resetState() clears counters but preserves config
// ---------------------------------------------------------------------------
static void test_reset_preserves_config()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.agc_base_gain = 42;
    agc.gain_step_down = 8;
    agc.cal_offset[3] = -5;

    // Generate some state
    agc.update(true);
    agc.update(true);
    assert(agc.saturation_event_count == 2);
    assert(agc.last_saturated == true);

    agc.resetState();

    // State cleared
    assert(agc.holdoff_counter == 0);
    assert(agc.last_saturated == false);
    assert(agc.saturation_event_count == 0);

    // Config preserved
    assert(agc.agc_base_gain == 42 - 8 - 8);  // two saturations applied before reset
    assert(agc.gain_step_down == 8);
    assert(agc.cal_offset[3] == -5);
}

// ---------------------------------------------------------------------------
// Test 11: Saturation counter increments correctly
// ---------------------------------------------------------------------------
static void test_saturation_counter()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test

    for (int i = 0; i < 10; ++i) {
        agc.update(true);
    }
    assert(agc.saturation_event_count == 10);

    // Clear frames don't increment saturation count
    for (int i = 0; i < 5; ++i) {
        agc.update(false);
    }
    assert(agc.saturation_event_count == 10);
}

// ---------------------------------------------------------------------------
// Test 12: Mixed saturation/clear sequence
// ---------------------------------------------------------------------------
static void test_mixed_sequence()
{
    ADAR1000_AGC agc;
    agc.enabled = true;  // default is OFF; enable for this test
    agc.agc_base_gain = 30;
    agc.gain_step_down = 4;
    agc.gain_step_up = 1;
    agc.holdoff_frames = 3;

    // Saturate: 30 -> 26
    agc.update(true);
    assert(agc.agc_base_gain == 26);
    assert(agc.holdoff_counter == 0);

    // 2 clear frames (not enough for recovery)
    agc.update(false);
    agc.update(false);
    assert(agc.agc_base_gain == 26);
    assert(agc.holdoff_counter == 2);

    // Saturate again: 26 -> 22, counter resets
    agc.update(true);
    assert(agc.agc_base_gain == 22);
    assert(agc.holdoff_counter == 0);
    assert(agc.saturation_event_count == 2);

    // 3 clear frames -> recovery: 22 -> 23
    agc.update(false);
    agc.update(false);
    agc.update(false);
    assert(agc.agc_base_gain == 23);
    assert(agc.holdoff_counter == 0);

    // 3 more clear -> 23 -> 24
    agc.update(false);
    agc.update(false);
    agc.update(false);
    assert(agc.agc_base_gain == 24);
}

// ---------------------------------------------------------------------------
// Test 13: Effective gain with edge-case base_gain values
// ---------------------------------------------------------------------------
static void test_effective_gain_edge_cases()
{
    ADAR1000_AGC agc;
    agc.min_gain = 5;
    agc.max_gain = 250;

    // Base gain at zero with positive offset
    agc.agc_base_gain = 0;
    agc.cal_offset[0] = 3;
    assert(agc.effectiveGain(0) == 5);  // 0 + 3 = 3, clamped to min_gain=5

    // Base gain at max with zero offset
    agc.agc_base_gain = 250;
    agc.cal_offset[0] = 0;
    assert(agc.effectiveGain(0) == 250);

    // Base gain at max with positive offset -> clamped
    agc.agc_base_gain = 250;
    agc.cal_offset[0] = 10;
    assert(agc.effectiveGain(0) == 250);  // clamped to max_gain
}

// ---------------------------------------------------------------------------
// Test 14: Saturation must never increase gain (issue #145 regression)
// ---------------------------------------------------------------------------
static void test_saturation_never_increases_gain()
{
    ADAR1000_AGC agc;
    agc.enabled = true;
    agc.agc_base_gain = 30;
    agc.min_gain      = 40;  // misconfigured: min > current base
    agc.gain_step_down = 4;

    uint8_t before = agc.agc_base_gain;
    agc.update(true);  // saturation event
    assert(agc.agc_base_gain <= before);  // must not increase

    // At-floor case: base == min
    agc.agc_base_gain = 40;
    agc.update(true);
    assert(agc.agc_base_gain == 40);  // no movement at floor
}

// ---------------------------------------------------------------------------
// Test 15: configure() validates min <= max and clamps agc_base_gain
// ---------------------------------------------------------------------------
static void test_configure_validation()
{
    ADAR1000_AGC agc;

    // Valid config
    assert(agc.configure(10, 120, 4, 1, 4) == true);
    assert(agc.min_gain == 10);
    assert(agc.max_gain == 120);
    assert(agc.gain_step_down == 4);
    assert(agc.gain_step_up == 1);
    assert(agc.holdoff_frames == 4);

    // Invalid: min > max — fields must be unchanged
    assert(agc.configure(100, 50, 4, 1, 4) == false);
    assert(agc.min_gain == 10);
    assert(agc.max_gain == 120);

    // agc_base_gain above new max is clamped down
    agc.agc_base_gain = 200;
    agc.configure(10, 120, 4, 1, 4);
    assert(agc.agc_base_gain == 120);

    // agc_base_gain below new min is clamped up
    agc.agc_base_gain = 5;
    agc.configure(10, 120, 4, 1, 4);
    assert(agc.agc_base_gain == 10);

    // agc_base_gain within range is left unchanged
    agc.agc_base_gain = 60;
    agc.configure(10, 120, 4, 1, 4);
    assert(agc.agc_base_gain == 60);

    // holdoff_frames == 0 is invalid: recovery condition would be permanently
    // true, causing gain-up on every non-saturated frame
    assert(agc.configure(10, 120, 4, 1, 0) == false);
    assert(agc.holdoff_frames == 4);  // unchanged
}

// ---------------------------------------------------------------------------
// Test 16: configure() rejects zero step_down (would silence AGC attack)
// ---------------------------------------------------------------------------
static void test_configure_rejects_zero_step_down()
{
    ADAR1000_AGC agc;
    assert(agc.configure(10, 120, 4, 1, 4) == true);  // baseline

    assert(agc.configure(10, 120, 0, 1, 4) == false);
    assert(agc.gain_step_down == 4);  // unchanged
    assert(agc.gain_step_up  == 1);   // unchanged
}

// ---------------------------------------------------------------------------
// Test 17: configure() rejects zero step_up (would silence AGC recovery)
// ---------------------------------------------------------------------------
static void test_configure_rejects_zero_step_up()
{
    ADAR1000_AGC agc;
    assert(agc.configure(10, 120, 4, 1, 4) == true);  // baseline

    assert(agc.configure(10, 120, 4, 0, 4) == false);
    assert(agc.gain_step_down == 4);  // unchanged
    assert(agc.gain_step_up  == 1);   // unchanged
}

// ---------------------------------------------------------------------------
// Test 18: configure() rejects max_gain > 127 (exceeds 7-bit VGA register)
// ---------------------------------------------------------------------------
static void test_configure_rejects_max_gain_out_of_range()
{
    ADAR1000_AGC agc;
    assert(agc.configure(10, 120, 4, 1, 4) == true);  // baseline

    assert(agc.configure(10, 200, 4, 1, 4) == false);
    assert(agc.max_gain == 120);  // unchanged
    assert(agc.min_gain == 10);   // unchanged

    // Boundary: exactly 127 is valid
    assert(agc.configure(10, 127, 4, 1, 4) == true);
    assert(agc.max_gain == 127);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    printf("=== ADAR1000_AGC Outer-Loop Unit Tests ===\n");

    RUN_TEST(test_defaults);
    RUN_TEST(test_saturation_reduces_gain);
    RUN_TEST(test_holdoff_prevents_early_gain_up);
    RUN_TEST(test_recovery_after_holdoff);
    RUN_TEST(test_min_gain_clamp);
    RUN_TEST(test_max_gain_clamp);
    RUN_TEST(test_calibration_offsets);
    RUN_TEST(test_disabled_noop);
    RUN_TEST(test_apply_gain_spi);
    RUN_TEST(test_reset_preserves_config);
    RUN_TEST(test_saturation_counter);
    RUN_TEST(test_mixed_sequence);
    RUN_TEST(test_effective_gain_edge_cases);
    RUN_TEST(test_saturation_never_increases_gain);
    RUN_TEST(test_configure_validation);
    RUN_TEST(test_configure_rejects_zero_step_down);
    RUN_TEST(test_configure_rejects_zero_step_up);
    RUN_TEST(test_configure_rejects_max_gain_out_of_range);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
