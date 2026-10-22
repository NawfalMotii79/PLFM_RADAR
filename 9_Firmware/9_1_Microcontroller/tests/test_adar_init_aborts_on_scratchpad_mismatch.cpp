// test_adar_init_aborts_on_scratchpad_mismatch.cpp
//
// previously initializeSingleDevice() logged a
// warning when the scratchpad readback didn't match 0xA5 but still set
// devices_[i]->initialized = true and returned true. That meant
// initializeAllDevices() would happily report success with four dead chips.
//
// New: any scratchpad mismatch aborts init. The device stays
// uninitialized, the function returns false, and downstream calls (e.g.
// readTemperature) return the "not initialized" sentinel -273.15.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "stm32_hal_mock.h"
#include "ADAR1000_Manager.h"

uint8_t GUI_start_flag_received = 0;
uint8_t USB_Buffer[64] = {0};
extern "C" void Error_Handler(void) {}

static int tests_passed = 0;
static int tests_total  = 0;

#define RUN_TEST(fn)                                                           \
    do {                                                                        \
        tests_total++;                                                          \
        printf("  [%2d] %-65s ", tests_total, #fn);                            \
        fn();                                                                   \
        tests_passed++;                                                         \
        printf("PASS\n");                                                       \
    } while (0)

// With default mock state, HAL_SPI_TransmitReceive writes 0x00 into rx_buffer[2].
// The scratchpad write programs 0xA5; the readback gets 0x00; mismatch -> abort.
static void test_scratchpad_mismatch_aborts_init()
{
    spy_reset();
    ADAR1000Manager mgr;

    bool ok = mgr.initializeAllDevices();

    assert(ok == false);  // would have been true under the old bug

    // Downstream proof: readTemperature must report "not initialized" sentinel,
    // not a temperature value, because the device is not marked initialized.
    float t = mgr.readTemperature(0);
    assert(t == -273.15f);
}

// When scratchpad readback matches, init succeeds. mock_spi_set_rx_byte(0xA5)
// makes every read return 0xA5, satisfying the verify step.
static void test_scratchpad_match_lets_init_succeed()
{
    spy_reset();
    mock_spi_set_rx_byte(0xA5);
    ADAR1000Manager mgr;

    bool ok = mgr.initializeAllDevices();

    assert(ok == true);

    // Now temperature read should produce a real number (not -273.15 sentinel).
    // adarAdcRead loops on bit 0 of REG_ADC_CONTROL; with rx_byte=0xA5 (bit 0 = 1)
    // the loop exits immediately, then REG_ADC_OUT also reads 0xA5.
    float t = mgr.readTemperature(0);
    assert(!std::isnan(t));
    assert(t != -273.15f);
}

// Init aborts on the first device that fails. Stats reflect partial progress
// (some writes_ok before the abort) which is the trend signal callers will
// query via getCommStats().
static void test_init_failure_recorded_in_stats()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    bool ok = mgr.initializeAllDevices();
    assert(ok == false);

    const auto& stats = mgr.getCommStats();
    // Several writes happened before scratchpad verify failed: soft reset,
    // configA, RAM bypass, ADC enable, scratchpad-write itself, and the
    // SDO-active toggles inside the scratchpad-read.
    assert(stats.writes_ok > 0);
    // The scratchpad readback succeeded at the SPI layer (HAL_OK) but produced
    // a wrong value -- that's not a read failure, it's a verify mismatch. So
    // reads_fail can stay 0 in the pure-mismatch case.
}

int main()
{
    printf("=== ADAR1000 init scratchpad-mismatch propagation tests ===\n");

    RUN_TEST(test_scratchpad_mismatch_aborts_init);
    RUN_TEST(test_scratchpad_match_lets_init_succeed);
    RUN_TEST(test_init_failure_recorded_in_stats);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
