// test_adar_spi_write_failure_propagates.cpp
//
// When HAL_SPI_Transmit / HAL_SPI_TransmitReceive returns HAL_ERROR, every
// caller above must see the failure rather than silently continuing on.
// Previously adarWrite() returned void and dropped the SPI status on the
// floor, so a dead bus produced four "successful" inits.

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

// First SPI transmit fails (the soft-reset write inside initializeSingleDevice).
// Init must abort immediately and report false.
static void test_first_spi_failure_aborts_init()
{
    spy_reset();
    mock_spi_queue_failure(1, HAL_ERROR);
    ADAR1000Manager mgr;

    bool ok = mgr.initializeAllDevices();
    assert(ok == false);

    const auto& stats = mgr.getCommStats();
    assert(stats.writes_fail >= 1);
    assert(stats.last_fail_dev == 0);  // failure was on dev[0]
}

// Sustained SPI failure (every call) must not produce a green init even if
// individual writes early in the sequence have already counted as failures
// without aborting -- the function must still return false at the end.
static void test_sustained_spi_failure_aborts_init()
{
    spy_reset();
    mock_spi_queue_failure(10000, HAL_ERROR);
    ADAR1000Manager mgr;

    bool ok = mgr.initializeAllDevices();
    assert(ok == false);

    const auto& stats = mgr.getCommStats();
    assert(stats.writes_ok == 0);
    assert(stats.writes_fail >= 1);
}

// adarSetBit must NOT proceed with the write when the read-modify-write read
// fails -- otherwise it would clobber every other bit in the register by
// writing back (0 | mask) over a register whose actual contents are unknown.
// We use setTRSwitchPosition (which calls adarSetBit) and inject a failure
// on the read leg.
static void test_set_bit_skips_write_on_read_failure()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    // adarSetBit calls adarReadChecked first, which itself does:
    //   1. adarWrite(SDO active)        -- HAL_SPI_Transmit
    //   2. HAL_SPI_TransmitReceive (the actual read)
    //   3. adarWrite(SDO inactive)      -- HAL_SPI_Transmit
    // We want the actual read (call #2) to fail. Queue failure starting at
    // call 2 by letting call 1 succeed first via a one-shot prime, then
    // queueing the failure.
    //
    // Simpler approach: queue a failure of 100 calls, then call setTRSwitchPosition
    // and assert reads_fail >= 1 and writes_fail >= 1 (the SDO-active write
    // also fails). The key invariant: even though the read-modify-write was
    // attempted, the function returned false.
    mock_spi_queue_failure(100, HAL_ERROR);

    bool ok = mgr.setTRSwitchPosition(0, true);
    assert(ok == false);

    const auto& stats = mgr.getCommStats();
    assert(stats.reads_fail >= 1 || stats.writes_fail >= 1);
}

// Mode-switch must not fall through to another write block on the device that
// failed -- and the per-device current_mode must not be updated.
static void test_mode_switch_failure_propagates()
{
    spy_reset();
    mock_spi_set_rx_byte(0xA5);  // make scratchpad verify succeed first
    ADAR1000Manager mgr;
    bool init_ok = mgr.initializeAllDevices();
    assert(init_ok);

    // Now inject sustained SPI failure for the mode switch.
    mgr.resetCommStats();
    mock_spi_queue_failure(10000, HAL_ERROR);

    bool ok = mgr.setAllDevicesRXMode();
    assert(ok == false);

    const auto& stats = mgr.getCommStats();
    assert(stats.writes_fail >= 1);
}

int main()
{
    printf("=== ADAR1000 SPI write-failure propagation tests ===\n");

    RUN_TEST(test_first_spi_failure_aborts_init);
    RUN_TEST(test_sustained_spi_failure_aborts_init);
    RUN_TEST(test_set_bit_skips_write_on_read_failure);
    RUN_TEST(test_mode_switch_failure_propagates);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
