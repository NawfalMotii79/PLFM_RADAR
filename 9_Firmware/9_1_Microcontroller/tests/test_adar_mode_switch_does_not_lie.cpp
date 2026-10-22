// test_adar_mode_switch_does_not_lie.cpp
//
// setAllDevicesTXMode / setAllDevicesRXMode previously updated current_mode_
// (both global and per-device) before issuing the SPI writes that actually
// reconfigure the chip, then returned true unconditionally. A SPI failure
// during the mode switch left software believing it was in TX mode while the
// hardware was still in RX (or vice-versa) -- a real safety hazard given
// that PA biasing is mode-dependent.
//
// New: current_mode_ only updates if every underlying write
// succeeded; otherwise the function returns false and the mode flag is left
// at its last-known-good value.

#include <cassert>
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

static void init_devices_clean(ADAR1000Manager& mgr)
{
    mock_spi_set_rx_byte(0xA5);
    bool ok = mgr.initializeAllDevices();
    assert(ok);
}

// After a clean init, the manager is in TX mode (initializeAllDevices ends
// with setAllDevicesTXMode). Force RX mode under sustained SPI failure: must
// return false AND must not flip current_mode_ to RX.
static void test_failed_rx_switch_does_not_update_mode()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::TX);

    mgr.resetCommStats();
    mock_spi_queue_failure(10000, HAL_ERROR);

    bool ok = mgr.setAllDevicesRXMode();
    assert(ok == false);

    // Mode flag must NOT have moved -- this is the dangerous lie we are fixing.
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::TX);
}

// Symmetric case: cleanly transition to RX, then fail TX setup. Mode flag
// must stay RX.
static void test_failed_tx_switch_does_not_update_mode()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);

    mock_spi_set_rx_byte(0xA5);
    bool rx_ok = mgr.setAllDevicesRXMode();
    assert(rx_ok);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::RX);

    mgr.resetCommStats();
    mock_spi_queue_failure(10000, HAL_ERROR);

    bool ok = mgr.setAllDevicesTXMode();
    assert(ok == false);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::RX);
}

// Healthy round-trip: mode flag tracks the call that was made. Guards against
// the new code path accidentally refusing to advance the mode on success.
static void test_clean_mode_switches_update_flag()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::TX);

    mock_spi_set_rx_byte(0xA5);
    bool to_rx = mgr.setAllDevicesRXMode();
    assert(to_rx);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::RX);

    bool to_tx = mgr.setAllDevicesTXMode();
    assert(to_tx);
    assert(mgr.getCurrentMode() == ADAR1000Manager::BeamDirection::TX);
}

int main()
{
    printf("=== ADAR1000 mode-switch honesty tests ===\n");

    RUN_TEST(test_failed_rx_switch_does_not_update_mode);
    RUN_TEST(test_failed_tx_switch_does_not_update_mode);
    RUN_TEST(test_clean_mode_switches_update_flag);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
