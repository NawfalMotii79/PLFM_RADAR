// test_adar_adc_timeout_returns_nan.cpp
//
// readTemperature() previously returned -50.0 C silently when the on-chip
// ADC never completed a conversion (raw=0 timeout sentinel mapped through
// (raw * 0.5) - 50). A hung chip looked like a cold radar.
//
// New: on ADC timeout or any comm failure inside adarAdcRead(),
// readTemperature returns NaN, and comm_stats_.adc_timeouts increments.
// checkSystemHealth() in main.cpp uses isnan() to route NaN to the comm-error
// bucket (which triggers attemptErrorRecovery).

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

// Helper: bring all 4 devices through init successfully so readTemperature()
// gets past its "not initialized" guard.
static void init_devices_clean(ADAR1000Manager& mgr)
{
    mock_spi_set_rx_byte(0xA5);
    bool ok = mgr.initializeAllDevices();
    assert(ok);
}

// Default mock returns 0x00 to every read after init. Since bit 0 of the ADC
// status register never goes high, the polling loop in adarAdcRead is supposed
// to time out after 100 ms. Auto-advancing the tick on every HAL_GetTick()
// drives that timer forward without wall-clock waits.
static void test_polling_timeout_returns_nan()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);

    // Switch the rx-byte back to 0x00 so the ADC status bit stays low forever
    // and the polling loop runs to its 100 ms watchdog.
    mock_spi_set_rx_byte(0x00);
    mock_set_tick_auto_advance(150);  // each HAL_GetTick() advances 150 ms

    mgr.resetCommStats();
    float t = mgr.readTemperature(0);

    assert(std::isnan(t));
    const auto& stats = mgr.getCommStats();
    assert(stats.adc_timeouts >= 1);
}

// SPI failure during adarAdcRead's start-conversion write must also count as
// a timeout (caller has no valid ADC reading) and produce NaN.
static void test_start_conv_spi_failure_returns_nan()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);

    mgr.resetCommStats();
    // Fail the very next SPI call -- the start-conversion write inside
    // adarAdcRead. Following polling reads will also fail, but the function
    // bails on the start-conv write first.
    mock_spi_queue_failure(100, HAL_ERROR);

    float t = mgr.readTemperature(0);
    assert(std::isnan(t));

    const auto& stats = mgr.getCommStats();
    assert(stats.adc_timeouts >= 1);
}

// Healthy path: ADC bit 0 is high on the first poll (rx_byte = 0xA5 after init),
// so adarAdcRead exits the loop immediately, and readTemperature returns a
// finite number. This guards against false-positive NaN from the new code path.
static void test_healthy_adc_returns_finite_temp()
{
    spy_reset();
    ADAR1000Manager mgr;
    init_devices_clean(mgr);

    mgr.resetCommStats();
    float t = mgr.readTemperature(0);

    assert(!std::isnan(t));
    const auto& stats = mgr.getCommStats();
    assert(stats.adc_timeouts == 0);
}

int main()
{
    printf("=== ADAR1000 ADC timeout -> NaN propagation tests ===\n");

    RUN_TEST(test_polling_timeout_returns_nan);
    RUN_TEST(test_start_conv_spi_failure_returns_nan);
    RUN_TEST(test_healthy_adc_returns_finite_temp);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
