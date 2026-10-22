// test_adar_comm_stats_increment.cpp
//
// Documents and locks the CommStats observability surface added in this PR.
// Without this test, a future "simplification" could quietly remove the
// counters and nothing else would catch it.
//
// Contract:
//   - Every successful adarWrite increments writes_ok.
//   - Every failed   adarWrite increments writes_fail and updates last_fail_dev.
//   - Every successful adarRead  increments reads_ok.
//   - Every failed   adarRead  increments reads_fail  and updates last_fail_dev.
//   - resetCommStats() zeroes all counters and resets last_fail_dev to 0xFF.
//   - PR2 may promote a richer OpStatus enum return type; this struct is the
//     forward-compatible observability hook either way.

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

static void test_default_stats_are_zero()
{
    spy_reset();
    ADAR1000Manager mgr;

    const auto& s = mgr.getCommStats();
    assert(s.writes_ok    == 0);
    assert(s.writes_fail  == 0);
    assert(s.reads_ok     == 0);
    assert(s.reads_fail   == 0);
    assert(s.adc_timeouts == 0);
    assert(s.last_fail_dev == 0xFF);
}

static void test_successful_write_increments_writes_ok()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    bool ok = mgr.writeRegister(0, 0x010, 0x42);
    assert(ok);

    const auto& s = mgr.getCommStats();
    assert(s.writes_ok   == 1);
    assert(s.writes_fail == 0);
    assert(s.last_fail_dev == 0xFF);
}

static void test_failed_write_increments_writes_fail_and_records_dev()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    mock_spi_queue_failure(1, HAL_ERROR);
    bool ok = mgr.writeRegister(2, 0x010, 0x42);  // dev[2]
    assert(ok == false);

    const auto& s = mgr.getCommStats();
    assert(s.writes_ok    == 0);
    assert(s.writes_fail  == 1);
    assert(s.last_fail_dev == 2);
}

static void test_successful_read_increments_reads_ok()
{
    spy_reset();
    mock_spi_set_rx_byte(0xA5);
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    uint8_t value = 0;
    bool ok = mgr.adarReadChecked(1, 0x010, &value);
    assert(ok);
    assert(value == 0xA5);

    const auto& s = mgr.getCommStats();
    assert(s.reads_ok   == 1);
    assert(s.reads_fail == 0);
}

static void test_failed_read_increments_reads_fail_and_records_dev()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    // adarReadChecked sequence:
    //   1. adarWrite(SDO active)        -- HAL_SPI_Transmit
    //   2. HAL_SPI_TransmitReceive (the actual read)   <-- we want this to fail
    //   3. adarWrite(SDO inactive)
    // Letting calls 1+2 fail (queue 2 failures) covers the case where SDO-active
    // also fails -- adarReadChecked aborts after #1 and bumps reads_fail.
    mock_spi_queue_failure(2, HAL_ERROR);

    uint8_t value = 0xCC;  // pre-poison to confirm out param gets cleared
    bool ok = mgr.adarReadChecked(3, 0x010, &value);
    assert(ok == false);
    assert(value == 0);  // adarReadChecked must zero the out param on failure

    const auto& s = mgr.getCommStats();
    assert(s.reads_fail >= 1);
    assert(s.last_fail_dev == 3);
}

// Reset must clear everything to defaults, including last_fail_dev back to 0xFF.
static void test_reset_clears_all_counters()
{
    spy_reset();
    ADAR1000Manager mgr;

    // Generate some non-zero state in every field.
    mgr.writeRegister(0, 0x010, 0x42);            // writes_ok++
    mock_spi_queue_failure(1, HAL_ERROR);
    mgr.writeRegister(1, 0x010, 0x42);            // writes_fail++, last_fail_dev=1
    mock_spi_set_rx_byte(0xA5);
    uint8_t v = 0;
    mgr.adarReadChecked(0, 0x010, &v);            // reads_ok++

    {
        const auto& s = mgr.getCommStats();
        assert(s.writes_ok     >  0);
        assert(s.writes_fail   >  0);
        assert(s.reads_ok      >  0);
        assert(s.last_fail_dev != 0xFF);
    }

    mgr.resetCommStats();

    const auto& s = mgr.getCommStats();
    assert(s.writes_ok     == 0);
    assert(s.writes_fail   == 0);
    assert(s.reads_ok      == 0);
    assert(s.reads_fail    == 0);
    assert(s.adc_timeouts  == 0);
    assert(s.last_fail_dev == 0xFF);
}

// Out-of-range device index counts as a write/read failure (not a silent skip).
// This is the kind of bug that would otherwise hide behind a "device 4 ignored"
// log line and never escalate to the caller.
static void test_out_of_range_device_index_counts_as_failure()
{
    spy_reset();
    ADAR1000Manager mgr;
    mgr.resetCommStats();

    bool wok = mgr.writeRegister(99, 0x010, 0x42);
    assert(wok == false);
    assert(mgr.getCommStats().writes_fail == 1);
    assert(mgr.getCommStats().last_fail_dev == 99);

    uint8_t v = 0;
    bool rok = mgr.adarReadChecked(99, 0x010, &v);
    assert(rok == false);
    assert(mgr.getCommStats().reads_fail == 1);
}

int main()
{
    printf("=== ADAR1000 CommStats observability tests ===\n");

    RUN_TEST(test_default_stats_are_zero);
    RUN_TEST(test_successful_write_increments_writes_ok);
    RUN_TEST(test_failed_write_increments_writes_fail_and_records_dev);
    RUN_TEST(test_successful_read_increments_reads_ok);
    RUN_TEST(test_failed_read_increments_reads_fail_and_records_dev);
    RUN_TEST(test_reset_clears_all_counters);
    RUN_TEST(test_out_of_range_device_index_counts_as_failure);

    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_total);
    return (tests_passed == tests_total) ? 0 : 1;
}
