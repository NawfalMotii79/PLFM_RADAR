/*******************************************************************************
 * test_gap3_fault_ack_clears_emergency.c
 *
 * Verifies the FAULT_ACK clear path for system_emergency_state:
 *   - USBHandler detects exactly [0x40, 0x00, 0x00, 0x00] in a 4-byte packet
 *   - Detection is false-positive-free: larger packets (settings data) carrying
 *     the same bytes as a subsequence must NOT trigger the ack
 *   - Main-loop blink logic clears system_emergency_state on receipt
 *
 * Logic extracted from USBHandler.cpp + main.cpp to mirror the actual code
 * paths without requiring HAL headers.
 ******************************************************************************/
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

/* ── Simulated USBHandler state ─────────────────────────────────────────── */
static bool fault_ack_received = false;
static volatile bool system_emergency_state = false;

static const uint8_t FAULT_ACK_SEQ[4] = {0x40, 0x00, 0x00, 0x00};

/* Mirrors USBHandler::processUSBData() detection logic */
static void sim_processUSBData(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0) return;
    if (length == 4 && memcmp(data, FAULT_ACK_SEQ, 4) == 0) {
        fault_ack_received = true;
        return;
    }
    /* (normal state machine omitted — not under test here) */
}

/* Mirrors one iteration of the blink loop in main.cpp */
static void sim_blink_iteration(void)
{
    /* HAL_GPIO_TogglePin + HAL_Delay omitted */
    if (fault_ack_received) {
        system_emergency_state = false;
        fault_ack_received = false;
    }
}

int main(void)
{
    printf("=== Gap-3 FAULT_ACK clears system_emergency_state ===\n");

    /* Test 1: exact 4-byte FAULT_ACK packet sets the flag */
    printf("  Test 1: exact FAULT_ACK packet detected... ");
    fault_ack_received = false;
    const uint8_t ack_pkt[4] = {0x40, 0x00, 0x00, 0x00};
    sim_processUSBData(ack_pkt, 4);
    assert(fault_ack_received == true);
    printf("PASS\n");

    /* Test 2: flag cleared and system_emergency_state exits blink loop */
    printf("  Test 2: blink loop exits on FAULT_ACK... ");
    system_emergency_state = true;
    fault_ack_received = true;
    sim_blink_iteration();
    assert(system_emergency_state == false);
    assert(fault_ack_received == false);
    printf("PASS\n");

    /* Test 3: blink loop does NOT exit without ack */
    printf("  Test 3: blink loop holds without ack... ");
    system_emergency_state = true;
    fault_ack_received = false;
    sim_blink_iteration();
    assert(system_emergency_state == true);
    printf("PASS\n");

    /* Test 4: settings-sized packet carrying [0x40,0x00,0x00,0x00] as first
     * 4 bytes does NOT trigger ack (IEEE 754 double 2.0 = 0x4000000000000000) */
    printf("  Test 4: settings packet with 2.0 double does not false-trigger... ");
    fault_ack_received = false;
    uint8_t settings_pkt[82];
    memset(settings_pkt, 0, sizeof(settings_pkt));
    /* First 4 bytes look like FAULT_ACK but packet length is 82 */
    settings_pkt[0] = 0x40; settings_pkt[1] = 0x00;
    settings_pkt[2] = 0x00; settings_pkt[3] = 0x00;
    sim_processUSBData(settings_pkt, sizeof(settings_pkt));
    assert(fault_ack_received == false);
    printf("PASS\n");

    /* Test 5: 3-byte packet (truncated) does not trigger */
    printf("  Test 5: truncated 3-byte packet ignored... ");
    fault_ack_received = false;
    const uint8_t short_pkt[3] = {0x40, 0x00, 0x00};
    sim_processUSBData(short_pkt, 3);
    assert(fault_ack_received == false);
    printf("PASS\n");

    /* Test 6: wrong opcode byte in 4-byte packet does not trigger */
    printf("  Test 6: wrong opcode (0x28 AGC_ENABLE) not detected as FAULT_ACK... ");
    fault_ack_received = false;
    const uint8_t agc_pkt[4] = {0x28, 0x00, 0x00, 0x01};
    sim_processUSBData(agc_pkt, 4);
    assert(fault_ack_received == false);
    printf("PASS\n");

    /* Test 7: multiple blink iterations — loop stays active until ack */
    printf("  Test 7: loop stays active across multiple iterations until ack... ");
    system_emergency_state = true;
    fault_ack_received = false;
    sim_blink_iteration();
    assert(system_emergency_state == true);
    sim_blink_iteration();
    assert(system_emergency_state == true);
    /* Now ack arrives */
    sim_processUSBData(ack_pkt, 4);
    assert(fault_ack_received == true);
    sim_blink_iteration();
    assert(system_emergency_state == false);
    printf("PASS\n");

    printf("\n=== Gap-3 FAULT_ACK: ALL 7 TESTS PASSED ===\n\n");
    return 0;
}
