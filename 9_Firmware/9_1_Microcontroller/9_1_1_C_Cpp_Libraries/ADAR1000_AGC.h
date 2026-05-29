// ADAR1000_AGC.h -- STM32 outer-loop AGC for ADAR1000 RX VGA gain
//
// Adjusts the analog VGA common-mode gain on each ADAR1000 RX channel based on
// the FPGA's saturation flag (DIG_5 / PD13).  Runs once per radar frame
// (~258 ms) in the main loop, after runRadarPulseSequence().
//
// Architecture:
//   - Inner loop (FPGA, per-sample): rx_gain_control auto-adjusts digital
//     gain_shift based on peak magnitude / saturation.  Range ±42 dB.
//   - Outer loop (THIS MODULE, per-frame): reads FPGA DIG_5 GPIO.  If
//     saturation detected, reduces agc_base_gain immediately (attack).  If no
//     saturation for holdoff_frames, increases agc_base_gain (decay/recovery).
//
// Per-channel gain formula:
//   VGA[dev][ch] = clamp(agc_base_gain + cal_offset[dev*4+ch], min_gain, max_gain)
//
// The cal_offset array allows per-element calibration to correct inter-channel
// gain imbalance.  Default is all zeros (uniform gain).

#ifndef ADAR1000_AGC_H
#define ADAR1000_AGC_H

#include <stdint.h>

// Forward-declare to avoid pulling in the full ADAR1000_Manager header here.
// The .cpp includes the real header.
class ADAR1000Manager;

// Number of ADAR1000 devices
#define AGC_NUM_DEVICES   4
// Number of channels per ADAR1000
#define AGC_NUM_CHANNELS  4
// Total RX channels
#define AGC_TOTAL_CHANNELS (AGC_NUM_DEVICES * AGC_NUM_CHANNELS)

class ADAR1000_AGC {
public:
    // --- Configuration (public for easy field-testing / GUI override) ---

    // Common-mode base gain (raw ADAR1000 register value, 0-255).
    // Default matches ADAR1000Manager::kDefaultRxVgaGain = 30.
    uint8_t agc_base_gain;

    // Per-channel calibration offset (signed, added to agc_base_gain).
    // Index = device*4 + channel.  Default: all 0.
    int8_t cal_offset[AGC_TOTAL_CHANNELS];

    // How much to decrease agc_base_gain per frame when saturated (attack).
    uint8_t gain_step_down;

    // How much to increase agc_base_gain per frame when recovering (decay).
    uint8_t gain_step_up;

    // Minimum allowed agc_base_gain (floor).
    uint8_t min_gain;

    // Maximum allowed agc_base_gain (ceiling).
    uint8_t max_gain;

    // Number of consecutive non-saturated frames required before gain-up.
    uint8_t holdoff_frames;

    // Master enable.  When false, update() is a no-op.
    bool enabled;

    // --- Runtime state (read-only for diagnostics) ---

    // Consecutive non-saturated frame counter (resets on saturation).
    uint8_t holdoff_counter;

    // True if the last update() saw saturation.
    bool last_saturated;

    // Total saturation events since reset/construction.
    uint32_t saturation_event_count;

    // --- Methods ---

    ADAR1000_AGC();

    // Call once per frame after runRadarPulseSequence().
    // fpga_saturation: result of HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET
    void update(bool fpga_saturation);

    // Apply the current gain to all 16 RX VGA channels via the Manager.
    void applyGain(ADAR1000Manager &mgr);

    // Set configuration fields with validation.
    // Returns false (no-op) if p_min_gain > p_max_gain.
    // On success, all five fields are updated and agc_base_gain is clamped
    // to the new [min_gain, max_gain] range.
    bool configure(uint8_t p_min_gain, uint8_t p_max_gain,
                   uint8_t p_step_down, uint8_t p_step_up,
                   uint8_t p_holdoff_frames);

    // Reset runtime state (holdoff counter, saturation count) without
    // changing configuration.
    void resetState();

    // Compute the effective gain for a specific channel index (0-15),
    // clamped to [min_gain, max_gain].  Useful for diagnostics.
    uint8_t effectiveGain(uint8_t channel_index) const;
};

#endif // ADAR1000_AGC_H
