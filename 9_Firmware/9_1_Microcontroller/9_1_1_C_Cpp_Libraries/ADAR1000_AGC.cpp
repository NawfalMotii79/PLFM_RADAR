// ADAR1000_AGC.cpp -- STM32 outer-loop AGC implementation
//
// See ADAR1000_AGC.h for architecture overview.

#include "ADAR1000_AGC.h"
#include "ADAR1000_Manager.h"
#include "diag_log.h"

#include <cstring>

// ---------------------------------------------------------------------------
// Constructor -- set all config fields to safe defaults
// ---------------------------------------------------------------------------
ADAR1000_AGC::ADAR1000_AGC()
    : agc_base_gain(ADAR1000Manager::kDefaultRxVgaGain) // 30
    , gain_step_down(4)
    , gain_step_up(1)
    , min_gain(0)
    , max_gain(127)
    , holdoff_frames(4)
    , enabled(false)
    , holdoff_counter(0)
    , last_saturated(false)
    , saturation_event_count(0)
{
    memset(cal_offset, 0, sizeof(cal_offset));
    if (holdoff_frames == 0) holdoff_frames = 1;
}

// ---------------------------------------------------------------------------
// update -- called once per frame with the FPGA DIG_5 saturation flag
//
// Returns true if agc_base_gain changed (caller should then applyGain).
// ---------------------------------------------------------------------------
void ADAR1000_AGC::update(bool fpga_saturation)
{
    if (!enabled)
        return;

    last_saturated = fpga_saturation;

    if (fpga_saturation) {
        // Attack: reduce gain immediately
        saturation_event_count++;
        holdoff_counter = 0;

        if (agc_base_gain >= gain_step_down + min_gain) {
            agc_base_gain -= gain_step_down;
        } else {
            agc_base_gain = min_gain;
        }

        DIAG("AGC", "SAT detected -- gain_base -> %u  (events=%lu)",
             (unsigned)agc_base_gain, (unsigned long)saturation_event_count);

    } else {
        // Recovery: wait for holdoff, then increase gain
        holdoff_counter++;

        if (holdoff_counter >= holdoff_frames) {
            holdoff_counter = 0;

            if (agc_base_gain + gain_step_up <= max_gain) {
                agc_base_gain += gain_step_up;
            } else {
                agc_base_gain = max_gain;
            }

            DIAG("AGC", "Recovery step -- gain_base -> %u", (unsigned)agc_base_gain);
        }
    }
}

// ---------------------------------------------------------------------------
// applyGain -- write effective gain to all 16 RX VGA channels
//
// Uses the Manager's adarSetRxVgaGain which takes 1-based channel indices
// (matching the convention in setBeamAngle).
// ---------------------------------------------------------------------------
void ADAR1000_AGC::applyGain(ADAR1000Manager &mgr)
{
    for (uint8_t dev = 0; dev < AGC_NUM_DEVICES; ++dev) {
        for (uint8_t ch = 0; ch < AGC_NUM_CHANNELS; ++ch) {
            uint8_t gain = effectiveGain(dev * AGC_NUM_CHANNELS + ch);
            // Channel parameter is 1-based per Manager convention
            mgr.adarSetRxVgaGain(dev, ch + 1, gain, BROADCAST_OFF);
        }
    }
}

// ---------------------------------------------------------------------------
// resetState -- clear runtime counters, preserve configuration
// ---------------------------------------------------------------------------
void ADAR1000_AGC::resetState()
{
    holdoff_counter = 0;
    last_saturated = false;
    saturation_event_count = 0;
}

// ---------------------------------------------------------------------------
// effectiveGain -- compute clamped per-channel gain
// ---------------------------------------------------------------------------
uint8_t ADAR1000_AGC::effectiveGain(uint8_t channel_index) const
{
    if (channel_index >= AGC_TOTAL_CHANNELS)
        return min_gain;  // safety fallback — OOB channels get minimum gain

    int16_t raw = static_cast<int16_t>(agc_base_gain) + cal_offset[channel_index];

    if (raw < static_cast<int16_t>(min_gain))
        return min_gain;
    if (raw > static_cast<int16_t>(max_gain))
        return max_gain;

    return static_cast<uint8_t>(raw);
}
