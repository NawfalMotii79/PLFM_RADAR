// ADAR1000_Manager.cpp
#include "main.h"
#include "stm32f7xx_hal.h"
#include "ADAR1000_Manager.h"
#include "diag_log.h"
#include <cmath>
#include <cstring>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

// Chip Select GPIO definitions
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} CHIP_SELECTS[4] = {
    {ADAR_1_CS_3V3_GPIO_Port, ADAR_1_CS_3V3_Pin}, // ADAR1000 #1
    {ADAR_2_CS_3V3_GPIO_Port, ADAR_2_CS_3V3_Pin}, // ADAR1000 #2
    {ADAR_3_CS_3V3_GPIO_Port, ADAR_3_CS_3V3_Pin}, // ADAR1000 #3
    {ADAR_4_CS_3V3_GPIO_Port, ADAR_4_CS_3V3_Pin}  // ADAR1000 #4
};

// ADAR1000 Vector Modulator lookup tables (128-state phase grid, 2.8125 deg step).
//
// Source: Analog Devices ADAR1000 datasheet Rev. B, Tables 13-16, page 34
//   (7_Components Datasheets and Application notes/ADAR1000.pdf)
// Cross-checked against the ADI Linux mainline driver (GPL-2.0, NOT vendored):
//   https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/
//     drivers/iio/beamformer/adar1000.c  (adar1000_phase_values[])
// The 128 byte values themselves are factual data from the datasheet and are
// not subject to copyright; only the ADI driver code is GPL.
//
// Byte format (per datasheet):
//   bit  [7:6] reserved (0)
//   bit  [5]   polarity:  1 = positive lobe (sign(I) or sign(Q) >= 0)
//                          0 = negative lobe
//   bits [4:0] 5-bit unsigned magnitude (0..31)
// At magnitude=0 the polarity bit is physically meaningless; the datasheet
// uses POL=1 (e.g. VM_Q at 0 deg = 0x20, VM_I at 90 deg = 0x21).
//
// Index mapping is uniform: VM_I[k] / VM_Q[k] correspond to phase angle
// k * 360/128 = k * 2.8125 degrees.  Callers index as VM_*[phase % 128].
const uint8_t ADAR1000Manager::VM_I[128] = {
    0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3E, 0x3E, 0x3D,  // [  0]   0.0000 deg
    0x3D, 0x3C, 0x3C, 0x3B, 0x3A, 0x39, 0x38, 0x37,  // [  8]  22.5000 deg
    0x36, 0x35, 0x34, 0x33, 0x32, 0x30, 0x2F, 0x2E,  // [ 16]  45.0000 deg
    0x2C, 0x2B, 0x2A, 0x28, 0x27, 0x25, 0x24, 0x22,  // [ 24]  67.5000 deg
    0x21, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A,  // [ 32]  90.0000 deg
    0x0B, 0x0D, 0x0E, 0x0F, 0x11, 0x12, 0x13, 0x14,  // [ 40] 112.5000 deg
    0x16, 0x17, 0x18, 0x19, 0x19, 0x1A, 0x1B, 0x1C,  // [ 48] 135.0000 deg
    0x1C, 0x1D, 0x1E, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F,  // [ 56] 157.5000 deg
    0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1E, 0x1D,  // [ 64] 180.0000 deg
    0x1D, 0x1C, 0x1C, 0x1B, 0x1A, 0x19, 0x18, 0x17,  // [ 72] 202.5000 deg
    0x16, 0x15, 0x14, 0x13, 0x12, 0x10, 0x0F, 0x0E,  // [ 80] 225.0000 deg
    0x0C, 0x0B, 0x0A, 0x08, 0x07, 0x05, 0x04, 0x02,  // [ 88] 247.5000 deg
    0x01, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A,  // [ 96] 270.0000 deg
    0x2B, 0x2D, 0x2E, 0x2F, 0x31, 0x32, 0x33, 0x34,  // [104] 292.5000 deg
    0x36, 0x37, 0x38, 0x39, 0x39, 0x3A, 0x3B, 0x3C,  // [112] 315.0000 deg
    0x3C, 0x3D, 0x3E, 0x3E, 0x3E, 0x3F, 0x3F, 0x3F,  // [120] 337.5000 deg
};

const uint8_t ADAR1000Manager::VM_Q[128] = {
    0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A,  // [  0]   0.0000 deg
    0x2B, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34,  // [  8]  22.5000 deg
    0x35, 0x36, 0x37, 0x38, 0x38, 0x39, 0x3A, 0x3A,  // [ 16]  45.0000 deg
    0x3B, 0x3C, 0x3C, 0x3C, 0x3D, 0x3D, 0x3D, 0x3D,  // [ 24]  67.5000 deg
    0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3C, 0x3C, 0x3C,  // [ 32]  90.0000 deg
    0x3B, 0x3A, 0x3A, 0x39, 0x38, 0x38, 0x37, 0x36,  // [ 40] 112.5000 deg
    0x35, 0x34, 0x33, 0x31, 0x30, 0x2F, 0x2E, 0x2D,  // [ 48] 135.0000 deg
    0x2B, 0x2A, 0x28, 0x27, 0x26, 0x24, 0x23, 0x21,  // [ 56] 157.5000 deg
    0x20, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A,  // [ 64] 180.0000 deg
    0x0B, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x13, 0x14,  // [ 72] 202.5000 deg
    0x15, 0x16, 0x17, 0x18, 0x18, 0x19, 0x1A, 0x1A,  // [ 80] 225.0000 deg
    0x1B, 0x1C, 0x1C, 0x1C, 0x1D, 0x1D, 0x1D, 0x1D,  // [ 88] 247.5000 deg
    0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1C, 0x1C, 0x1C,  // [ 96] 270.0000 deg
    0x1B, 0x1A, 0x1A, 0x19, 0x18, 0x18, 0x17, 0x16,  // [104] 292.5000 deg
    0x15, 0x14, 0x13, 0x11, 0x10, 0x0F, 0x0E, 0x0D,  // [112] 315.0000 deg
    0x0B, 0x0A, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01,  // [120] 337.5000 deg
};

// NOTE: a VM_GAIN[128] table previously existed here as a placeholder but was
// never populated and never read.  The ADAR1000 vector modulator has no
// separate gain register: phase-state magnitude is encoded directly in
// bits [4:0] of the VM_I/VM_Q bytes above.  Per-channel VGA gain is a
// distinct register (CHx_RX_GAIN at 0x10-0x13, CHx_TX_GAIN at 0x1C-0x1F)
// written with the user-supplied byte directly by adarSetRxVgaGain() /
// adarSetTxVgaGain().  Do not reintroduce a VM_GAIN[] array.

ADAR1000Manager::ADAR1000Manager() {
    for (int i = 0; i < 4; ++i) {
        devices_.push_back(std::make_unique<ADAR1000Device>(i));
    }
}

ADAR1000Manager::~ADAR1000Manager() {
    // Automatic cleanup by unique_ptr
}

// System Management
bool ADAR1000Manager::powerUpSystem() {
    DIAG_SECTION("BF POWER-UP SEQUENCE");
    uint32_t t0 = HAL_GetTick();
    const uint8_t msg[] = "Starting System Power-Up Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg) - 1, 1000);

    // Power-up sequence steps...
    DIAG("BF", "Enabling VDD_SW (3.3V)");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    DIAG("BF", "Enabling VSS_SW (3.3V)");
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    // Initialize devices
    DIAG("BF", "Calling initializeAllDevices()");
    if (!initializeAllDevices()) {
        DIAG_ERR("BF", "initializeAllDevices() FAILED");
        const uint8_t err[] = "ERROR: ADAR1000 initialization failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err) - 1, 1000);
        return false;
    }
    DIAG("BF", "initializeAllDevices() OK");

    // Start in RX mode
    DIAG("BF", "Setting initial mode to RX");
    if (!switchToRXMode()) {
        DIAG_ERR("BF", "Initial switchToRXMode() FAILED -- leaving in last-known mode");
        return false;
    }

    DIAG_ELAPSED("BF", "powerUpSystem() total", t0);
    const uint8_t success[] = "System Power-Up Sequence Completed Successfully.\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success) - 1, 1000);
    return true;
}

bool ADAR1000Manager::powerDownSystem() {
    DIAG_SECTION("BF POWER-DOWN SEQUENCE");
    DIAG("BF", "Switching to RX mode before power-down");
    // Note: even if RX-mode switch fails partially, we still cut the rails
    // below. Power-down must always proceed -- a stuck PA bias would be
    // worse than losing the RX-mode telemetry. We capture the status to
    // return at the end.
    bool rx_ok = switchToRXMode();
    HAL_Delay(10);

    DIAG("BF", "Disabling PA supplies");
    disablePASupplies();
    DIAG("BF", "Disabling LNA supplies");
    disableLNASupplies();
    DIAG("BF", "Disabling VSS_SW rail");
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_RESET);
    DIAG("BF", "Disabling VDD_SW rail");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);

    DIAG("BF", "powerDownSystem() %s", rx_ok ? "complete" : "complete (RX-mode setup had failures, rails cut anyway)");
    return rx_ok;
}

// Mode Switching
bool ADAR1000Manager::switchToTXMode() {
    DIAG_SECTION("BF SWITCH TO TX MODE");
    bool ok = true;
    DIAG("BF", "Step 1: LNA bias OFF");
    ok = setLNABias(false) && ok;
    delayUs(10);
    DIAG("BF", "Step 2: Enable PA supplies");
    enablePASupplies();
    delayUs(100);
    DIAG("BF", "Step 3: PA bias ON");
    ok = setPABias(true) && ok;
    delayUs(50);
    DIAG("BF", "Step 4: ADTR1107 -> TX");
    ok = setADTR1107Control(true) && ok;

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;
        dev_ok = adarSetTxBias(dev, BROADCAST_OFF) && dev_ok;
        if (dev_ok) {
            devices_[dev]->current_mode = BeamDirection::TX;
            DIAG("BF", "  dev[%u] TX enables=0x0F, TX bias set", dev);
        } else {
            DIAG_ERR("BF", "  dev[%u] TX setup FAILED -- per-device current_mode unchanged", dev);
            ok = false;
        }
    }
    if (ok) current_mode_ = BeamDirection::TX;
    DIAG("BF", "switchToTXMode() %s", ok ? "complete" : "completed WITH FAILURES (mode unchanged)");
    return ok;
}

bool ADAR1000Manager::switchToRXMode() {
    DIAG_SECTION("BF SWITCH TO RX MODE");
    bool ok = true;
    DIAG("BF", "Step 1: PA bias OFF");
    ok = setPABias(false) && ok;
    delayUs(50);
    DIAG("BF", "Step 2: Disable PA supplies");
    disablePASupplies();
    delayUs(10);
    DIAG("BF", "Step 3: ADTR1107 -> RX");
    ok = setADTR1107Control(false) && ok;
    DIAG("BF", "Step 4: Enable LNA supplies");
    enableLNASupplies();
    delayUs(50);
    DIAG("BF", "Step 5: LNA bias ON");
    ok = setLNABias(true) && ok;
    delayUs(50);

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;
        if (dev_ok) {
            devices_[dev]->current_mode = BeamDirection::RX;
            DIAG("BF", "  dev[%u] RX enables=0x0F", dev);
        } else {
            DIAG_ERR("BF", "  dev[%u] RX setup FAILED -- per-device current_mode unchanged", dev);
            ok = false;
        }
    }
    if (ok) current_mode_ = BeamDirection::RX;
    DIAG("BF", "switchToRXMode() %s", ok ? "complete" : "completed WITH FAILURES (mode unchanged)");
    return ok;
}

bool ADAR1000Manager::fastTXMode() {
    DIAG("BF", "fastTXMode(): ADTR1107 -> TX (no bias sequencing)");
    bool ok = setADTR1107Control(true);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;
        if (dev_ok) devices_[dev]->current_mode = BeamDirection::TX;
        ok = dev_ok && ok;
    }
    if (ok) current_mode_ = BeamDirection::TX;
    return ok;
}

bool ADAR1000Manager::fastRXMode() {
    DIAG("BF", "fastRXMode(): ADTR1107 -> RX (no bias sequencing)");
    bool ok = setADTR1107Control(false);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;
        if (dev_ok) devices_[dev]->current_mode = BeamDirection::RX;
        ok = dev_ok && ok;
    }
    if (ok) current_mode_ = BeamDirection::RX;
    return ok;
}

bool ADAR1000Manager::pulseTXMode() {
    DIAG("BF", "pulseTXMode(): TR switch only");
    bool ok = setADTR1107Control(true);
    last_switch_time_us_ = HAL_GetTick() * 1000;
    return ok;
}

bool ADAR1000Manager::pulseRXMode() {
    DIAG("BF", "pulseRXMode(): TR switch only");
    bool ok = setADTR1107Control(false);
    last_switch_time_us_ = HAL_GetTick() * 1000;
    return ok;
}

// Beam Steering
bool ADAR1000Manager::setBeamAngle(float angle_degrees, BeamDirection direction) {
    DIAG("BF", "setBeamAngle(%.1f deg, %s)", (double)angle_degrees,
         direction == BeamDirection::TX ? "TX" : "RX");
    uint8_t phase_settings[4];
    calculatePhaseSettings(angle_degrees, phase_settings);
    DIAG("BF", "  phase[0..3] = %u, %u, %u, %u",
         phase_settings[0], phase_settings[1], phase_settings[2], phase_settings[3]);

    bool ok = (direction == BeamDirection::TX) ? setAllDevicesTXMode() : setAllDevicesRXMode();

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            if (direction == BeamDirection::TX) {
                ok = adarSetTxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF) && ok;
                ok = adarSetTxVgaGain(dev, ch + 1, kDefaultTxVgaGain, BROADCAST_OFF) && ok;
            } else {
                ok = adarSetRxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF) && ok;
                ok = adarSetRxVgaGain(dev, ch + 1, kDefaultRxVgaGain, BROADCAST_OFF) && ok;
            }
        }
    }
    return ok;
}

bool ADAR1000Manager::setCustomBeamPattern(const uint8_t phase_settings[4], const uint8_t gain_settings[4], BeamDirection direction) {
    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            if (direction == BeamDirection::TX) {
                ok = adarSetTxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF) && ok;
                ok = adarSetTxVgaGain(dev, ch + 1, gain_settings[ch], BROADCAST_OFF) && ok;
            } else {
                ok = adarSetRxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF) && ok;
                ok = adarSetRxVgaGain(dev, ch + 1, gain_settings[ch], BROADCAST_OFF) && ok;
            }
        }
    }
    return ok;
}

// Beam Sweeping
void ADAR1000Manager::startBeamSweeping() {
    beam_sweeping_active_ = true;
    current_beam_index_ = 0;
    last_beam_update_time_ = HAL_GetTick();
}

void ADAR1000Manager::stopBeamSweeping() {
    beam_sweeping_active_ = false;
}

void ADAR1000Manager::updateBeamPosition() {
    if (!beam_sweeping_active_) return;

    uint32_t current_time = HAL_GetTick();
    const std::vector<BeamConfig>& sequence =
        (current_mode_ == BeamDirection::TX) ? tx_beam_sequence_ : rx_beam_sequence_;

    if (sequence.empty()) return;

    if (current_time - last_beam_update_time_ >= beam_dwell_time_ms_) {
        const BeamConfig& beam = sequence[current_beam_index_];
        setCustomBeamPattern(beam.phase_settings, beam.gain_settings, current_mode_);

        current_beam_index_ = (current_beam_index_ + 1) % sequence.size();
        last_beam_update_time_ = current_time;
    }
}

void ADAR1000Manager::setBeamSequence(const std::vector<BeamConfig>& sequence, BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        tx_beam_sequence_ = sequence;
    } else {
        rx_beam_sequence_ = sequence;
    }
}

void ADAR1000Manager::clearBeamSequence(BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        tx_beam_sequence_.clear();
    } else {
        rx_beam_sequence_.clear();
    }
}

// Monitoring and Diagnostics
float ADAR1000Manager::readTemperature(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size() || !devices_[deviceIndex]->initialized) {
        DIAG_WARN("BF", "readTemperature(dev[%u]) skipped: not initialized", deviceIndex);
        return -273.15f;
    }

    // Snapshot the timeout counter so we can detect a timeout that occurred
    // anywhere inside adarAdcRead (start-conv write, polling, output read).
    // This keeps the float signature while still giving callers an explicit
    // "this reading is invalid" channel via std::isnan().
    uint32_t timeouts_before = comm_stats_.adc_timeouts;
    uint8_t  temp_raw        = adarAdcRead(deviceIndex, BROADCAST_OFF);
    if (comm_stats_.adc_timeouts > timeouts_before) {
        DIAG_WARN("BF", "readTemperature(dev[%u]): ADC timeout/comm-fail -- returning NaN", deviceIndex);
        return std::nanf("");
    }

    float temp_c = (temp_raw * 0.5f) - 50.0f;
    DIAG("BF", "readTemperature(dev[%u]): raw=0x%02X => %.1f C", deviceIndex, temp_raw, (double)temp_c);
    return temp_c;
}

bool ADAR1000Manager::verifyDeviceCommunication(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): index out of range", deviceIndex);
        return false;
    }

    // Distinguish three failure modes that previously all looked the same:
    //   1. scratchpad write failed at the SPI layer
    //   2. scratchpad read failed at the SPI layer
    //   3. value round-tripped but didn't match (real chip mismatch)
    constexpr uint8_t test_value = 0xA5;
    if (!adarWrite(deviceIndex, REG_SCRATCHPAD, test_value, BROADCAST_OFF)) {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): scratchpad WRITE failed", deviceIndex);
        return false;
    }
    HAL_Delay(1);
    uint8_t readback = 0;
    if (!adarReadChecked(deviceIndex, REG_SCRATCHPAD, &readback)) {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): scratchpad READ failed", deviceIndex);
        return false;
    }
    bool pass = (readback == test_value);
    if (pass) {
        DIAG("BF", "verifyDeviceComm(dev[%u]): scratchpad 0xA5 -> 0x%02X OK", deviceIndex, readback);
    } else {
        DIAG_ERR("BF", "verifyDeviceComm(dev[%u]): scratchpad 0xA5 -> 0x%02X MISMATCH", deviceIndex, readback);
    }
    return pass;
}

uint8_t ADAR1000Manager::readRegister(uint8_t deviceIndex, uint32_t address) {
    return adarRead(deviceIndex, address);
}

bool ADAR1000Manager::writeRegister(uint8_t deviceIndex, uint32_t address, uint8_t value) {
    return adarWrite(deviceIndex, address, value, BROADCAST_OFF);
}

// Configuration
void ADAR1000Manager::setSwitchSettlingTime(uint32_t us) {
    switch_settling_time_us_ = us;
}

void ADAR1000Manager::setFastSwitchMode(bool enable) {
    DIAG("BF", "setFastSwitchMode(%s)", enable ? "ON" : "OFF");
    fast_switch_mode_ = enable;
    if (enable) {
        switch_settling_time_us_ = 10;
        DIAG("BF", "  settling time = 10 us, enabling PA+LNA supplies and bias simultaneously");
        enablePASupplies();
        enableLNASupplies();
        setPABias(true);
        setLNABias(true);
    } else {
        switch_settling_time_us_ = 50;
        DIAG("BF", "  settling time = 50 us");
    }
}

void ADAR1000Manager::setBeamDwellTime(uint32_t ms) {
    beam_dwell_time_ms_ = ms;
}

// Private helper methods (implementation continues...)
// ... include all the private method implementations from your original file
// ============================================================================
// PRIVATE HELPER METHODS - Add these to the end of ADAR1000_Manager.cpp
// ============================================================================

bool ADAR1000Manager::initializeAllDevices() {
    DIAG_SECTION("BF INIT ALL DEVICES");

    // Initialize each ADAR1000
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        DIAG("BF", "Initializing ADAR1000 dev[%u]...", i);
        if (!initializeSingleDevice(i)) {
            DIAG_ERR("BF", "initializeSingleDevice(%u) FAILED -- aborting init", i);
            return false;
        }
        DIAG("BF", "  dev[%u] init OK", i);
    }

    DIAG("BF", "All 4 ADAR1000 devices initialized, setting TX mode");
    if (!setAllDevicesTXMode()) {
        DIAG_ERR("BF", "initializeAllDevices: setAllDevicesTXMode() failed");
        return false;
    }
    return true;
}

bool ADAR1000Manager::initializeSingleDevice(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) return false;

    DIAG("BF", "  dev[%u] soft reset", deviceIndex);
    if (!adarSoftReset(deviceIndex)) {
        DIAG_ERR("BF", "  dev[%u] soft reset FAILED -- aborting init", deviceIndex);
        return false;
    }
    HAL_Delay(10);

    DIAG("BF", "  dev[%u] write ConfigA (SDO_ACTIVE)", deviceIndex);
    if (!adarWriteConfigA(deviceIndex, INTERFACE_CONFIG_A_SDO_ACTIVE, BROADCAST_OFF)) {
        DIAG_ERR("BF", "  dev[%u] ConfigA write FAILED -- aborting init", deviceIndex);
        return false;
    }
    DIAG("BF", "  dev[%u] set RAM bypass (bias+beam)", deviceIndex);
    if (!adarSetRamBypass(deviceIndex, BROADCAST_OFF)) {
        DIAG_ERR("BF", "  dev[%u] RAM bypass write FAILED -- aborting init", deviceIndex);
        return false;
    }

    // Initialize ADC
    DIAG("BF", "  dev[%u] enable ADC (2MHz clk)", deviceIndex);
    if (!adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_2MHZ_CLK | ADAR1000_ADC_EN, BROADCAST_OFF)) {
        DIAG_ERR("BF", "  dev[%u] ADC enable write FAILED -- aborting init", deviceIndex);
        return false;
    }

    // Verify communication with scratchpad test. Previous behavior was to log
    // a warning and mark the device initialized anyway -- that hid completely
    // dead chips behind a green init. Now this is a hard failure.
    DIAG("BF", "  dev[%u] verifying SPI communication...", deviceIndex);
    if (!verifyDeviceCommunication(deviceIndex)) {
        DIAG_ERR("BF", "  dev[%u] scratchpad verify FAILED -- NOT marking initialized", deviceIndex);
        devices_[deviceIndex]->initialized = false;
        return false;
    }

    devices_[deviceIndex]->initialized = true;
    return true;
}

bool ADAR1000Manager::initializeADTR1107Sequence() {
    DIAG_SECTION("ADTR1107 POWER SEQUENCE (9-step)");
    uint32_t t0 = HAL_GetTick();

	//Powering up ADTR1107 TX mode
    const uint8_t msg[] = "Starting ADTR1107 Power Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg) - 1, 1000);

    // Step 1: Connect all GND pins to ground (assumed in hardware)
    DIAG("BF", "Step 1: GND pins (hardware -- assumed connected)");

    // Step 2: Set VDD_SW to 3.3V
    DIAG("BF", "Step 2: VDD_SW -> 3.3V");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 3: Set VSS_SW to -3.3V
    DIAG("BF", "Step 3: VSS_SW -> -3.3V");
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 4: Set CTRL_SW to RX mode initially via GPIO
    DIAG("BF", "Step 4: CTRL_SW -> RX (initial safe mode)");
    if (!setADTR1107Control(false)) {
        DIAG_ERR("BF", "ADTR1107 step 4 FAILED -- aborting power sequence");
        return false;
    }
    HAL_Delay(1);

    // Step 5: Set VGG_LNA to 0
    DIAG("BF", "Step 5: VGG_LNA bias -> OFF (0x%02X)", kLnaBiasOff);
    uint8_t lna_bias_voltage = kLnaBiasOff;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        if (!adarWrite(dev, REG_LNA_BIAS_ON, lna_bias_voltage, BROADCAST_OFF) ||
            !adarWrite(dev, REG_LNA_BIAS_OFF, kLnaBiasOff, BROADCAST_OFF)) {
            DIAG_ERR("BF", "ADTR1107 step 5 dev[%u] LNA bias write FAILED", dev);
            return false;
        }
    }

    // Step 6: Set VDD_LNA to 0V for TX mode
    DIAG("BF", "Step 6: VDD_LNA -> 0V (disable ADTR LNA supply)");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    // Step 7: Set VGG_PA to safe negative voltage (PA off for TX mode)
    /*A 0x00 value in the
    on or off bias registers, correspond to a 0 V output. A 0xFF in the
    on or off bias registers correspond to a −4.8 V output.*/
    DIAG("BF", "Step 7: VGG_PA -> safe bias 0x%02X (~ -1.75V, PA off)", kPaBiasTxSafe);
    uint8_t safe_pa_bias = kPaBiasTxSafe; // Safe negative voltage (-1.75V) to keep PA off
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        if (!adarWrite(dev, REG_PA_CH1_BIAS_ON, safe_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH2_BIAS_ON, safe_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH3_BIAS_ON, safe_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH4_BIAS_ON, safe_pa_bias, BROADCAST_OFF)) {
            DIAG_ERR("BF", "ADTR1107 step 7 dev[%u] safe PA bias write FAILED -- aborting before enabling supplies",
                     dev);
            return false;
        }
    }
    HAL_Delay(10);

    // Step 8: Set VDD_PA to 0V (PA powered up for TX mode)
    DIAG("BF", "Step 8: Enable PA supplies (VDD_PA)");
    enablePASupplies();
    HAL_Delay(50);

    // Step 9: Adjust VGG_PA voltage between −1.75 V and −0.25 V to achieve the desired IDQ_PA=220mA
    //Set VGG_PA to safe negative voltage (PA off for TX mode)
    /*A 0x00 value in the
    on or off bias registers, correspond to a 0 V output. A 0xFF in the
    on or off bias registers correspond to a −4.8 V output.*/
    DIAG("BF", "Step 9: VGG_PA -> Idq cal bias 0x%02X (~ -0.24V, target 220mA)", kPaBiasIdqCalibration);
    uint8_t Idq_pa_bias = kPaBiasIdqCalibration; // Safe negative voltage (-0.2447V) to keep PA off
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        if (!adarWrite(dev, REG_PA_CH1_BIAS_ON, Idq_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH2_BIAS_ON, Idq_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH3_BIAS_ON, Idq_pa_bias, BROADCAST_OFF) ||
            !adarWrite(dev, REG_PA_CH4_BIAS_ON, Idq_pa_bias, BROADCAST_OFF)) {
            DIAG_ERR("BF", "ADTR1107 step 9 dev[%u] Idq cal bias write FAILED", dev);
            return false;
        }
    }
    HAL_Delay(10);

    DIAG_ELAPSED("BF", "ADTR1107 power sequence", t0);

    const uint8_t success[] = "ADTR1107 power sequence completed.\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success) - 1, 1000);

    return true;
}

bool ADAR1000Manager::setAllDevicesTXMode() {
    DIAG("BF", "setAllDevicesTXMode(): ADTR1107 -> TX, then configure ADAR1000s");
    // Set ADTR1107 to TX mode first. If this fails, do NOT advance state --
    // software was previously claiming TX mode while hardware stayed in RX.
    if (!setADTR1107Mode(BeamDirection::TX)) {
        DIAG_ERR("BF", "setAllDevicesTXMode: ADTR1107 TX setup FAILED -- not updating mode flags");
        return false;
    }

    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;
        dev_ok = adarSetTxBias(dev, BROADCAST_OFF) && dev_ok;

        if (dev_ok) {
            devices_[dev]->current_mode = BeamDirection::TX;
            DIAG("BF", "  dev[%u] TX mode set (enables=0x0F, bias applied)", dev);
        } else {
            DIAG_ERR("BF", "  dev[%u] TX mode setup FAILED -- per-device current_mode unchanged", dev);
            ok = false;
        }
    }
    if (ok) {
        current_mode_ = BeamDirection::TX;
    } else {
        DIAG_ERR("BF", "setAllDevicesTXMode: at least one device failed -- global current_mode unchanged");
    }
    return ok;
}

bool ADAR1000Manager::setAllDevicesRXMode() {
    DIAG("BF", "setAllDevicesRXMode(): ADTR1107 -> RX, then configure ADAR1000s");
    if (!setADTR1107Mode(BeamDirection::RX)) {
        DIAG_ERR("BF", "setAllDevicesRXMode: ADTR1107 RX setup FAILED -- not updating mode flags");
        return false;
    }

    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        bool dev_ok = true;
        dev_ok = adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF) && dev_ok;
        dev_ok = adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF) && dev_ok;

        if (dev_ok) {
            devices_[dev]->current_mode = BeamDirection::RX;
            DIAG("BF", "  dev[%u] RX mode set (enables=0x0F)", dev);
        } else {
            DIAG_ERR("BF", "  dev[%u] RX mode setup FAILED -- per-device current_mode unchanged", dev);
            ok = false;
        }
    }
    if (ok) {
        current_mode_ = BeamDirection::RX;
    } else {
        DIAG_ERR("BF", "setAllDevicesRXMode: at least one device failed -- global current_mode unchanged");
    }
    return ok;
}

bool ADAR1000Manager::setADTR1107Mode(BeamDirection direction) {
    bool ok = true;
    if (direction == BeamDirection::TX) {
        DIAG_SECTION("ADTR1107 -> TX MODE");
        ok = setADTR1107Control(true) && ok;

        DIAG("BF", "  Disable LNA supplies");
        disableLNASupplies();
        HAL_Delay(5);

        DIAG("BF", "  LNA bias -> OFF (0x%02X)", kLnaBiasOff);
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarWrite(dev, REG_LNA_BIAS_ON, kLnaBiasOff, BROADCAST_OFF) && ok;
        }
        HAL_Delay(5);

        DIAG("BF", "  Enable PA supplies");
        enablePASupplies();
        HAL_Delay(10);

        DIAG("BF", "  PA bias -> operational (0x%02X)", kPaBiasOperational);
        uint8_t operational_pa_bias = kPaBiasOperational;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarWrite(dev, REG_PA_CH1_BIAS_ON, operational_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH2_BIAS_ON, operational_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH3_BIAS_ON, operational_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH4_BIAS_ON, operational_pa_bias, BROADCAST_OFF) && ok;
        }
        HAL_Delay(5);

        DIAG("BF", "  TR switch -> TX (TR_SOURCE=1, BIAS_EN)");
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarSetBit(dev, REG_SW_CONTROL, 2, BROADCAST_OFF) && ok;
            ok = adarSetBit(dev, REG_MISC_ENABLES, 5, BROADCAST_OFF) && ok;
        }
        DIAG("BF", "  ADTR1107 TX mode %s", ok ? "complete" : "completed WITH FAILURES");

    } else {
        DIAG_SECTION("ADTR1107 -> RX MODE");
        ok = setADTR1107Control(false) && ok;

        DIAG("BF", "  Disable PA supplies");
        disablePASupplies();
        HAL_Delay(5);

        DIAG("BF", "  PA bias -> safe (0x%02X)", kPaBiasRxSafe);
        uint8_t safe_pa_bias = kPaBiasRxSafe;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarWrite(dev, REG_PA_CH1_BIAS_ON, safe_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH2_BIAS_ON, safe_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH3_BIAS_ON, safe_pa_bias, BROADCAST_OFF) && ok;
            ok = adarWrite(dev, REG_PA_CH4_BIAS_ON, safe_pa_bias, BROADCAST_OFF) && ok;
        }
        HAL_Delay(5);

        DIAG("BF", "  Enable LNA supplies");
        enableLNASupplies();
        HAL_Delay(10);

        DIAG("BF", "  LNA bias -> operational (0x%02X)", kLnaBiasOperational);
        uint8_t operational_lna_bias = kLnaBiasOperational;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarWrite(dev, REG_LNA_BIAS_ON, operational_lna_bias, BROADCAST_OFF) && ok;
        }
        HAL_Delay(5);

        DIAG("BF", "  TR switch -> RX (TR_SOURCE=0, LNA_BIAS_OUT_EN)");
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            ok = adarResetBit(dev, REG_SW_CONTROL, 2, BROADCAST_OFF) && ok;
            ok = adarSetBit(dev, REG_MISC_ENABLES, 4, BROADCAST_OFF) && ok;
        }
        DIAG("BF", "  ADTR1107 RX mode %s", ok ? "complete" : "completed WITH FAILURES");
    }
    return ok;
}

bool ADAR1000Manager::setADTR1107Control(bool tx_mode) {
    DIAG("BF", "setADTR1107Control(%s): setting TR switch on all %u devices, settling %lu us",
         tx_mode ? "TX" : "RX", (unsigned)devices_.size(), (unsigned long)switch_settling_time_us_);
    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        ok = setTRSwitchPosition(dev, tx_mode) && ok;
    }
    delayUs(switch_settling_time_us_);
    return ok;
}

bool ADAR1000Manager::setTRSwitchPosition(uint8_t deviceIndex, bool tx_mode) {
    if (tx_mode) {
        // TX mode: Set TR_SOURCE = 1
        return adarSetBit(deviceIndex, REG_SW_CONTROL, 2, BROADCAST_OFF);
    }
    // RX mode: Set TR_SOURCE = 0
    return adarResetBit(deviceIndex, REG_SW_CONTROL, 2, BROADCAST_OFF);
}

// Add the new public method
bool ADAR1000Manager::setCustomBeamPattern16(const uint8_t phase_pattern[16], BeamDirection direction) {
    bool ok = true;
    for (uint8_t dev = 0; dev < 4; ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            uint8_t phase = phase_pattern[dev * 4 + ch];
            if (direction == BeamDirection::TX) {
                ok = adarSetTxPhase(dev, ch + 1, phase, BROADCAST_OFF) && ok;
            } else {
                ok = adarSetRxPhase(dev, ch + 1, phase, BROADCAST_OFF) && ok;
            }
        }
    }
    return ok;
}

void ADAR1000Manager::enablePASupplies() {
    DIAG("BF", "enablePASupplies(): PA1+PA2+PA3 -> ON");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disablePASupplies() {
    DIAG("BF", "disablePASupplies(): PA1+PA2+PA3 -> OFF");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);
}

void ADAR1000Manager::enableLNASupplies() {
    DIAG("BF", "enableLNASupplies(): ADTR 3.3V -> ON");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disableLNASupplies() {
    DIAG("BF", "disableLNASupplies(): ADTR 3.3V -> OFF");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
}

bool ADAR1000Manager::setPABias(bool enable) {
    uint8_t pa_bias = enable ? kPaBiasOperational : kPaBiasRxSafe; // Operational vs safe bias
    DIAG("BF", "setPABias(%s): bias=0x%02X", enable ? "ON" : "OFF", pa_bias);

    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        ok = adarWrite(dev, REG_PA_CH1_BIAS_ON, pa_bias, BROADCAST_OFF) && ok;
        ok = adarWrite(dev, REG_PA_CH2_BIAS_ON, pa_bias, BROADCAST_OFF) && ok;
        ok = adarWrite(dev, REG_PA_CH3_BIAS_ON, pa_bias, BROADCAST_OFF) && ok;
        ok = adarWrite(dev, REG_PA_CH4_BIAS_ON, pa_bias, BROADCAST_OFF) && ok;
    }
    return ok;
}

bool ADAR1000Manager::setLNABias(bool enable) {
    uint8_t lna_bias = enable ? kLnaBiasOperational : kLnaBiasOff; // Operational vs off
    DIAG("BF", "setLNABias(%s): bias=0x%02X", enable ? "ON" : "OFF", lna_bias);

    bool ok = true;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        ok = adarWrite(dev, REG_LNA_BIAS_ON, lna_bias, BROADCAST_OFF) && ok;
    }
    return ok;
}

void ADAR1000Manager::delayUs(uint32_t microseconds) {
    // Simple implementation - for F7 @ 216MHz, each loop ~7 cycles ≈ 0.032us
    volatile uint32_t cycles = microseconds * 10; // Adjust this multiplier for your clock
    while (cycles--) {
        __NOP();
    }
}

void ADAR1000Manager::calculatePhaseSettings(float angle_degrees, uint8_t phase_settings[4]) {
    const float freq = 10.5e9;
    const float c = 3e8;
    const float wavelength = c / freq;
    const float element_spacing = wavelength / 2;

    float angle_rad = angle_degrees * M_PI / 180.0;
    float phase_shift = (2 * M_PI * element_spacing * sin(angle_rad)) / wavelength;

    for (int i = 0; i < 4; ++i) {
        float element_phase = i * phase_shift;
        while (element_phase < 0) element_phase += 2 * M_PI;
        while (element_phase >= 2 * M_PI) element_phase -= 2 * M_PI;
        phase_settings[i] = static_cast<uint8_t>((element_phase / (2 * M_PI)) * 128);
    }
}

bool ADAR1000Manager::performSystemCalibration() {
    DIAG_SECTION("BF SYSTEM CALIBRATION");
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        DIAG("BF", "Calibration: verifying dev[%u] communication...", i);
        if (!verifyDeviceCommunication(i)) {
            DIAG_ERR("BF", "Calibration FAILED at dev[%u]", i);
            return false;
        }
    }
    DIAG("BF", "performSystemCalibration() OK -- all devices verified");
    return true;
}

// ============================================================================
// LOW-LEVEL SPI COMMUNICATION METHODS
// ============================================================================

void ADAR1000Manager::resetCommStats() {
    comm_stats_ = {0, 0, 0, 0, 0, 0xFF};
}

bool ADAR1000Manager::spiTransfer(uint8_t* txData, uint8_t* rxData, uint32_t size) {
    HAL_StatusTypeDef status;

    if (rxData) {
        status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, size, 1000);
    } else {
        status = HAL_SPI_Transmit(&hspi1, txData, size, 1000);
    }

    if (status != HAL_OK) {
        DIAG_ERR("BF", "SPI1 transfer FAILED: HAL status=%d, size=%lu", (int)status, (unsigned long)size);
        return false;
    }
    return true;
}

void ADAR1000Manager::setChipSelect(uint8_t deviceIndex, bool state) {
    if (deviceIndex >= devices_.size()) return;
    HAL_GPIO_WritePin(CHIP_SELECTS[deviceIndex].port,
                      CHIP_SELECTS[deviceIndex].pin,
                      state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool ADAR1000Manager::adarWrite(uint8_t deviceIndex, uint32_t mem_addr, uint8_t data, uint8_t broadcast) {
    if (deviceIndex >= devices_.size()) {
        comm_stats_.writes_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
        DIAG_ERR("BF", "adarWrite(dev[%u]): index out of range", deviceIndex);
        return false;
    }

    uint8_t instruction[3];

    if (broadcast) {
        instruction[0] = 0x08;
    } else {
        instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    }

    instruction[0] |= (0x1F00 & mem_addr) >> 8;
    instruction[1] = (0xFF & mem_addr);
    instruction[2] = data;

    setChipSelect(deviceIndex, true);
    bool ok = spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);

    if (ok) {
        comm_stats_.writes_ok++;
    } else {
        comm_stats_.writes_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
    }
    return ok;
}

bool ADAR1000Manager::adarReadChecked(uint8_t deviceIndex, uint32_t mem_addr, uint8_t* out) {
    if (out == nullptr) return false;
    *out = 0;

    if (deviceIndex >= devices_.size()) {
        comm_stats_.reads_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
        DIAG_ERR("BF", "adarRead(dev[%u]): index out of range", deviceIndex);
        return false;
    }

    uint8_t instruction[3] = {0};
    uint8_t rx_buffer[3]   = {0};

    // Set SDO active. Failure here means we cannot trust the readback that follows.
    if (!adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, INTERFACE_CONFIG_A_SDO_ACTIVE, 0)) {
        comm_stats_.reads_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
        return false;
    }

    instruction[0] = 0x80 | ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[0] |= ((0xff00 & mem_addr) >> 8);
    instruction[1] = (0xff & mem_addr);
    instruction[2] = 0x00;

    setChipSelect(deviceIndex, true);
    bool ok = spiTransfer(instruction, rx_buffer, sizeof(instruction));
    setChipSelect(deviceIndex, false);

    // Best-effort: clear SDO active even if the read above failed. Don't let a
    // failure on the trailing write override the read failure status.
    bool sdo_off_ok = adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, 0, 0);
    (void)sdo_off_ok;  // already counted in writes_*; don't double-count as a read failure.

    if (!ok) {
        comm_stats_.reads_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
        return false;
    }

    *out = rx_buffer[2];
    comm_stats_.reads_ok++;
    return true;
}

uint8_t ADAR1000Manager::adarRead(uint8_t deviceIndex, uint32_t mem_addr) {
    uint8_t value = 0;
    (void)adarReadChecked(deviceIndex, mem_addr, &value);
    return value;
}

bool ADAR1000Manager::adarSetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    uint8_t temp = 0;
    // Critical: read-modify-write must NOT proceed on a failed read, otherwise we
    // would write back (0 | mask) and clobber every other bit in the register.
    if (!adarReadChecked(deviceIndex, mem_addr, &temp)) {
        DIAG_ERR("BF", "adarSetBit(dev[%u], 0x%03lX, bit %u): read failed -- skipping write to avoid corruption",
                 deviceIndex, (unsigned long)mem_addr, bit);
        return false;
    }
    uint8_t data = temp | (1 << bit);
    return adarWrite(deviceIndex, mem_addr, data, broadcast);
}

bool ADAR1000Manager::adarResetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    uint8_t temp = 0;
    if (!adarReadChecked(deviceIndex, mem_addr, &temp)) {
        DIAG_ERR("BF", "adarResetBit(dev[%u], 0x%03lX, bit %u): read failed -- skipping write to avoid corruption",
                 deviceIndex, (unsigned long)mem_addr, bit);
        return false;
    }
    uint8_t data = temp & ~(1 << bit);
    return adarWrite(deviceIndex, mem_addr, data, broadcast);
}

bool ADAR1000Manager::adarSoftReset(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) return false;
    DIAG("BF", "adarSoftReset(dev[%u]): addr=0x%02X", deviceIndex, devices_[deviceIndex]->dev_addr);
    uint8_t instruction[3];
    instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[1] = 0x00;
    instruction[2] = 0x81;

    setChipSelect(deviceIndex, true);
    bool ok = spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);

    if (ok) {
        comm_stats_.writes_ok++;
    } else {
        comm_stats_.writes_fail++;
        comm_stats_.last_fail_dev = deviceIndex;
    }
    return ok;
}

bool ADAR1000Manager::adarWriteConfigA(uint8_t deviceIndex, uint8_t flags, uint8_t broadcast) {
    return adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, flags, broadcast);
}

bool ADAR1000Manager::adarSetRamBypass(uint8_t deviceIndex, uint8_t broadcast) {
    uint8_t data = (MEM_CTRL_BIAS_RAM_BYPASS | MEM_CTRL_BEAM_RAM_BYPASS);
    return adarWrite(deviceIndex, REG_MEM_CTL, data, broadcast);
}

bool ADAR1000Manager::adarSetRxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4) per API contract documented in
    // ADAR1000_AGC.cpp and matching ADI datasheet terminology.
    // Reject out-of-range early so a stale 0-based caller does not
    // silently wrap to ((0-1) & 0x03) == 3 and write to CH4.
    // See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetRxPhase: channel %u out of range [1..4], ignored", channel);
        return false;
    }
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    // Subtract 1 to convert 1-based channel to 0-based register offset
    // before masking. See issue #90.
    uint32_t mem_addr_i = REG_CH1_RX_PHS_I + ((channel - 1) & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_RX_PHS_Q + ((channel - 1) & 0x03) * 2;

    bool ok = adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    ok = adarWrite(deviceIndex, mem_addr_q, q_val, broadcast) && ok;
    ok = adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast) && ok;
    return ok;
}

bool ADAR1000Manager::adarSetTxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetTxPhase: channel %u out of range [1..4], ignored", channel);
        return false;
    }
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    uint32_t mem_addr_i = REG_CH1_TX_PHS_I + ((channel - 1) & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_TX_PHS_Q + ((channel - 1) & 0x03) * 2;

    bool ok = adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    ok = adarWrite(deviceIndex, mem_addr_q, q_val, broadcast) && ok;
    ok = adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast) && ok;
    return ok;
}

bool ADAR1000Manager::adarSetRxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetRxVgaGain: channel %u out of range [1..4], ignored", channel);
        return false;
    }
    uint32_t mem_addr = REG_CH1_RX_GAIN + ((channel - 1) & 0x03);
    bool ok = adarWrite(deviceIndex, mem_addr, gain, broadcast);
    ok = adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast) && ok;
    return ok;
}

bool ADAR1000Manager::adarSetTxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    // channel is 1-based (CH1..CH4). See issue #90.
    if (channel < 1 || channel > 4) {
        DIAG("BF", "adarSetTxVgaGain: channel %u out of range [1..4], ignored", channel);
        return false;
    }
    uint32_t mem_addr = REG_CH1_TX_GAIN + ((channel - 1) & 0x03);
    bool ok = adarWrite(deviceIndex, mem_addr, gain, broadcast);
    ok = adarWrite(deviceIndex, REG_LOAD_WORKING, LD_WRK_REGS_LDTX_OVERRIDE, broadcast) && ok;
    return ok;
}

bool ADAR1000Manager::adarSetTxBias(uint8_t deviceIndex, uint8_t broadcast) {
    bool ok = adarWrite(deviceIndex, REG_BIAS_CURRENT_TX, kTxBiasCurrent, broadcast);
    ok = adarWrite(deviceIndex, REG_BIAS_CURRENT_TX_DRV, kTxDriverBiasCurrent, broadcast) && ok;
    ok = adarWrite(deviceIndex, REG_LOAD_WORKING, 0x2, broadcast) && ok;
    return ok;
}

uint8_t ADAR1000Manager::adarAdcRead(uint8_t deviceIndex, uint8_t broadcast) {
    if (!adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_ST_CONV, broadcast)) {
        DIAG_ERR("BF", "adarAdcRead(dev[%u]): ADC start-conversion write failed", deviceIndex);
        comm_stats_.adc_timeouts++;  // treat as a "no-result" event for caller observability
        return 0;
    }

    uint32_t t0 = HAL_GetTick();
    uint32_t polls = 0;
    uint8_t  ctrl  = 0;
    while (true) {
        if (!adarReadChecked(deviceIndex, REG_ADC_CONTROL, &ctrl)) {
            DIAG_ERR("BF", "adarAdcRead(dev[%u]): ADC poll read failed", deviceIndex);
            comm_stats_.adc_timeouts++;
            return 0;
        }
        if (ctrl & 0x01) break;
        polls++;
        if (HAL_GetTick() - t0 > 100) {
            DIAG_ERR("BF", "adarAdcRead(dev[%u]): ADC conversion TIMEOUT after %lu ms, %lu polls",
                     deviceIndex, (unsigned long)(HAL_GetTick() - t0), (unsigned long)polls);
            comm_stats_.adc_timeouts++;
            return 0;
        }
    }
    DIAG("BF", "adarAdcRead(dev[%u]): conversion done in %lu ms (%lu polls)",
         deviceIndex, (unsigned long)(HAL_GetTick() - t0), (unsigned long)polls);

    uint8_t out = 0;
    if (!adarReadChecked(deviceIndex, REG_ADC_OUT, &out)) {
        DIAG_ERR("BF", "adarAdcRead(dev[%u]): ADC output read failed", deviceIndex);
        comm_stats_.adc_timeouts++;
        return 0;
    }
    return out;
}
