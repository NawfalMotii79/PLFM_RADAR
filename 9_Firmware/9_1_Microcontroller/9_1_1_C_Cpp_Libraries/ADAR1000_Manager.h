// ADAR1000_Manager.h
#ifndef ADAR1000_MANAGER_H
#define ADAR1000_MANAGER_H

#include "stm32f7xx_hal.h"
#include "main.h"

#include <vector>
#include <memory>

class ADAR1000Manager {
public:
    enum class BeamDirection {
        TX = 0,
        RX = 1
    };

    struct BeamConfig {
        float angle_degrees;
        uint8_t phase_settings[4];
        uint8_t gain_settings[4];
        uint32_t dwell_time_ms;

        BeamConfig() : angle_degrees(0), dwell_time_ms(100) {
            for(int i = 0; i < 4; i++) {
                phase_settings[i] = 0;
                gain_settings[i] = 0x7F;
            }
        }

        BeamConfig(float angle, uint32_t dwell = 100) : angle_degrees(angle), dwell_time_ms(dwell) {
            for(int i = 0; i < 4; i++) {
                phase_settings[i] = 0;
                gain_settings[i] = 0x7F;
            }
        }
    };

    // Communication health counters. Incremented by checked SPI helpers; queryable
    // for telemetry/observability without changing the boolean public contract.
    // PR2 may promote a richer OpStatus enum; today the policy is bool returns
    // for control flow + this struct for trends.
    struct CommStats {
        uint32_t writes_ok;
        uint32_t writes_fail;
        uint32_t reads_ok;
        uint32_t reads_fail;
        uint32_t adc_timeouts;
        uint8_t  last_fail_dev;   // device index of most recent failure (0xFF if none)
    };

    ADAR1000Manager();
    ~ADAR1000Manager();

    // System Management
    bool powerUpSystem();
    bool powerDownSystem();
    bool initializeAllDevices();
    bool performSystemCalibration();

    // Mode Switching
    bool switchToTXMode();
    bool switchToRXMode();
    bool fastTXMode();
    bool fastRXMode();
    bool pulseTXMode();
    bool pulseRXMode();

    // Beam Steering
    bool setBeamAngle(float angle_degrees, BeamDirection direction);
    bool setCustomBeamPattern(const uint8_t phase_settings[16], const uint8_t gain_settings[4], BeamDirection direction);
    bool setCustomBeamPattern16(const uint8_t phase_pattern[16], BeamDirection direction);

    // Beam Sweeping
    void startBeamSweeping();
    void stopBeamSweeping();
    void updateBeamPosition();
    void setBeamSequence(const std::vector<BeamConfig>& sequence, BeamDirection direction);
    void clearBeamSequence(BeamDirection direction);

    // Device Control
    bool setAllDevicesTXMode();
    bool setAllDevicesRXMode();
    bool setADTR1107Mode(BeamDirection direction);
    bool setADTR1107Control(bool tx_mode);

    // Monitoring and Diagnostics
    // readTemperature returns NaN when the on-chip ADC times out, so callers
    // can distinguish a hung chip from a real cold reading via std::isnan().
    float readTemperature(uint8_t deviceIndex);
    bool verifyDeviceCommunication(uint8_t deviceIndex);
    uint8_t readRegister(uint8_t deviceIndex, uint32_t address);
    bool writeRegister(uint8_t deviceIndex, uint32_t address, uint8_t value);

    // Communication health observability
    const CommStats& getCommStats() const { return comm_stats_; }
    void resetCommStats();

    // Configuration
    void setSwitchSettlingTime(uint32_t us);
    void setFastSwitchMode(bool enable);
    void setBeamDwellTime(uint32_t ms);

    // Getters
    bool isBeamSweepingActive() const { return beam_sweeping_active_; }
    uint8_t getCurrentBeamIndex() const { return current_beam_index_; }
    BeamDirection getCurrentMode() const { return current_mode_; }
    uint32_t getLastSwitchTime() const { return last_switch_time_us_; }

    struct ADAR1000Device {
        uint8_t dev_addr;
        bool initialized;
        BeamDirection current_mode;
        float temperature;

        ADAR1000Device(uint8_t addr)
            : dev_addr(addr), initialized(false), current_mode(BeamDirection::RX), temperature(25.0f) {
        }
    };

    // Configuration
    bool fast_switch_mode_ = false;
    uint32_t switch_settling_time_us_ = 50;
    uint32_t beam_dwell_time_ms_ = 100;
    uint32_t last_switch_time_us_ = 0;

    // Device Management
    std::vector<std::unique_ptr<ADAR1000Device>> devices_;
    BeamDirection current_mode_ = BeamDirection::RX;

    // Comm health counters (zeroed in resetCommStats()).
    CommStats comm_stats_ = {0, 0, 0, 0, 0, 0xFF};

    // Beam Sweeping
    std::vector<BeamConfig> tx_beam_sequence_;
    std::vector<BeamConfig> rx_beam_sequence_;
    uint8_t current_beam_index_ = 0;
    bool beam_sweeping_active_ = false;
    uint32_t last_beam_update_time_ = 0;

    // Vector Modulator lookup tables (see ADAR1000_Manager.cpp for provenance).
    // Indexed as VM_*[phase % 128] on a uniform 2.8125 deg grid.
    // No VM_GAIN[] table exists: VM magnitude is bits [4:0] of the I/Q bytes
    // themselves; per-channel VGA gain uses a separate register.
    static const uint8_t VM_I[128];
    static const uint8_t VM_Q[128];

    // Named defaults for the ADTR1107 and ADAR1000 power sequence.
    static constexpr uint8_t kDefaultTxVgaGain = 0x7F;
    static constexpr uint8_t kDefaultRxVgaGain = 30;
    static constexpr uint8_t kLnaBiasOff = 0x00;
    static constexpr uint8_t kLnaBiasOperational = 0x30;
    static constexpr uint8_t kPaBiasTxSafe = 0x5D;
    static constexpr uint8_t kPaBiasIdqCalibration = 0x0D;
    static constexpr uint8_t kPaBiasOperational = 0x7F;
    static constexpr uint8_t kPaBiasRxSafe = 0x20;
    static constexpr uint8_t kTxBiasCurrent = 0x2D;
    static constexpr uint8_t kTxDriverBiasCurrent = 0x06;

    // Private Methods
    bool initializeSingleDevice(uint8_t deviceIndex);
    bool initializeADTR1107Sequence();
    void calculatePhaseSettings(float angle_degrees, uint8_t phase_settings[4]);
    void delayUs(uint32_t microseconds);

    // Power Management
    // PA/LNA supply rails are pure GPIO toggles -- those stay void.
    // setPABias/setLNABias issue per-device SPI writes, so they propagate.
    void enablePASupplies();
    void disablePASupplies();
    void enableLNASupplies();
    void disableLNASupplies();
    bool setPABias(bool enable);
    bool setLNABias(bool enable);

    // SPI Communication
    // setChipSelect is a pure GPIO toggle (no failure mode in HAL_GPIO_WritePin).
    // Everything else returns true on success, false on SPI failure or invalid index.
    void setChipSelect(uint8_t deviceIndex, bool state);
    bool spiTransfer(uint8_t* txData, uint8_t* rxData, uint32_t size);
    bool adarWrite(uint8_t deviceIndex, uint32_t mem_addr, uint8_t data, uint8_t broadcast);
    // adarRead returns the register byte; on SPI failure it returns 0 and the
    // failure is reflected in comm_stats_.reads_fail (callers wanting an
    // explicit ok/fail signal should use adarReadChecked below).
    uint8_t adarRead(uint8_t deviceIndex, uint32_t mem_addr);
    bool adarReadChecked(uint8_t deviceIndex, uint32_t mem_addr, uint8_t* out);
    bool adarSetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast);
    bool adarResetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast);
    bool adarSoftReset(uint8_t deviceIndex);
    bool adarWriteConfigA(uint8_t deviceIndex, uint8_t flags, uint8_t broadcast);
    bool adarSetRamBypass(uint8_t deviceIndex, uint8_t broadcast);

    // Channel Configuration
    bool adarSetRxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast);
    bool adarSetTxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast);
    bool adarSetRxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast);
    bool adarSetTxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast);
    bool adarSetTxBias(uint8_t deviceIndex, uint8_t broadcast);
    // adarAdcRead returns 0 on timeout AND increments comm_stats_.adc_timeouts.
    // readTemperature() detects the timeout via that counter delta.
    uint8_t adarAdcRead(uint8_t deviceIndex, uint8_t broadcast);
    bool setTRSwitchPosition(uint8_t deviceIndex, bool tx_mode);

private:

};

// Register Definitions
#define BROADCAST_OFF                      0
#define BROADCAST_ON                       1

#define REG_INTERFACE_CONFIG_A             0x000
#define REG_SCRATCHPAD                     0x00A
#define REG_CH1_RX_GAIN                    0x010
#define REG_CH2_RX_GAIN                    0x011
#define REG_CH3_RX_GAIN                    0x012
#define REG_CH4_RX_GAIN                    0x013
#define REG_CH1_RX_PHS_I                   0x014
#define REG_CH1_RX_PHS_Q                   0x015
#define REG_CH2_RX_PHS_I                   0x016
#define REG_CH2_RX_PHS_Q                   0x017
#define REG_CH3_RX_PHS_I                   0x018
#define REG_CH3_RX_PHS_Q                   0x019
#define REG_CH4_RX_PHS_I                   0x01A
#define REG_CH4_RX_PHS_Q                   0x01B
#define REG_CH1_TX_GAIN                    0x01C
#define REG_CH2_TX_GAIN                    0x01D
#define REG_CH3_TX_GAIN                    0x01E
#define REG_CH4_TX_GAIN                    0x01F
#define REG_CH1_TX_PHS_I                   0x020
#define REG_CH1_TX_PHS_Q                   0x021
#define REG_CH2_TX_PHS_I                   0x022
#define REG_CH2_TX_PHS_Q                   0x023
#define REG_CH3_TX_PHS_I                   0x024
#define REG_CH3_TX_PHS_Q                   0x025
#define REG_CH4_TX_PHS_I                   0x026
#define REG_CH4_TX_PHS_Q                   0x027
#define REG_LOAD_WORKING                   0x028
#define REG_PA_CH1_BIAS_ON                 0x029
#define REG_PA_CH2_BIAS_ON                 0x02A
#define REG_PA_CH3_BIAS_ON                 0x02B
#define REG_PA_CH4_BIAS_ON                 0x02C
#define REG_LNA_BIAS_ON                    0x02D
#define REG_RX_ENABLES                     0x02E
#define REG_TX_ENABLES                     0x02F
#define REG_MISC_ENABLES                   0x030
#define REG_SW_CONTROL                     0x031
#define REG_ADC_CONTROL                    0x032
#define REG_ADC_OUT                        0x033
#define REG_BIAS_CURRENT_TX                0x036
#define REG_BIAS_CURRENT_TX_DRV            0x037
#define REG_MEM_CTL                        0x038
#define REG_PA_CH1_BIAS_OFF                0x046
#define REG_PA_CH2_BIAS_OFF                0x047
#define REG_PA_CH3_BIAS_OFF                0x048
#define REG_PA_CH4_BIAS_OFF                0x049
#define REG_LNA_BIAS_OFF                   0x04A

// Register Constants
#define INTERFACE_CONFIG_A_SDO_ACTIVE      ((1 << 4) | (1 << 3))
#define ADAR1000_ADC_2MHZ_CLK              0x00
#define ADAR1000_ADC_EN                    0x60
#define ADAR1000_ADC_ST_CONV               0x70
#define MEM_CTRL_BIAS_RAM_BYPASS           (1 << 5)
#define MEM_CTRL_BEAM_RAM_BYPASS           (1 << 6)
#define LD_WRK_REGS_LDTX_OVERRIDE          (1 << 1)

#endif // ADAR1000_MANAGER_H
