//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/types/ranges.hpp>
#include <uhd/usrp/zbx_tune_map_item.hpp>
#include <string>
#include <vector>

namespace udc {

// Extension Name
static constexpr char NAME[] = "sc2470";

// Default power reference settings resulting in minimum gain/power levels.
static constexpr double RX_DEFAULT_POWER_REF = 10;
static constexpr double TX_DEFAULT_POWER_REF = -60;

// The ZBX is capable of TX output power levels exceeding the UDC TX input no-damage
// specification of 5 dBm. For good measure, back this off by another 5 dB.
static constexpr double TX_IN_ABS_MAX_POWER = 0;
// TODO: Update this based on recent SCM discoveries
//! JKEELING - Not sure what the SCM discovery was but scm is set to TX_IN_TYP_MAX_POWER = -5 + ((8 + 15) / 2);
//! Will revisit this cutoff point when we have HW
// The X410 specification indicates the TX EVM is optimized between -15 and 5 dBm for a 5G
// NR 100 MHz waveform. Limiting the ZBX output power to -10 dBm provides a margin for other
// waveform types. The UDC has sufficient TX gain to increase the ZBX output level up to
// the UDC TX output power specification (Linear Power assuming 10 dB PAR).
static constexpr double ZBX_TX_IN_TYP_MAX_POWER = -10;
// The X410 specification states a maximum RX input power of 0 dBm.
// The X440 specification states a maximum RX input power of 10 dBm.
// Use the lower of the two for now.
static constexpr double ZBX_RX_OUT_MAX_POWER = 0;

// TX/RX Antennas controlled simultaneously
static const std::vector<std::string> ANTENNAS = {"FDD", "TDD"};
// LO Sources
static const std::vector<std::string> LO_SOURCES = {"internal"};
// Reference Sources
static const std::vector<std::string> REF_SOURCES = {"internal", "external10MHz", "external100MHz"};

// Frequency Conversion Ranges
static constexpr double IF_FREQ_BYPASS_MIN = 50e6;
static constexpr double IF_FREQ_MIN = 700e6;
static constexpr double IF_FREQ_MAX = 6e9;
static constexpr double IF_FREQ_DEFAULT = 2e9;
static constexpr double IF_FREQ_X410_MAX = 6e9;
static const uhd::freq_range_t IF_FREQ_OVERALL_RANGE{IF_FREQ_BYPASS_MIN, IF_FREQ_MAX, 0.001};
static const uhd::freq_range_t IF_FREQ_X410_RANGE{IF_FREQ_MIN, IF_FREQ_X410_MAX, 0.001};
static const uhd::freq_range_t IF_FREQ_X410_BYPASS_RANGE{IF_FREQ_BYPASS_MIN, IF_FREQ_X410_MAX, 0.001};
static constexpr double IF_FREQ_X440_MAX = 4e9;
static const uhd::freq_range_t IF_FREQ_X440_RANGE{IF_FREQ_MIN, IF_FREQ_X440_MAX, 0.001};
static const uhd::freq_range_t IF_FREQ_X440_BYPASS_RANGE{IF_FREQ_BYPASS_MIN, IF_FREQ_X440_MAX, 0.001};
static constexpr double RF_FREQ_DEFAULT = 10e9;
static constexpr double RF_FREQ_MIN = 50e6;
static constexpr double RF_FREQ_NON_BYPASS_MIN = 6.7e9;
static constexpr double RF_FREQ_MAX = 26e9;
static const uhd::freq_range_t RF_FREQ_RANGE{RF_FREQ_MIN, RF_FREQ_MAX, 0.001};
static const uhd::freq_range_t RF_FREQ_NON_BYPASS_RANGE{RF_FREQ_NON_BYPASS_MIN, RF_FREQ_MAX, 0.001};
static constexpr double LO_FREQ_DEFAULT = 8e9;
static constexpr double LO_FREQ_MIN = 6e9;
static constexpr double LO_FREQ_MAX = 25.3e9;
static const uhd::freq_range_t LO_FREQ_RANGE{LO_FREQ_MIN, LO_FREQ_MAX, 0.001};


// Attenuator Ranges
static constexpr double DEFAULT_GAIN_STEP = 0.5;
static constexpr double DEFAULT_GAIN_TX_MIN = -10.0;
static constexpr double DEFAULT_GAIN_TX_MAX = 30.0;
static constexpr double DEFAULT_GAIN_RX_MIN = 10.0;
static constexpr double DEFAULT_GAIN_RX_MAX = 60.0;

static constexpr double HIGH_POWER_PROT_EN_TX_MAX = 20.0;
static constexpr double HIGH_POWER_PROT_EN_RX_MAX = 40.0; 

static const uhd::meta_range_t DEFAULT_TX_GAIN_RANGE{DEFAULT_GAIN_TX_MIN, DEFAULT_GAIN_TX_MAX, DEFAULT_GAIN_STEP};
static const uhd::meta_range_t DEFAULT_RX_GAIN_RANGE{DEFAULT_GAIN_RX_MIN, DEFAULT_GAIN_RX_MAX, DEFAULT_GAIN_STEP};

} // namespace udc
