//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once


#include "types/path.hpp"
#include "types/daughterboard.hpp"
#include "udc/udc_phy.hpp"
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/serial.hpp>
#include <array>
#include <map>

namespace udc {

struct freq_set
{
    double rf_freq = 0;
    double if_freq = 0;
    double lo_freq = 0; 

    freq_set(double rf_init, double if_init)
    {
        rf_freq = rf_init;
        if_freq = if_init;
        lo_freq = 0;
    }
    freq_set(double rf_init, double if_init, double lo_init)
    {
        rf_freq = rf_init;
        if_freq = if_init;
        lo_freq = lo_init;
    }
};

// This class contains the high-level UDC functionality
class udc_ctrl final
{
private:
    uhd::rfnoc::radio_control::sptr _radio;
    udc_phy::sptr _phy;
    uhd::meta_range_t _current_tx_gain_range = DEFAULT_TX_GAIN_RANGE;
    uhd::meta_range_t _current_rx_gain_range = DEFAULT_RX_GAIN_RANGE;

public:
    using sptr = std::shared_ptr<udc_ctrl>;
    udc_ctrl(uhd::rfnoc::radio_control::sptr radio, daughterboard::type db_type);

    void set_antenna(const path& path, const proto::value::rf_path ant);
    void set_ref_source(const path& path, const proto::value::ref_source ref);
    void set_rx_gain(size_t chan, double gain);
    void set_tx_gain(size_t chan, double gain);
    freq_set set_lo_freq(const path& path, freq_set& freqs);

    void set_attn_latch(const size_t chan, const bool enable);
    bool get_attn_latch(const size_t chan);

    uhd::gain_range_t get_tx_gain_range(const size_t chan) const;
    uhd::gain_range_t get_rx_gain_range(const size_t chan) const;

    uint32_t direct_io(const size_t chan, const uint32_t data);

    uhd::gain_range_t get_safe_fe_gain_range(const path& path, uhd::gain_range_t fe_gain_range, uhd::gain_range_t fe_power_range);
    uhd::gain_range_t get_safe_fe_power_range(const path& path, uhd::gain_range_t fe_power_range);
    double get_zero_gain_power_reference(const path& path, uhd::gain_range_t fe_gain_range, uhd::gain_range_t fe_power_range);
    uhd::gain_range_t get_safe_udc_gain_range(const bool hi_power_protection, const path& path);
    uhd::gain_range_t get_safe_overall_gain_range(const bool hi_power_protection, const uhd::gain_range_t& safe_fe_gain_range, const path& path);
    uhd::gain_range_t get_safe_udc_power_range(const bool hi_power_protection, const uhd::gain_range_t& safe_fe_power_range, const path& path);

private:
    void set_active_path(const path& path);
    void set_lo_active_path(const path& path);
    void set_conv_gain(const path& path, const double voltage_mV);
    void validate_freq_set(freq_set& freqs);
    freq_set get_freqs(const path& path);

};
} // namespace udc
