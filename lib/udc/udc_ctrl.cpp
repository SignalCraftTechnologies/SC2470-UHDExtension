//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "udc/udc_ctrl.hpp"
#include "sc2470_constants.hpp"
#include "utility/version.hpp"
#include "utility/float_util.hpp"
#include "utility/range_util.hpp"
#include <uhd/features/spi_getter_iface.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/utils/log.hpp>

namespace udc {

udc_ctrl::udc_ctrl(uhd::rfnoc::radio_control::sptr radio, daughterboard::type db_type)
    : _radio(radio), _phy(std::make_shared<udc_phy>(radio, db_type))
{
    // Initialize Channels
    for (size_t chan = 0; chan < daughterboard::channel_count(db_type).value; chan++) {
        //* UHD assumes the extension is available for all channels of the DB
        //* Since this isn't always the case, skip IO one disconnected channels
        if (!_phy->is_connected(chan))
            continue;

        UHD_LOG_DEBUG(
            udc::NAME, "Initializing UDC [" << radio->get_slot_name() << ":" << chan << "]");
        const version hw(_phy->spi_send(chan, proto::hw_status(proto::hw_status::id::HW_REV), std::string(__func__)) & 0xFFFF);
        UHD_LOG_DEBUG(udc::NAME, "HW Version: " << hw.to_string());

        // Assert Attenuator LE to enable gain updates
        set_attn_latch(chan, true);
        // Initialize the Attenuator LE and ATR triggers
        const uint16_t triggers = proto::value::EXT_TRIG_ENABLE;
        _phy->spi_send(chan, proto::hw_control(proto::hw_control::id::EXT_TRIG, triggers), std::string(__func__));
        // Set default gain        
        const proto::tdd_gain::tx_attn tx_min_gain{_current_tx_gain_range.step(), _current_tx_gain_range.start()};
        _phy->spi_send(chan, proto::tdd_gain(tx_min_gain), std::string(__func__));
        const proto::tdd_gain::rx_attn rx_min_gain{_current_rx_gain_range.step(), _current_rx_gain_range.start()};
        _phy->spi_send(chan, proto::tdd_gain(rx_min_gain), std::string(__func__));
        // Set default freqs
        const udc::path rx_path = {chan, uhd::direction_t::RX_DIRECTION};
        const udc::path tx_path = {chan, uhd::direction_t::TX_DIRECTION};
        udc::freq_set rx_freqs = {RF_FREQ_DEFAULT, IF_FREQ_DEFAULT, 0.0};
        udc::freq_set tx_freqs = {RF_FREQ_DEFAULT, IF_FREQ_DEFAULT, 0.0};
        set_lo_freq(rx_path, rx_freqs);
        set_lo_freq(tx_path, tx_freqs);
    }
}

void udc_ctrl::set_antenna(const path& path, const proto::value::rf_path ant)
{
    if (!_phy->is_connected(path.chan))
        return;

    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RF_PATH, ant), std::string(__func__));
}

void udc_ctrl::set_ref_source(const path& path, const proto::value::ref_source ref)
{
    if (!_phy->is_connected(path.chan))
        return;

    // Need to store and reprogram LO frequency after...
    freq_set stored_freqs = get_freqs(path);
    set_lo_active_path(path);
    _phy->spi_send(path.chan, proto::lo_control(proto::lo_control::id::REFPLL_CONFIG, ref), std::string(__func__));
    stored_freqs.lo_freq = 0.0;
    set_lo_freq(path, stored_freqs);
}

void udc_ctrl::set_tx_gain(size_t chan, double gain)
{
    if (!_phy->is_connected(chan))
        return;

    set_active_path({chan, uhd::TX_DIRECTION});
    
    const proto::tdd_gain::tx_attn setting{_current_tx_gain_range.step(), utility::fixed_uhd_meta_range_clip(_current_tx_gain_range, gain, true)};

    _phy->spi_send(chan, proto::tdd_gain(setting), std::string(__func__));
}

void udc_ctrl::set_rx_gain(size_t chan, double gain)
{
    if (!_phy->is_connected(chan))
        return;

    set_active_path({chan, uhd::RX_DIRECTION});

    const proto::tdd_gain::rx_attn setting{_current_rx_gain_range.step(), utility::fixed_uhd_meta_range_clip(_current_rx_gain_range, gain, true)};
    _phy->spi_send(chan, proto::tdd_gain(setting), std::string(__func__));
}

freq_set udc_ctrl::get_freqs(const path& path)
{
    if (!_phy->is_connected(path.chan))
        return {0,0,0};

    set_active_path(path);

    udc::proto::freq_millihz lo_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_L), std::string(__func__))
    );
   
    udc::proto::freq_millihz rf_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_L), std::string(__func__))
    );

    udc::proto::freq_millihz if_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_L), std::string(__func__))
    );

    freq_set result(rf_millihz.full / 1000.0, if_millihz.full / 1000.0, lo_millihz.full / 1000.0);
    return result;
}

freq_set udc_ctrl::set_lo_freq(const path& path, freq_set& freqs)
{
    if (!_phy->is_connected(path.chan))
        return {0,0,0};
    if(!utility::isAlmostEqualToZero(freqs.lo_freq, 1e-9)) {
        UHD_LOG_WARNING(udc::NAME, "[" << udc::NAME << "] lo frequency specified in set_lo_freq - ignoring, will calculate new value");
    }
    validate_freq_set(freqs);

    set_active_path(path);

    udc::proto::freq_millihz rf_millihz = udc::proto::freq_millihz(freqs.rf_freq);
    udc::proto::freq_millihz if_millihz = udc::proto::freq_millihz(freqs.if_freq);

    // Load RF + IF
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_H, rf_millihz.high_bits), std::string(__func__));
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_M, rf_millihz.med_bits), std::string(__func__));
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_L, rf_millihz.low_bits), std::string(__func__));
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_H, if_millihz.high_bits), std::string(__func__));
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_M, if_millihz.med_bits), std::string(__func__));
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_L, if_millihz.low_bits), std::string(__func__));

    // Apply RF + IF
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::APPLY_FREQ, 
        (proto::value::apply_freq::APPLY_FREQ_RFFREQ | proto::value::apply_freq::APPLY_FREQ_IFFREQ)), std::string(__func__));

    // Load final RF + IF + LO
    _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOAD_FREQ, 
        (proto::value::load_freq::LOAD_FREQ_RFFREQ | proto::value::load_freq::LOAD_FREQ_IFFREQ | proto::value::load_freq::LOAD_FREQ_LOFREQ)), std::string(__func__));

    udc::proto::freq_millihz lo_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::LOFREQ_L), std::string(__func__))
    );
   
    rf_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::RFFREQ_L), std::string(__func__))
    );

    if_millihz = udc::proto::freq_millihz(
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_H), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_M), std::string(__func__)),
        _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::IFFREQ_L), std::string(__func__))
    );

    freq_set result(rf_millihz.full / 1000.0, if_millihz.full / 1000.0, lo_millihz.full / 1000.0);

    // Update gain range
    if (path.trx == uhd::TX_DIRECTION) 
    {
        _current_tx_gain_range = proto::gain_set(
            _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::GAIN_LIM_MIN), std::string(__func__)),
            _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::GAIN_LIM_MAX), std::string(__func__))
        ).range;
    }
    else
    {
        _current_rx_gain_range = proto::gain_set(
            _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::GAIN_LIM_MIN), std::string(__func__)),
            _phy->spi_send(path.chan, proto::hw_control(proto::hw_control::id::GAIN_LIM_MAX), std::string(__func__))
        ).range;
    }
    return result;  
}

void udc_ctrl::set_attn_latch(const size_t chan, const bool enable)
{
    _phy->set_attn_latch(chan, enable);
}

bool udc_ctrl::get_attn_latch(const size_t chan)
{
    return _phy->get_attn_latch(chan);
}

uhd::gain_range_t udc_ctrl::get_tx_gain_range(const size_t chan) const
{
    return _current_tx_gain_range;
}

uhd::gain_range_t udc_ctrl::get_rx_gain_range(const size_t chan) const
{
    return _current_rx_gain_range;
}

uint32_t udc_ctrl::direct_io(const size_t chan, const uint32_t data)
{
    return _phy->spi_send(chan, data, std::string(__func__));
}

void udc_ctrl::set_active_path(const path& path)
{
    const auto active = path.trx == uhd::TX_DIRECTION ? proto::value::ACT_PATH_TX
                                                      : proto::value::ACT_PATH_RX;
    _phy->spi_send(path.chan, proto::command(proto::command::id::ACT_PATH, active), std::string(__func__));
}

void udc_ctrl::set_lo_active_path(const path& path)
{
    const auto active = path.trx == uhd::TX_DIRECTION ? proto::value::LO_ACT_PATH_TX
                                                      : proto::value::LO_ACT_PATH_RX;
    _phy->spi_send(path.chan, proto::command(proto::command::id::LO_ACT_PATH, active), std::string(__func__));
}

void udc_ctrl::validate_freq_set(freq_set& freqs)
{
    // Freqs must obey the frequency equation RF = LO + IF or (bypass mode)  RF = IF
    // And each must be within settable range for SC2470.
    double validationEpsilon = 2e-3; // The system can return a frequency a tenth of a hz off from what was programmed.  This is ok.
    uhd::freq_range_t if_range;
    if(utility::isAlmostEqual(freqs.rf_freq, freqs.if_freq, validationEpsilon)) {
        // Bypass mode
        if_range = _phy->get_daughterboard_type() == daughterboard::type::zbx ? IF_FREQ_X410_BYPASS_RANGE : IF_FREQ_X440_BYPASS_RANGE;
        if(!utility::isAlmostEqual(if_range.clip(freqs.if_freq), freqs.if_freq, validationEpsilon)) {
            std::stringstream msg;
            msg << "[" << udc::NAME << "] Bypass frequency " << freqs.if_freq << "is out of allowable range of " << if_range.to_pp_string();
            UHD_LOG_ERROR(udc::NAME, msg.str());
            throw uhd::runtime_error(msg.str());
        }
        freqs.if_freq = freqs.rf_freq; // Eliminate any small difference.

    } else {
        // Non bypass mode
        // Need to allow 
        if_range = _phy->get_daughterboard_type() == daughterboard::type::zbx ? IF_FREQ_X410_RANGE : IF_FREQ_X440_RANGE;
        double lo_calc = freqs.rf_freq - freqs.if_freq;
        if(!utility::isAlmostEqual(RF_FREQ_RANGE.clip(freqs.rf_freq), freqs.rf_freq, validationEpsilon)) {
            std::stringstream msg;
            msg << "[" << udc::NAME << "] RF frequency " << freqs.rf_freq << "is out of allowable range of " << RF_FREQ_RANGE.to_pp_string();
            UHD_LOG_ERROR(udc::NAME, msg.str());
            throw uhd::runtime_error(msg.str());
        } 
        if(!utility::isAlmostEqual(if_range.clip(freqs.if_freq), freqs.if_freq, validationEpsilon)) {
            std::stringstream msg;
            msg << "[" << udc::NAME << "] IF frequency " << freqs.if_freq << "is out of allowable range of " << if_range.to_pp_string();
            UHD_LOG_ERROR(udc::NAME,  msg.str());
            throw uhd::runtime_error(msg.str());
        }
        if(!utility::isAlmostEqual(LO_FREQ_RANGE.clip(lo_calc), lo_calc, validationEpsilon)) {
            std::stringstream msg;
            msg << "[" << udc::NAME << "] LO frequency " << lo_calc << "is out of allowable range of " << LO_FREQ_RANGE.to_pp_string();
            UHD_LOG_ERROR(udc::NAME, msg.str());
            throw uhd::runtime_error(msg.str());
        }
    }
    freqs.if_freq = utility::fixed_uhd_meta_range_clip(if_range, freqs.if_freq, true);
    freqs.rf_freq = utility::fixed_uhd_meta_range_clip(RF_FREQ_RANGE, freqs.rf_freq, true);
    freqs.lo_freq = utility::fixed_uhd_meta_range_clip(LO_FREQ_RANGE, freqs.lo_freq, true);

}

uhd::gain_range_t udc_ctrl::get_safe_fe_gain_range(const path& path, uhd::gain_range_t fe_gain_range, uhd::gain_range_t fe_power_range) {
    double max_safe_fe_gain = path.trx ? std::floor(TX_IN_ABS_MAX_POWER + fe_gain_range.start() - fe_power_range.start()) : 1000;
    double min_safe_fe_gain = path.trx ? -1000 : std::ceil(-1.0 * ZBX_RX_OUT_MAX_POWER + fe_power_range.stop() - fe_gain_range.start()); 
    return uhd::gain_range_t(std::max(min_safe_fe_gain, fe_gain_range.start()), std::min(max_safe_fe_gain, fe_gain_range.stop()), fe_gain_range.step());
}

uhd::gain_range_t udc_ctrl::get_safe_fe_power_range(const path& path, uhd::gain_range_t fe_power_range) {
    return path.trx ? uhd::gain_range_t(fe_power_range.start(), TX_IN_ABS_MAX_POWER) : uhd::gain_range_t(fe_power_range.start(), ZBX_RX_OUT_MAX_POWER, 0.0);
}

double udc_ctrl::get_zero_gain_power_reference(const path& path, uhd::gain_range_t fe_gain_range, uhd::gain_range_t fe_power_range) {
    uhd::gain_range_t safe_fe_gain_range = get_safe_fe_gain_range(path, fe_gain_range, fe_power_range);
    uhd::gain_range_t safe_fe_power_range = get_safe_fe_power_range(path, fe_power_range);
    return safe_fe_power_range.stop() + (path.trx ? (-1.0 * safe_fe_gain_range.stop()) : safe_fe_gain_range.start());
}


uhd::gain_range_t udc_ctrl::get_safe_udc_gain_range(const bool hi_power_protection, const path& path) {
    uhd::gain_range_t udc_gain_range = path.trx ? get_tx_gain_range(path.chan) : get_rx_gain_range(path.chan);
    return uhd::gain_range_t(udc_gain_range.start(), hi_power_protection ? (path.trx ? HIGH_POWER_PROT_EN_TX_MAX : HIGH_POWER_PROT_EN_RX_MAX) : udc_gain_range.stop(), udc_gain_range.step()); 

}

uhd::gain_range_t udc_ctrl::get_safe_overall_gain_range(const bool hi_power_protection, const uhd::gain_range_t& safe_fe_gain_range, const path& path) {
    uhd::gain_range_t safe_udc_gain_range = get_safe_udc_gain_range(hi_power_protection, path);
    return uhd::gain_range_t(safe_udc_gain_range.start() + safe_fe_gain_range.start(), safe_udc_gain_range.stop() + safe_fe_gain_range.stop(), safe_udc_gain_range.step());
}

uhd::gain_range_t udc_ctrl::get_safe_udc_power_range(const bool hi_power_protection, const uhd::gain_range_t& safe_fe_power_range, const path& path) {
    uhd::gain_range_t safe_udc_gain_range = get_safe_udc_gain_range(hi_power_protection, path);
    return path.trx ? uhd::gain_range_t(safe_fe_power_range.start() + safe_udc_gain_range.start(), safe_fe_power_range.stop() + safe_udc_gain_range.stop(), safe_udc_gain_range.step())
                  : uhd::gain_range_t(safe_fe_power_range.start() - safe_udc_gain_range.stop(),
                        safe_fe_power_range.stop() - safe_udc_gain_range.start(), safe_udc_gain_range.step());
}

} // namespace udc
