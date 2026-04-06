//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "sc2470_expert.hpp"
#include "sc2470_constants.hpp"
#include "utility/float_util.hpp"
#include "utility/range_util.hpp"

#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/utils/log.hpp>
#include <algorithm>

using namespace uhd;

namespace udc {

void udc_antenna_expert::resolve(void)
{
    UHD_LOG_TRACE(udc::NAME, "udc_antenna_expert::resolve");

    std::string antenna = _antenna_in;
    if (std::find(ANTENNAS.begin(), ANTENNAS.end(), antenna) == ANTENNAS.end()) {
        antenna = ANTENNAS[0];
        throw uhd::value_error("Set antenna to invalid value!");
    }
    UHD_LOG_DEBUG(udc::NAME, _path << " Setting UDC Antenna: " << antenna);

    _udc->set_antenna(
        _path, (antenna == "FDD") ? proto::value::RF_PATH_FDD : proto::value::RF_PATH_TDD);
    _antenna_out = antenna;
}

void udc_reference_expert::resolve(void)
{
    UHD_LOG_TRACE(udc::NAME, "udc_reference_expert::resolve");

    std::string reference = _reference_source_in;
    if (std::find(REF_SOURCES.begin(), REF_SOURCES.end(), reference) == REF_SOURCES.end()) {
        reference = REF_SOURCES[0];
        throw uhd::value_error("Set reference to invalid value!");
    }
    UHD_LOG_DEBUG(udc::NAME, " Setting UDC Reference Source: " << reference);

    proto::value::ref_source ref = (reference == "internal") ? proto::value::REF_SOURCE_INT : (reference == "external10MHz") ? proto::value::REF_SOURCE_EXT_10MHZ : proto::value::REF_SOURCE_EXT_100MHZ;
    _udc->set_ref_source({_path.chan, uhd::direction_t::RX_DIRECTION}, ref);
    _udc->set_ref_source({_path.chan, uhd::direction_t::TX_DIRECTION}, ref);
}

void udc_freq_expert::resolve(void)
{
    UHD_LOG_TRACE(udc::NAME, "udc_freq_expert::resolve");
    UHD_LOG_TRACE(udc::NAME, _path << " udc_freq_expert: Input Hi Power Protection: " << _high_power_prot_in);
    UHD_LOG_TRACE(udc::NAME, _path << " udc_freq_expert: Input RF: " << _rf_frequency_in);
    UHD_LOG_TRACE(udc::NAME, _path << " udc_freq_expert: Input IF Override: " << _if_frequency_override_in);

        
    double if_freq_override = _if_frequency_override_in;
    _if_frequency_override_out = 0.0; // Immediately set to zero so it's a one-time override.

    bool bypass = calc_bypass(_high_power_prot_in, _rf_frequency_in);
    
    double rf_freq = utility::fixed_uhd_meta_range_clip(bypass ? RF_FREQ_RANGE : RF_FREQ_NON_BYPASS_RANGE, _rf_frequency_in, true);
    double if_freq;
    if(bypass) {
        if_freq = rf_freq;
    } else {
        double if_freq_preferred = (if_freq_override ? if_freq_override : _prefer_nyquist ? calc_nyquist(rf_freq) : 0);
        if_freq = calc_if(rf_freq, if_freq_preferred);
    }

    if_freq = (_path.trx == uhd::RX_DIRECTION) ? _radio->set_rx_frequency(if_freq, _path.chan) : _radio->set_tx_frequency(if_freq, _path.chan);

    udc::freq_set freqs{rf_freq, if_freq};
    freqs = _udc->set_lo_freq(_path, freqs);

    _udc_bypass_out = bypass;
    _lo_frequency_out = freqs.lo_freq;
    _if_frequency_out = freqs.if_freq;
    _rf_frequency_out = freqs.rf_freq;

    UHD_LOG_DEBUG(udc::NAME, _path << " udc_freq_expert: Resolved UDC Bypass: " << _udc_bypass_out);
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_freq_expert: Resolved RF: " << _rf_frequency_out);
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_freq_expert: Resolved IF: " << _if_frequency_out);
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_freq_expert: Resolved LO: " << _lo_frequency_out);
}

uhd::freq_range_t udc_freq_expert::calc_non_bypass_if_range(double rf_freq) {
    double if_max = std::min(_if_full_range.stop(), rf_freq - LO_FREQ_MIN);
    double if_min = std::max(_if_full_range.start(), rf_freq - LO_FREQ_MAX);
    if(if_max < if_min) {
        throw runtime_error("This should not happen");
    }

    return uhd::freq_range_t(if_min, if_max, 0.001);
}

double udc_freq_expert::calc_nyquist(double rf_freq) {
    uhd::freq_range_t if_range = calc_non_bypass_if_range(rf_freq);
    double conversion_rate;
    size_t max_nyquist_zones;
    if (_path.trx == RX_DIRECTION) {
        conversion_rate   = _radio->get_rx_sensor("rfdc_rate", _path.chan).to_real();
        max_nyquist_zones = 3; // from app note
    } else {
        conversion_rate   = _radio->get_tx_sensor("rfdc_rate", _path.chan).to_real();
        max_nyquist_zones = 2; // from app note
    }

    double nyquist_bw = conversion_rate / 2;

    size_t nyquist_count = std::min(
        max_nyquist_zones, static_cast<size_t>(std::floor(_if_bypass_range.stop() / nyquist_bw)));

    // Select ideal nyquist zone that meets criteria
    size_t nyquist_index = 0;
    for (size_t i = 0; i < nyquist_count; i++) {
        // Assuming middle 80% of nyquist zone is usable, 10% cut below and above
        double nyquist_low_bound  = (i * nyquist_bw) + (nyquist_bw * 0.1);
        double nyquist_high_bound = (i * nyquist_bw) + (nyquist_bw * 0.9);
        double nyquist_mid        = (i * nyquist_bw) + nyquist_bw * 0.5;

        if (nyquist_low_bound > IF_FREQ_MIN && nyquist_high_bound < _if_bypass_range.stop()
            && nyquist_mid < if_range.stop() && nyquist_mid > if_range.start()) {
            nyquist_index = i;
            break;
        }
    }
    double nyquist_freq = (nyquist_index * nyquist_bw) + (nyquist_bw * 0.5);
    UHD_LOG_DEBUG(udc::NAME, _path << " Selected Nyquist Zone: " << (nyquist_index + 1));
    return nyquist_freq;
}

double udc_freq_expert::calc_if(double rf_freq, double if_freq_preferred) {
    double if_freq;
    uhd::freq_range_t if_range = calc_non_bypass_if_range(rf_freq);

    if (if_freq_preferred) {
        if_freq = utility::fixed_uhd_meta_range_clip(
            _if_full_range, std::min(if_range.stop(), std::max(if_range.start(), if_freq_preferred)), true);
    } else {
        if_freq = (if_range.start() + if_range.stop()) / 2.0;
    }
    return if_freq;
}

bool udc_freq_expert::calc_bypass(bool high_power_prot, double rf_freq) {
    // calc rf and bypass
    bool bypass;
    if (rf_freq <= _if_bypass_range.stop()) { // Bypass mode required if not high power mode
        if (high_power_prot == true) {
            bypass = false;
            UHD_LOG_ERROR(udc::NAME,  "The requested frequency should use the SC2470 in bypass mode. Bypass "
                         "mode is only available when high power protection is disabled. See "
                         "documentation or contact SCT for details.");
        } else {
            bypass = true;
        }
    } else {
        bypass = false;
    }
    return bypass;
}

// Resolution of gain/power:
// For RX:
//   - For best performance, utilize the UDC gain while minimizing the ZBX gain.
//   - Once the UDC gain is maximized, adjust the ZBX gain as required.
// For TX:
//   - For best performance, limit the ZBX output power to TX_IN_TYP_MAX_POWER.
//   - Set additional gain using the UDC once TX_IN_TYP_MAX_POWER is exceeded.
void zbx_gain_expert::resolve(void)
{
    UHD_LOG_DEBUG(udc::NAME, "zbx_gain_expert::resolve");
    if(_power_control_mode) {
        UHD_LOG_TRACE(udc::NAME, _path << " Resolving Power: " << _power_ref_in << " dBm");
    } else {
        UHD_LOG_TRACE(udc::NAME, _path << " Resolving Gain: " << _gain_in << " dB");
    }
    // UHD_LOG_TRACE(udc::NAME, _path << " Input High Power Protection: " << _high_power_prot_in);
    const auto chan = _path.chan;


    // calculate ranges and safe gain maximums
    uhd::gain_range_t fe_gain_range = _path.trx ? _radio->get_tx_gain_range(_path.chan) : _radio->get_rx_gain_range(_path.chan);
    uhd::gain_range_t overall_gain_range = _udc->get_overall_gain_range(_path, fe_gain_range);
    uhd::gain_range_t fe_power_range = _path.trx ? _radio->get_tx_power_range(_path.chan) : _radio->get_rx_power_range(_path.chan);
    uhd::gain_range_t safe_fe_gain_range =  _udc->get_safe_fe_gain_range(_path, fe_gain_range, fe_power_range);
    uhd::gain_range_t safe_fe_power_range =  _udc->get_safe_fe_power_range(_path, fe_power_range);
    double zero_gain_power_reference =  _udc->get_zero_gain_power_reference(_path, fe_gain_range, fe_power_range);
    uhd::gain_range_t safe_udc_power_range = _udc->get_safe_udc_power_range(_high_power_prot_in, safe_fe_power_range, _path);
    uhd::gain_range_t safe_udc_gain_range = _udc->get_safe_udc_gain_range(_high_power_prot_in, _path);
    uhd::gain_range_t safe_overall_gain_range = _udc->get_safe_overall_gain_range(_high_power_prot_in, safe_fe_gain_range, _path);
    
    double total_gain;
    if(_power_control_mode) {
        double safe_clipped_udc_power =
            utility::fixed_uhd_meta_range_clip(safe_udc_power_range, _power_ref_in, true);
        total_gain = _path.trx ? safe_clipped_udc_power - zero_gain_power_reference : zero_gain_power_reference - safe_clipped_udc_power;
    } else {
        total_gain = _gain_in;
    }
    // (TX-only) first use fe gain to step to ZBX_TX_IN_TYP_MAX_POWER
    // then use SC2470 gain to max
    // Then continue increase fe gain
    double unsafe_total_gain = utility::fixed_uhd_meta_range_clip(overall_gain_range, total_gain, true);
    total_gain = utility::fixed_uhd_meta_range_clip(safe_overall_gain_range, total_gain, true);
    if(_high_power_prot_in && ((unsafe_total_gain - total_gain) > (safe_udc_gain_range.step() / 2.0))) {
        std::stringstream msg;
        msg << "High Power Protection is enabled.  " << (_path.trx ? "TX" : "RX") << " gain is being limited. See documentation or contact SCT for details.";
        UHD_LOG_WARNING(udc::NAME, msg.str());
    }
    double udc_gain = safe_udc_gain_range.start();
    double fe_gain = safe_fe_gain_range.start();
    double excess_gain = total_gain - udc_gain - fe_gain;
    double remaining_gain = excess_gain;
    double fe_gain_for_typical = 0;
    if (_path.trx) {
        const double fe_gain_for_optimum_zbx_output = 
            ZBX_TX_IN_TYP_MAX_POWER - safe_fe_power_range.start();
        fe_gain_for_typical = std::min(safe_fe_gain_range.stop() - fe_gain, utility::fixed_uhd_meta_range_clip(safe_fe_gain_range, std::min(fe_gain_for_optimum_zbx_output, remaining_gain), true));
        remaining_gain -= fe_gain_for_typical;
        fe_gain += fe_gain_for_typical;
    }

    double available_extra_udc_gain = safe_udc_gain_range.stop() - udc_gain;
    double extra_udc_whole_gain = std::min(available_extra_udc_gain, (remaining_gain - std::floor(remaining_gain) >= 1.5 * safe_udc_gain_range.step()) ? std::ceil(remaining_gain) : std::floor(remaining_gain));
    udc_gain += extra_udc_whole_gain;
    remaining_gain        = remaining_gain - extra_udc_whole_gain;
    double extra_whole_fe_gain =
        std::min(safe_fe_gain_range.stop() - fe_gain, (remaining_gain - std::floor(remaining_gain) >= 1.5 * safe_udc_gain_range.step()) ? std::ceil(remaining_gain) : std::floor(remaining_gain));
    fe_gain += extra_whole_fe_gain;
    remaining_gain -= extra_whole_fe_gain;
    double udc_fractional_gain = std::min(safe_udc_gain_range.stop() - udc_gain, remaining_gain >= safe_udc_gain_range.step() / 2.0 ? safe_udc_gain_range.step() : 0.0);
    udc_gain += udc_fractional_gain;
    remaining_gain -= udc_fractional_gain;
    double max_unsafe_udc_gain = (_path.trx ? _udc->get_tx_gain_range(_path.chan) : _udc->get_rx_gain_range(_path.chan)).stop();

    _path.trx ? _radio->set_tx_gain(fe_gain, chan) : _radio->set_rx_gain(fe_gain, chan);
    
    double calculated_fe_power_ref = zero_gain_power_reference + (_path.trx ? fe_gain : (-1.0 * fe_gain) );
    double calculated_total_power_ref = _path.trx ? calculated_fe_power_ref + udc_gain : calculated_fe_power_ref - udc_gain;

    _udc_gain_out = udc_gain;
    _gain_out     = udc_gain + fe_gain;
    _fe_power_ref_out = calculated_fe_power_ref;
    _power_ref_out = calculated_total_power_ref;    

    UHD_LOG_DEBUG(udc::NAME, _path << " zbx_gain_expert::Resolved fe Gain: " << fe_gain << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " zbx_gain_expert::Resolved udc Gain: " << _udc_gain_out << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " zbx_gain_expert::Resolved Total Gain (transitional): " << _gain_out << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " zbx_gain_expert::Resolved fe Power Ref: " << _fe_power_ref_out << " dBm");
    UHD_LOG_DEBUG(udc::NAME, _path << " zbx_gain_expert::Resolved Total Power Ref (transitional): " << _power_ref_out << " dBm");
}

// Resolution of gain:
// For RX:
//   - For best performance, utilize the UDC gain while minimizing the FBX gain.
//   - Once the UDC gain is maximized, adjust the FBX gain as required.
// For TX:
//   - For best performance, limit the FBX output power to TX_IN_TYP_MAX_POWER.
//   - Set additional gain using the UDC once TX_IN_TYP_MAX_POWER is exceeded.
void fbx_gain_expert::resolve(void)
{
    UHD_LOG_DEBUG(udc::NAME, "fbx_gain_expert::resolve");
    UHD_LOG_TRACE(udc::NAME, _path << " fbx_gain_expert::Input Gain: " << _gain_in << " dB");

    const auto udc_gain_range = (_path.trx == RX_DIRECTION) ?
        (_udc->get_rx_gain_range(_path.chan)) :
        (_udc->get_tx_gain_range(_path.chan));

    _udc_gain_out = utility::fixed_uhd_meta_range_clip(udc_gain_range, _gain_in, true);
    _gain_out = _udc_gain_out;
    UHD_LOG_DEBUG(udc::NAME, _path << " fbx_gain_expert::Resolved UDC Gain: " << _udc_gain_out << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " fbx_gain_expert::Resolved Gain: " << _gain_out << " dB");

}

void udc_gain_expert::resolve(void)
{
    UHD_LOG_DEBUG(udc::NAME, "udc_gain_expert::resolve");
    UHD_LOG_TRACE(udc::NAME, _path << " udc_gain_expert::Input UDC Gain: " << _udc_gain_in << " dB");
    UHD_LOG_TRACE(udc::NAME, _path << " udc_gain_expert::Input (transitional) Gain: " << _gain_in << " dB");
    UHD_LOG_TRACE(udc::NAME, _path << " udc_gain_expert::Input (transitional) Power Ref: " << _power_ref_in << " dB");

    double gain    = _udc_gain_in;

    if (_path.trx == RX_DIRECTION)
    {
        if ((_high_power_prot_in == true) && (gain > HIGH_POWER_PROT_EN_RX_MAX))
        {
            gain = HIGH_POWER_PROT_EN_RX_MAX;
            UHD_LOG_WARNING(udc::NAME, "High Power Protection is enabled. RX gain is being limited. See documentation or contact SCT for details.");
        }
        _udc->set_rx_gain(_path.chan, gain);
    }
    else
    {
        if ((_high_power_prot_in == true) && (gain > HIGH_POWER_PROT_EN_TX_MAX))
        {
            gain = HIGH_POWER_PROT_EN_TX_MAX;
            UHD_LOG_WARNING(udc::NAME, "High Power Protection is enabled. TX gain is being limited. See documentation or contact SCT for details.");
        }
        _udc->set_tx_gain(_path.chan, gain);
    }
    double gain_correction = gain - _udc_gain_in;
    _gain_out = _gain_in + gain_correction;
    _power_ref_out = _power_ref_in + gain_correction;
    _udc_gain_out = gain;
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_gain_expert::Resolved Gain: " << _gain_out << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_gain_expert::Resolved UDC Gain: " << _udc_gain_out << " dB");
    UHD_LOG_DEBUG(udc::NAME, _path << " udc_gain_expert::Resolved Power Ref: " << _power_ref_out << " dB");
}

} // namespace udc
