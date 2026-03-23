//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

//#include "sc2470_constants.hpp"
#include "types/path.hpp"
#include "udc/udc_ctrl.hpp"
#include "types/daughterboard.hpp"
#include <uhd/experts/expert_factory.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/rf_control/antenna_iface.hpp>
#include <uhd/rfnoc/rf_control/nameless_gain_mixin.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/version.hpp>
#include <sc2470/sc2470.hpp>
#include <string>
#include <vector>

namespace udc {

class sc2470_impl : public sc2470,
                    public uhd::rfnoc::rf_control::antenna_radio_control_mixin,
                    public uhd::rfnoc::rf_control::nameless_gain_mixin
{
private:

    uhd::rfnoc::radio_control::sptr _radio;
    uhd::experts::expert_container::sptr _expert_container;
    uhd::property_tree::sptr _tree;
    udc_ctrl::sptr _udc;
    daughterboard::type _db_type;

public:
    using sptr = std::shared_ptr<sc2470_impl>;
    sc2470_impl(uhd::rfnoc::radio_control::sptr radio, uhd::property_tree::sptr tree, daughterboard::type db_type);
    ~sc2470_impl() = default;

    std::string get_tx_antenna(const size_t chan) const override
    {
        const auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::TX_DIRECTION) / "antenna" / "value");
        return prop.get();
    }

    std::string get_rx_antenna(const size_t chan) const override
    {
        const auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::RX_DIRECTION) / "antenna" / "value");
        return prop.get();
    }

    void set_tx_antenna(const std::string& ant, const size_t chan) override
    {
        auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::TX_DIRECTION) / "antenna" / "value");
        prop.set(ant);
    }

    void set_rx_antenna(const std::string& ant, const size_t chan) override
    {
        auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::RX_DIRECTION) / "antenna" / "value");
        prop.set(ant);
    }

    double get_tx_frequency(const size_t chan) override
    {
        const auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::TX_DIRECTION) / "rf_freq" / "value");
        return prop.get();
    }

    double get_rx_frequency(const size_t chan) override
    {
        const auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::RX_DIRECTION) / "rf_freq" / "value");
        return prop.get();
    }

    double set_tx_frequency(const double freq, size_t chan) override
    {
        auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::TX_DIRECTION) / "rf_freq" / "value");
        prop.set(freq);
        return prop.get();
    }

    double set_rx_frequency(const double freq, const size_t chan) override
    {
        auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::RX_DIRECTION) / "rf_freq" / "value");
        prop.set(freq);
        return prop.get();
    }

    void set_tx_tune_args(const uhd::device_addr_t& args, const size_t chan) override
    {
        // const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::TX_DIRECTION);
        // _tree->access<double>(udc_fe_path / "if_freq" / "value")
        //     .set(args.get("if_freq"));
    }

    void set_rx_tune_args(const uhd::device_addr_t& args, const size_t chan) override
    {
        
    }

    std::vector<std::string> get_tx_gain_names(const size_t) const override
    {
        return {"all"};
    }

    std::vector<std::string> get_rx_gain_names(const size_t) const override
    {
        return {"all"};
    }

    double get_tx_gain(const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = _get_fe_path(chan, uhd::TX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").get();
    }

    double get_rx_gain(const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = _get_fe_path(chan, uhd::RX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").get();
    }

    double set_tx_gain(const double gain, const std::string& name_, const size_t chan) override
    {
        if(has_tx_power_reference(chan) && get_tx_power_control_mode(chan)) {
            _set_tx_power_control_mode(false, chan);
        }
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = _get_fe_path(chan, uhd::TX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").set(gain).get();
    }

    double set_rx_gain(const double gain, const std::string& name_, const size_t chan) override
    {
        if(has_rx_power_reference(chan) && get_rx_power_control_mode(chan)) {
            _set_rx_power_control_mode(false, chan);
        }
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = _get_fe_path(chan, uhd::RX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").set(gain).get();
    }

    void set_rx_agc(const bool, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_agc is not supported on this radio");
    }

    uhd::meta_range_t get_tx_bandwidth_range(const size_t chan) const override
    {
        return _radio->get_tx_bandwidth_range(chan);
    }

    uhd::meta_range_t get_rx_bandwidth_range(const size_t chan) const override
    {
        return _radio->get_rx_bandwidth_range(chan);
    }

    double get_tx_bandwidth(const size_t chan) override
    {
        return _radio->get_tx_bandwidth(chan);
    }

    double get_rx_bandwidth(const size_t chan) override
    {
        return _radio->get_rx_bandwidth(chan);
    }

    double set_tx_bandwidth(const double bandwidth, const size_t chan) override
    {
        return _radio->set_tx_bandwidth(bandwidth, chan);
    }

    double set_rx_bandwidth(const double bandwidth, const size_t chan) override
    {
        return _radio->set_rx_bandwidth(bandwidth, chan);
    }

    uhd::gain_range_t get_rx_gain_range(const std::string& name, const size_t chan) const override;
    uhd::gain_range_t get_tx_gain_range(const std::string& name, const size_t chan) const override;

    uhd::freq_range_t get_rx_frequency_range(const size_t) const override;
    uhd::freq_range_t get_tx_frequency_range(const size_t) const override;

    // ! The following methods are not implemented in the UHD extension framework...
    // ! UHD calls the 'radio' implementation instead of the 'rf_core' extension overrides.
    // ! --- Start not implemented ---
    std::vector<std::string> get_tx_lo_names(const size_t) const override
    {
        return {"udc"};
    }

    std::vector<std::string> get_rx_lo_names(const size_t) const override
    {
        return {"udc"};
    }

    std::vector<std::string> get_tx_lo_sources(const std::string&, const size_t) const override
    {
        return LO_SOURCES;
    }

    std::vector<std::string> get_rx_lo_sources(const std::string&, const size_t) const override
    {
        return LO_SOURCES;
    }

    uhd::freq_range_t get_tx_lo_freq_range(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_freq_range is not supported");
    }

    uhd::freq_range_t get_rx_lo_freq_range(const std::string&, const size_t) const override
    {
        throw uhd::not_implemented_error("get_rx_lo_freq_range is not supported");
    }

    void set_rx_lo_source(const std::string& src, const std::string&, const size_t chan) override
    {
        auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::RX_DIRECTION) / "lo_source" / "value");
        prop.set(src);
    }

    void set_tx_lo_source(const std::string& src, const std::string&, const size_t chan) override
    {
        auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::TX_DIRECTION) / "lo_source" / "value");
        prop.set(src);
    }

// UHD 4.7.0.0 changed the declaration 
#if (UHD_VERSION > 4070000)
    std::string get_rx_lo_source(const std::string&, const size_t chan) override
#else
    const std::string get_rx_lo_source(const std::string&, const size_t chan) override
#endif
    {
        const auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::RX_DIRECTION) / "lo_source" / "value");
        return prop.get();
    }

// UHD 4.7.0.0 changed the declaration 
#if (UHD_VERSION > 4070000)
    std::string get_tx_lo_source(const std::string&, const size_t chan) override
#else
    const std::string get_tx_lo_source(const std::string&, const size_t chan) override
#endif
    {
        const auto& prop = _tree->access<std::string>(
            _get_udc_fe_path(chan, uhd::TX_DIRECTION) / "lo_source" / "value");
        return prop.get();
    }

    void set_rx_lo_export_enabled(bool, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_lo_export_enabled is not supported on this radio");
    }

    void set_tx_lo_export_enabled(const bool, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_tx_lo_export_enabled is not supported on this radio");
    }

    bool get_rx_lo_export_enabled(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_lo_export_enabled is not supported on this radio");
    }

    bool get_tx_lo_export_enabled(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_export_enabled is not supported on this radio");
    }

    double set_tx_lo_freq(double, const std::string&, const size_t chan) override
    {
        throw uhd::not_implemented_error("set_tx_lo_freq is not supported on this radio");
    }

    double set_rx_lo_freq(double, const std::string&, const size_t chan) override
    {
        throw uhd::not_implemented_error("set_rx_lo_freq is not supported on this radio");
    }

    double get_rx_lo_freq(const std::string&, const size_t chan) override
    {
        const auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::RX_DIRECTION) / "lo_freq" / "value");
        return prop.get();
    }

    double get_tx_lo_freq(const std::string&, const size_t chan) override
    {
        const auto& prop =
            _tree->access<double>(_get_udc_fe_path(chan, uhd::TX_DIRECTION) / "lo_freq" / "value");
        return prop.get();
    }

    bool has_rx_power_reference(const size_t chan) override
    {
        return _radio->has_rx_power_reference(chan);
    }

    bool has_tx_power_reference(const size_t chan) override
    {
        return _radio->has_tx_power_reference(chan);
    }

    void set_rx_power_reference(const double power_dbm, const size_t chan) override
    {
        if(!_radio->has_rx_power_reference()) {
            UHD_LOG_WARNING(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " in this direction does not support power control mode, ignoring set_rx_power_reference..."); 
            return;
        }
        _set_rx_power_control_mode(true, chan);
        const uhd::fs_path power_path = _get_fe_path(chan, uhd::RX_DIRECTION) / "power_ref";
        _tree->access<double>(power_path / "value").set(power_dbm);
    }

    void set_tx_power_reference(const double power_dbm, const size_t chan) override
    {
        if(!_radio->has_tx_power_reference()) {
            UHD_LOG_WARNING(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " in this direction does not support power control mode, ignoring set_tx_power_reference..."); 
            return;
        }
        _set_tx_power_control_mode(true, chan);
        const uhd::fs_path power_path = _get_fe_path(chan, uhd::TX_DIRECTION) / "power_ref";
        _tree->access<double>(power_path / "value").set(power_dbm);
    }

    double get_rx_power_reference(const size_t chan) override
    {
        if(!has_rx_power_reference(chan)) {
            UHD_LOG_ERROR(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " rx power reference is not supported on this device, value will be inaccurate..."); 
        }
        const uhd::fs_path power_path = _get_fe_path(chan, uhd::RX_DIRECTION) / "power_ref";
        return _tree->access<double>(power_path / "value").get();
    }

    double get_tx_power_reference(const size_t chan) override
    {
        if(!has_tx_power_reference(chan)) {
            UHD_LOG_ERROR(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " tx power reference is not supported on this device, value will be inaccurate..."); 
        }
        const uhd::fs_path power_path = _get_fe_path(chan, uhd::TX_DIRECTION) / "power_ref";
        return _tree->access<double>(power_path / "value").get();
    }

    uhd::meta_range_t get_rx_power_range(const size_t chan) override
    {
        if (!has_rx_power_reference(chan)) {
            UHD_LOG_ERROR(udc::NAME,
                to_string(_db_type)
                    << "[" << chan << "]" << " rx power reference is not supported on this device");
        }
        bool hi_power_protection = get_high_power_prot();
        udc::path path = {chan, uhd::direction_t::RX_DIRECTION};
        uhd::meta_range_t fe_power_range = _radio->get_rx_power_range(chan);
        uhd::gain_range_t udc_gain_range = _udc->get_rx_gain_range(chan);
        uhd::gain_range_t safe_fe_power_range = _udc->get_safe_fe_power_range(path, fe_power_range);
        uhd::gain_range_t safe_udc_power_range = _udc->get_safe_udc_power_range(hi_power_protection, safe_fe_power_range, path);
        return safe_udc_power_range;
    }

    uhd::meta_range_t get_tx_power_range(const size_t chan) override
    {
        if (!has_tx_power_reference(chan)) {
            UHD_LOG_ERROR(udc::NAME,
                to_string(_db_type)
                    << "[" << chan << "]" << " tx power reference is not supported on this device");
        }
        bool hi_power_protection = get_high_power_prot();
        udc::path path = {chan, uhd::direction_t::TX_DIRECTION};
        uhd::meta_range_t fe_power_range = _radio->get_tx_power_range(chan);
        uhd::gain_range_t udc_gain_range = _udc->get_tx_gain_range(chan);
        uhd::gain_range_t safe_fe_power_range = _udc->get_safe_fe_power_range(path, fe_power_range);
        uhd::gain_range_t safe_udc_power_range = _udc->get_safe_udc_power_range(hi_power_protection, safe_fe_power_range, path);
        return safe_udc_power_range;
    }

    std::vector<std::string> get_rx_power_ref_keys(const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_power_ref_keys is not supported on this radio");
    }

    std::vector<std::string> get_tx_power_ref_keys(const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_power_ref_keys is not supported on this radio");
    }

    // ! --- End not implemented ---

    std::string get_extension_version(void) override;
    void set_gain_updates(const bool enable, const size_t chan) override;
    bool get_gain_updates(const size_t chan) override;
    uint32_t direct_io(const uint32_t data, const size_t chan) override;
    void set_tx_if(const double if_hz, const size_t chan) override;
    void set_rx_if(const double if_hz, const size_t chan) override;
    void set_high_power_prot(const bool enable) override;
    void set_ref_source(const std::string& src) override;
    double get_tx_if(const size_t chan) override;
    double get_rx_if(const size_t chan) override;
    bool get_tx_bypass(const size_t chan) override;
    bool get_rx_bypass(const size_t chan) override;
    std::vector<std::string> get_ref_sources(void) override;
    std::string get_ref_source(void) override;
    bool get_high_power_prot(void) override;
    bool get_tx_power_control_mode(const size_t chan) override;
    bool get_rx_power_control_mode(const size_t chan) override;
    std::string get_daughterboard_type(void) override;
    double get_fe_rx_power_reference(const size_t chan) override;
    double get_fe_tx_power_reference(const size_t chan) override;
    double get_udc_rx_gain(const size_t chan) override;
    double get_udc_tx_gain(const size_t chan) override;
    double get_fe_rx_gain(const size_t chan) override;
    double get_fe_tx_gain(const size_t chan) override;
    uhd::gain_range_t get_safe_fe_gain_range(const uhd::direction_t dir, const path& path);



    std::string get_name() override
    {
        return udc::NAME;
    }

private:
    uhd::fs_path _get_fe_path(const size_t chan, const uhd::direction_t trx) const
    {
        uhd::fs_path frontend = (trx == uhd::TX_DIRECTION) ? "tx_frontends" : "rx_frontends";
        return frontend / chan;
    }

    uhd::fs_path _get_udc_fe_path(const size_t chan, const uhd::direction_t trx) const
    {
        return uhd::fs_path(udc::NAME) / _get_fe_path(chan, trx);
    }

    uhd::fs_path _get_udc_base_path(const std::string slot) const
    {
        return uhd::fs_path(udc::NAME) / uhd::fs_path(slot);
    }

    void _init_path(const size_t chan, const uhd::direction_t trx);
    void _init_antenna_prop_tree(const path& path);
    void _init_frequency_prop_tree(const path& path);
    void _init_gain_prop_tree(const path& path);
    void _init_external_control_prop_tree(const path& path);
    void _init_reference_prop_tree(const path& path);
    void _set_tx_power_control_mode(const bool enable, const size_t chan);
    void _set_rx_power_control_mode(const bool enable, const size_t chan);
    
};

} // namespace udc
