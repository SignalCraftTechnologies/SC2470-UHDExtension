//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "sc2470_impl.hpp"
#include "sc2470_constants.hpp"
#include "sc2470_expert.hpp"
#include "version.hpp"
#include <uhd/experts/expert_container.hpp>
#include <uhd/extension/extension.hpp>
#include <uhd/rfnoc/registry.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/math.hpp>
#include <cassert>

using namespace uhd;

namespace udc {

void printPropTree(uhd::property_tree::sptr tree, std::string path)
{
    if (tree->list(path).size() == 0)
        return;

    for (auto& childPath : tree->list(path))
    {
        UHD_LOG_DEBUG(udc::NAME, "Prop: " << path << childPath);
        printPropTree(tree, (path + childPath + "/"));
    }
}

sc2470::sptr sc2470::make(uhd::extension::extension::factory_args fargs)
{
    auto tree = fargs.radio_ctrl->get_tree();
    daughterboard::type db_type;

    if (tree->exists("dboard/rx_frontends/0/name") && (tree->access<std::string>("dboard/rx_frontends/0/name").get() == "ZBX"))
    {
        db_type = daughterboard::type::zbx;
    }
    else if(tree->exists("dboard/rx_frontends/0/name") && (tree->access<std::string>("dboard/rx_frontends/0/name").get() == "FBX"))
    {
        if (tree->exists("dboard/rx_frontends/1/name") 
            && tree->exists("dboard/rx_frontends/2/name") 
            && tree->exists("dboard/rx_frontends/3/name"))
        {
            db_type = daughterboard::type::fbx_400;
        }
        else
        {
            db_type = daughterboard::type::fbx_1600;
        }        
    }
    else
    {
        throw uhd::runtime_error("Daughterboard for " + std::string(udc::NAME) + " not found.");
    }
    
    return std::make_shared<sc2470_impl>(fargs.radio_ctrl, fargs.radio_ctrl->get_tree(), db_type);
}

sc2470_impl::sc2470_impl(uhd::rfnoc::radio_control::sptr radio, uhd::property_tree::sptr tree, daughterboard::type db_type)
    : nameless_gain_mixin([](uhd::direction_t, size_t) { return "all"; })
    , _radio(radio)
    , _tree(tree)
    , _udc(std::make_shared<udc_ctrl>(radio, db_type))
{
    _db_type = db_type;

    UHD_LOG_INFO(udc::NAME, "SC2470 Extension Version: " << get_extension_version()); 

    for (size_t chan = 0; chan < daughterboard::channel_count(_db_type).value; chan++) {
        _radio->set_tx_antenna("TX/RX0", chan);
        _radio->set_rx_antenna("RX1", chan);
    }

    const std::unordered_map<std::string, std::string> ANTENNA_NAME_COMPAT_MAP = {};

    _rx_antenna = std::make_shared<uhd::rfnoc::rf_control::enumerated_antenna>(
        tree,
        [this](
            size_t chan) { return this->_get_fe_path(chan, RX_DIRECTION) / "antenna" / "value"; },
        ANTENNAS,
        ANTENNA_NAME_COMPAT_MAP);
    _tx_antenna = std::make_shared<uhd::rfnoc::rf_control::enumerated_antenna>(
        tree,
        [this](
            size_t chan) { return this->_get_fe_path(chan, TX_DIRECTION) / "antenna" / "value"; },
        ANTENNAS,
        ANTENNA_NAME_COMPAT_MAP);

    _expert_container = uhd::experts::expert_factory::create_container("sc2470_radio");
    for (size_t chan = 0; chan < daughterboard::channel_count(_db_type).value; chan++) {
        _init_path(chan, TX_DIRECTION);
        _init_path(chan, RX_DIRECTION);
    }
    _expert_container->resolve_all();
    UHD_LOG_DEBUG(udc::NAME, "Initialized Slot [" << _radio->get_slot_name() << "]"); 
}

void sc2470_impl::_init_path(const size_t chan, const uhd::direction_t trx)
{
    const path path = {chan, trx};

    _init_external_control_prop_tree(path);
    _init_reference_prop_tree(path);
    _init_antenna_prop_tree(path);
    _init_frequency_prop_tree(path);
    _init_gain_prop_tree(path);
}

void sc2470_impl::_init_external_control_prop_tree(const path& path)
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());

    if (!_tree->exists(udc_base_path / "high_power_prot" / "value"))
    {
        uhd::experts::expert_factory::add_prop_node<bool>(_expert_container,
            _tree,
            udc_base_path / "high_power_prot" / "value",
            true,
            uhd::experts::AUTO_RESOLVE_ON_WRITE);
    }
}

void sc2470_impl::_init_reference_prop_tree(const path& path)
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    
    if (!_tree->exists(udc_base_path / "reference_source" / "value"))
    {
        uhd::experts::expert_factory::add_prop_node<std::string>(_expert_container,
            _tree,
            udc_base_path / "reference_source" / "value",
            REF_SOURCES[0],
            uhd::experts::AUTO_RESOLVE_ON_WRITE);
        uhd::experts::expert_factory::add_worker_node<udc_reference_expert>(
        _expert_container, _expert_container->node_retriever(), _udc, path, udc_base_path);
    }
    
}

void sc2470_impl::_init_antenna_prop_tree(const path& path)
{
    const auto fe_path     = _get_fe_path(path.chan, path.trx);
    const auto udc_fe_path = _get_udc_fe_path(path.chan, path.trx);

    _tree->create<std::vector<std::string>>(fe_path / "antenna" / "options")
        .set(path.trx == TX_DIRECTION ? get_tx_antennas(path.chan) : get_rx_antennas(path.chan))
        .add_coerced_subscriber([](const std::vector<std::string>&) {
            throw uhd::runtime_error("Attempting to update antenna options!");
        });
    _tree->create<std::string>(fe_path / "antenna" / "value")
        .set(ANTENNAS[0])
        .set_coercer([](const std::string& antenna) {
            for (const auto& entry : ANTENNAS)
                if (antenna == entry)
                    return antenna;
            throw uhd::value_error("Set antenna to invalid value!");
            return antenna;
        });

    uhd::experts::expert_factory::add_dual_prop_node<std::string>(_expert_container,
        _tree,
        udc_fe_path / "antenna" / "value",
        ANTENNAS[0],
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_worker_node<udc_antenna_expert>(
        _expert_container, _expert_container->node_retriever(), _udc, path, udc_fe_path);
}



void sc2470_impl::_init_frequency_prop_tree(const path& path)
{
    const auto fe_path     = _get_fe_path(path.chan, path.trx);
    const auto udc_fe_path = _get_udc_fe_path(path.chan, path.trx);
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        udc_fe_path / "rf_freq" / "value",
        RF_FREQ_DEFAULT,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        udc_fe_path / "if_freq_override" / "value",
        0.0,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_prop_node<double>(_expert_container,
        _tree,
        udc_fe_path / "if_freq" / "value",
        IF_FREQ_DEFAULT,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_prop_node<bool>(_expert_container,
        _tree,
        udc_fe_path / "udc_bypass" / "value",
        false,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);
    
    uhd::experts::expert_factory::add_prop_node<double>(_expert_container,
        _tree,
        udc_fe_path / "lo_freq" / "value",
        LO_FREQ_DEFAULT,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    bool prefer_nyquist = _db_type != udc::daughterboard::type::zbx;
    uhd::freq_range_t if_bypass_range = _db_type == udc::daughterboard::type::zbx ? IF_FREQ_X410_BYPASS_RANGE : IF_FREQ_X440_BYPASS_RANGE;
    uhd::freq_range_t if_full_range  = _db_type == udc::daughterboard::type::zbx ? IF_FREQ_X410_RANGE : IF_FREQ_X440_RANGE;
    uhd::experts::expert_factory::add_worker_node<udc_freq_expert>(
            _expert_container, _expert_container->node_retriever(), _radio, _udc, path, udc_fe_path, udc_base_path, prefer_nyquist, if_bypass_range, if_full_range);

}

void sc2470_impl::_init_gain_prop_tree(const path& path)
{
    const auto fe_path     = _get_fe_path(path.chan, path.trx);
    const auto udc_fe_path = _get_udc_fe_path(path.chan, path.trx);
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        udc_fe_path / "gains" / "all" / "value",
        -100,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        fe_path / "gains" / "all" / "value",
        -100,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_prop_node<double>(_expert_container,
        _tree,
        fe_path / "transitional_gain" / "value",
        -100,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_prop_node<double>(_expert_container,
        _tree,
        fe_path / "transitional_power_ref" / "value",
        -100,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);
    
    uhd::experts::expert_factory::add_prop_node<bool>(_expert_container,
            _tree,
            fe_path / "power_control" / "value",
            path.trx == TX_DIRECTION ? has_tx_power_reference(path.chan) : has_rx_power_reference(path.chan),
            uhd::experts::AUTO_RESOLVE_ON_WRITE);
    
    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
            _tree,
            fe_path / "power_ref" / "value",
            path.trx == TX_DIRECTION ? TX_DEFAULT_POWER_REF : RX_DEFAULT_POWER_REF,
            uhd::experts::AUTO_RESOLVE_ON_WRITE);
    
    uhd::experts::expert_factory::add_prop_node<double>(_expert_container,
            _tree,
            fe_path / "fe_power_ref" / "value",
            path.trx == TX_DIRECTION ? TX_DEFAULT_POWER_REF : RX_DEFAULT_POWER_REF,
            uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_worker_node<udc_gain_expert>(_expert_container, 
        _expert_container->node_retriever(), 
        _udc, 
        path, 
        udc_fe_path, 
        udc_base_path,
        fe_path);

    
    if (_db_type == udc::daughterboard::type::zbx)
    {        
        uhd::experts::expert_factory::add_worker_node<zbx_gain_expert>(_expert_container,
            _expert_container->node_retriever(),
            _radio,
            _udc,
            path,
            udc_base_path,
            udc_fe_path,
            fe_path);
    }
    else 
    {
        uhd::experts::expert_factory::add_worker_node<fbx_gain_expert>(_expert_container,
            _expert_container->node_retriever(),
            _radio,
            _udc,
            path,
            udc_fe_path,
            fe_path);
    }
}

uhd::gain_range_t sc2470_impl::get_rx_gain_range(const std::string& name, const size_t chan) const
{;
    path path = {chan, uhd::direction_t::RX_DIRECTION};
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    bool hiPowerProt = _tree->access<bool>(udc_base_path / "high_power_prot" / "value").get();
    uhd::gain_range_t fe_gain_range =  _radio->get_rx_gain_range(chan);
    if(_radio->has_rx_power_reference(chan)) {
        uhd::gain_range_t fe_power_range = _radio->get_rx_power_range(chan);
        uhd::gain_range_t safe_fe_gain_range = _udc->get_safe_fe_gain_range(path, fe_gain_range, fe_power_range);
        return _udc->get_safe_overall_gain_range(hiPowerProt, safe_fe_gain_range, path);
    } else {
        return _udc->get_safe_overall_gain_range(hiPowerProt, fe_gain_range, path);
    }
}

uhd::gain_range_t sc2470_impl::get_tx_gain_range(const std::string& name, const size_t chan) const
{
    path path = {chan, uhd::direction_t::TX_DIRECTION};
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    bool hiPowerProt = _tree->access<bool>(udc_base_path / "high_power_prot" / "value").get();
    uhd::gain_range_t fe_gain_range =  _radio->get_tx_gain_range(chan);
    if(_radio->has_tx_power_reference(chan)) {
        uhd::gain_range_t fe_power_range = _radio->get_tx_power_range(chan);
        uhd::gain_range_t safe_fe_gain_range = _udc->get_safe_fe_gain_range(path, fe_gain_range, fe_power_range);
        return _udc->get_safe_overall_gain_range(hiPowerProt, safe_fe_gain_range, path);
    } else {
        return _udc->get_safe_overall_gain_range(hiPowerProt, fe_gain_range, path);
    }
}

freq_range_t sc2470_impl::get_rx_frequency_range(const size_t) const
{
    return RF_FREQ_RANGE;
}

freq_range_t sc2470_impl::get_tx_frequency_range(const size_t) const
{
    return RF_FREQ_RANGE;
}

std::string sc2470_impl::get_extension_version(void) 
{
    return udc::get_version();
};

void sc2470_impl::set_gain_updates(const bool enable, const size_t chan)
{
    return _udc->set_attn_latch(chan, enable);
}

bool sc2470_impl::get_gain_updates(const size_t chan)
{
    return _udc->get_attn_latch(chan);
}

uint32_t sc2470_impl::direct_io(const uint32_t data, const size_t chan)
{
    return _udc->direct_io(chan, data);
}

void sc2470_impl::set_tx_if(const double if_hz, const size_t chan)
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::TX_DIRECTION);
    _tree->access<double>(udc_fe_path / "if_freq_override" / "value").set(if_hz);
}

void sc2470_impl::set_rx_if(const double if_hz, const size_t chan)
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::RX_DIRECTION);
    _tree->access<double>(udc_fe_path / "if_freq_override" / "value").set(if_hz);
}

void sc2470_impl::set_high_power_prot(const bool enable)
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    _tree->access<bool>(udc_base_path / "high_power_prot" / "value").set(enable);
}

void sc2470_impl::_set_rx_power_control_mode(const bool enable, const size_t chan)
{
    if(!_radio->has_rx_power_reference()) {
        UHD_LOG_WARNING(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " in this direction does not support power control mode, ignoring set_rx_power_control_mode..."); 
        return;
    }
    const uhd::fs_path fe_path = _get_fe_path(chan, uhd::direction_t::RX_DIRECTION);
    _tree->access<bool>(fe_path / "power_control" / "value").set(enable);
}

void sc2470_impl::_set_tx_power_control_mode(const bool enable, const size_t chan)
{
    if(!_radio->has_rx_power_reference()) {
        UHD_LOG_WARNING(udc::NAME, to_string(_db_type) << "[" << chan << "]" << " in this direction does not support power control mode, ignoring set_tx_power_control_mode..."); 
        return;
    }
    const uhd::fs_path fe_path = _get_fe_path(chan, uhd::direction_t::TX_DIRECTION);
    _tree->access<bool>(fe_path / "power_control" / "value").set(enable);
}

void sc2470_impl::set_ref_source(const std::string& src)
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    _tree->access<std::string>(udc_base_path / "reference_source" / "value").set(src);
}

double sc2470_impl::get_tx_if(const size_t chan) 
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::TX_DIRECTION);
    return _tree->access<double>(udc_fe_path / "if_freq" / "value").get();
}

double sc2470_impl::get_rx_if(const size_t chan) 
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::RX_DIRECTION);
    return _tree->access<double>(udc_fe_path / "if_freq" / "value").get();
}

bool sc2470_impl::get_tx_bypass(const size_t chan)
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::TX_DIRECTION);
    return _tree->access<bool>(udc_fe_path / "udc_bypass" / "value").get();
}

bool sc2470_impl::get_rx_bypass(const size_t chan)
{
    const uhd::fs_path udc_fe_path = _get_udc_fe_path(chan, uhd::direction_t::RX_DIRECTION);
    return _tree->access<bool>(udc_fe_path / "udc_bypass" / "value").get();
}


bool sc2470_impl::get_high_power_prot(void) 
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    return _tree->access<bool>(udc_base_path / "high_power_prot" / "value").get();
}

bool sc2470_impl::get_rx_power_control_mode(const size_t chan) 
{
    if(!_radio->has_rx_power_reference()) {
       return false;
    }
    const uhd::fs_path fe_path = _get_fe_path(chan, uhd::direction_t::RX_DIRECTION);
    return _tree->access<bool>(fe_path / "power_control" / "value").get();
}

bool sc2470_impl::get_tx_power_control_mode(const size_t chan) 
{
    if(!_radio->has_tx_power_reference()) {
       return false;
    }
    const uhd::fs_path fe_path = _get_fe_path(chan, uhd::direction_t::TX_DIRECTION);
    return _tree->access<bool>(fe_path / "power_control" / "value").get();
}

std::vector<std::string> sc2470_impl::get_ref_sources(void) 
{
    return udc::REF_SOURCES;
}

std::string sc2470_impl::get_ref_source(void) 
{
    const auto udc_base_path = _get_udc_base_path(_radio->get_slot_name());
    return _tree->access<std::string>(udc_base_path / "reference_source" / "value").get();
}

std::string sc2470_impl::get_daughterboard_type(void)
{
    return daughterboard::to_string(_db_type);
}

double sc2470_impl::get_fe_rx_power_reference(const size_t chan)
{
    if (!has_rx_power_reference(chan)) {
        UHD_LOG_ERROR(udc::NAME,
            to_string(_db_type) << "[" << chan << "]"
                                << " rx power reference is not supported on this device, value "
                                   "will be inaccurate...");
    }
    const uhd::fs_path power_path = _get_fe_path(chan, uhd::RX_DIRECTION) / "fe_power_ref";
    return _tree->access<double>(power_path / "value").get();
}

double sc2470_impl::get_fe_tx_power_reference(const size_t chan)
{
    if (!has_tx_power_reference(chan)) {
        UHD_LOG_ERROR(udc::NAME,
            to_string(_db_type) << "[" << chan << "]"
                                << " tx power reference is not supported on this device, value "
                                   "will be inaccurate...");
    }
    const uhd::fs_path power_path = _get_fe_path(chan, uhd::TX_DIRECTION) / "fe_power_ref";
    return _tree->access<double>(power_path / "value").get();
}

double sc2470_impl::get_udc_rx_gain(const size_t chan)
{
    const std::string name = "all";
    const uhd::fs_path gains_path = _get_udc_fe_path(chan, uhd::RX_DIRECTION) / "gains";
    return _tree->access<double>(gains_path / name / "value").get();
}

double sc2470_impl::get_udc_tx_gain(const size_t chan)
{
    const std::string name = "all";
    const uhd::fs_path gains_path = _get_udc_fe_path(chan, uhd::TX_DIRECTION) / "gains";
     return _tree->access<double>(gains_path / name / "value").get();
}

double sc2470_impl::get_fe_rx_gain(const size_t chan)
{
    double overallGain = get_rx_gain("", chan);
    double udcGain = get_udc_rx_gain(chan);
    return overallGain - udcGain;
}

double sc2470_impl::get_fe_tx_gain(const size_t chan)
{
    double overallGain = get_tx_gain("", chan);
    double udcGain = get_udc_tx_gain(chan);
    return overallGain - udcGain;
}

bool sc2470_impl::get_is_connected(const size_t chan)
{
    return _udc->get_is_connected(chan);
}



} // namespace udc

/*!
 * Register the SC2470 extension in the extension registry of UDC
 */
UHD_REGISTER_EXTENSION(sc2470, udc::sc2470);
