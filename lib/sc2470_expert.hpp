//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include "udc/udc_ctrl.hpp"
#include <uhd/experts/expert_nodes.hpp>

namespace udc {

// This expert is responsible for programming the antenna
class udc_antenna_expert : public uhd::experts::worker_node_t
{
public:
    udc_antenna_expert(const uhd::experts::node_retriever_t& db,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_fe_path)
        : uhd::experts::worker_node_t(udc_fe_path / "udc_antenna_expert")
        , _udc(udc)
        , _path(path)
        , _antenna_in(db, udc_fe_path / "antenna" / "value" / "desired")
        , _antenna_out(db, udc_fe_path / "antenna" / "value" / "coerced")
    {
        bind_accessor(_antenna_in);
        bind_accessor(_antenna_out);
    }

private:
    void resolve(void) override;

    udc_ctrl::sptr _udc;
    const path _path;

    // Input
    uhd::experts::data_reader_t<std::string> _antenna_in;
    // Output
    uhd::experts::data_writer_t<std::string> _antenna_out;
};

class udc_reference_expert : public uhd::experts::worker_node_t
{
public:
    udc_reference_expert(const uhd::experts::node_retriever_t& db,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_base_path)
        : uhd::experts::worker_node_t(udc_base_path / "udc_reference_expert")
        , _udc(udc)
        , _path(path)
        , _reference_source_in(db, udc_base_path / "reference_source" / "value")
    {
        bind_accessor(_reference_source_in);
    }

private:
    void resolve(void) override;

    udc_ctrl::sptr _udc;
    const path _path;

    // Input
    uhd::experts::data_reader_t<std::string> _reference_source_in;
};

// This expert is responsible for frequency selection.
class udc_freq_expert : public uhd::experts::worker_node_t
{
public:
    udc_freq_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_fe_path,
        const uhd::fs_path udc_base_path, const bool prefer_nyquist, const uhd::freq_range_t if_bypass_range, const uhd::freq_range_t if_full_range)
        : uhd::experts::worker_node_t(udc_fe_path / "udc_freq_expert")
        , _radio(radio)
        , _udc(udc)
        , _path(path)
        , _prefer_nyquist(prefer_nyquist)
        , _if_bypass_range(if_bypass_range)
        , _if_full_range(if_full_range)
        , _rf_frequency_in(db, udc_fe_path / "rf_freq" / "value" / "desired")
        , _if_frequency_override_in(db, udc_fe_path / "if_freq_override" / "value" / "desired")
        , _high_power_prot_in(db, udc_base_path / "high_power_prot" / "value")
        , _rf_frequency_out(db, udc_fe_path / "rf_freq" / "value" / "coerced")
        , _if_frequency_out(db, udc_fe_path / "if_freq" / "value")
        , _lo_frequency_out(db, udc_fe_path / "lo_freq" / "value")
        , _if_frequency_override_out(db, udc_fe_path / "if_freq_override" / "value" / "coerced")
        , _udc_bypass_out(db, udc_fe_path / "udc_bypass" / "value")
    {
        bind_accessor(_rf_frequency_in);        
        bind_accessor(_if_frequency_override_in);
        bind_accessor(_high_power_prot_in);
        bind_accessor(_rf_frequency_out);
        bind_accessor(_if_frequency_out);
        bind_accessor(_lo_frequency_out);
        bind_accessor(_if_frequency_override_out);
        bind_accessor(_udc_bypass_out);
    }

private:
    void resolve(void) override;
    bool calc_bypass(bool high_power_prot, double rf_freq);
    double calc_nyquist(double rf_freq);
    double calc_if(double rf_freq, double if_freq_preferred);
    uhd::freq_range_t calc_non_bypass_if_range(double rf_freq);

    uhd::rfnoc::radio_control::sptr _radio;
    udc_ctrl::sptr _udc;
    const path _path;
    const bool _prefer_nyquist;
    const uhd::freq_range_t _if_bypass_range;
    const uhd::freq_range_t _if_full_range;


    // Inputs
    uhd::experts::data_reader_t<double> _rf_frequency_in;
    uhd::experts::data_reader_t<double> _if_frequency_override_in;
    uhd::experts::data_reader_t<bool> _high_power_prot_in;
    // Outputs
    uhd::experts::data_writer_t<double> _rf_frequency_out;
    uhd::experts::data_writer_t<double> _if_frequency_out;
    uhd::experts::data_writer_t<double> _lo_frequency_out;
    uhd::experts::data_writer_t<double> _if_frequency_override_out;
    uhd::experts::data_writer_t<bool> _udc_bypass_out;
};

// This expert is responsible for distributing the gain between the x410 device and UDC
class zbx_gain_expert : public uhd::experts::worker_node_t
{
public:
    zbx_gain_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_base_path,
        const uhd::fs_path udc_fe_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(udc_fe_path / "zbx_gain_expert")
        , _radio(radio)
        , _udc(udc)
        , _path(path)
        , _power_control_mode(db, fe_path / "power_control" / "value")
        , _power_ref_in(db, fe_path / "power_ref" / "value" / "desired")
        , _gain_in(db, fe_path / "gains" / "all" / "value" / "desired")
        , _rf_frequency_in(db, udc_fe_path / "rf_freq" / "value" / "coerced")
        , _lo_frequency_in(db, udc_fe_path / "lo_freq" / "value")
        , _high_power_prot_in(db, udc_base_path / "high_power_prot" / "value")
        , _gain_out(db, fe_path / "transitional_gain" / "value")
        , _udc_gain_out(db, udc_fe_path / "gains" / "all" / "value" / "desired")
        , _power_ref_out(db, fe_path / "transitional_power_ref" / "value")
        , _fe_power_ref_out(db, fe_path / "fe_power_ref" / "value")

    {
        bind_accessor(_power_control_mode);
        bind_accessor(_power_ref_in);
        bind_accessor(_gain_in);
        bind_accessor(_rf_frequency_in);
        bind_accessor(_lo_frequency_in);
        bind_accessor(_high_power_prot_in);
        bind_accessor(_gain_out);
        bind_accessor(_udc_gain_out);
        bind_accessor(_power_ref_out);
        bind_accessor(_fe_power_ref_out);

    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    udc_ctrl::sptr _udc;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<bool> _power_control_mode;
    uhd::experts::data_reader_t<double> _power_ref_in;
    uhd::experts::data_reader_t<double> _gain_in;
    uhd::experts::data_reader_t<double> _rf_frequency_in;
    uhd::experts::data_reader_t<double> _lo_frequency_in;
    uhd::experts::data_reader_t<bool> _high_power_prot_in;
    // Outputs
    uhd::experts::data_writer_t<double> _gain_out;
    uhd::experts::data_writer_t<double> _udc_gain_out;
    uhd::experts::data_writer_t<double> _power_ref_out;
    uhd::experts::data_writer_t<double> _fe_power_ref_out;
};

// This expert is responsible for setting the power reference with respect to the UDC antenna ports
class zbx_power_ref_expert : public uhd::experts::worker_node_t
{
public:
    zbx_power_ref_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_fe_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(udc_fe_path / "zbx_power_ref_expert")
        , _radio(radio)
        , _udc(udc)
        , _path(path)
        , _power_control_mode(db, fe_path / "power_control" / "value")
        , _power_ref_in(db, fe_path / "power_ref" / "value" / "desired")
        , _rf_frequency_in(db, udc_fe_path / "rf_freq" / "value" / "coerced")
        , _lo_frequency_in(db, udc_fe_path / "lo_freq" / "value")
        , _power_ref_out(db, fe_path / "power_ref" / "value" / "coerced")
        , _udc_gain_out(db, udc_fe_path / "gains" / "all" / "value" / "desired")
    {
        bind_accessor(_power_control_mode);
        bind_accessor(_power_ref_in);
        bind_accessor(_rf_frequency_in);
        bind_accessor(_lo_frequency_in);
        bind_accessor(_power_ref_out);
        bind_accessor(_udc_gain_out);
    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    udc_ctrl::sptr _udc;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<bool> _power_control_mode;
    uhd::experts::data_reader_t<double> _power_ref_in;
    uhd::experts::data_reader_t<double> _rf_frequency_in;
    uhd::experts::data_reader_t<double> _lo_frequency_in;
    // Outputs
    uhd::experts::data_writer_t<double> _power_ref_out;
    uhd::experts::data_writer_t<double> _udc_gain_out;
};

// This expert is responsible for distributing the gain between the x440 device and UDC
class fbx_gain_expert : public uhd::experts::worker_node_t
{
public:
    fbx_gain_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_fe_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(udc_fe_path / "fbx_gain_expert")
        , _radio(radio)
        , _udc(udc)
        , _path(path)
        , _gain_in(db, fe_path / "gains" / "all" / "value" / "desired")
        , _rf_frequency_in(db, udc_fe_path / "rf_freq" / "value" / "coerced")
        , _lo_frequency_in(db, udc_fe_path / "lo_freq" / "value")
        , _gain_out(db, fe_path / "transitional_gain" / "value")
        , _udc_gain_out(db, udc_fe_path / "gains" / "all" / "value" / "desired")
    {
        bind_accessor(_gain_in);
        bind_accessor(_rf_frequency_in);
        bind_accessor(_lo_frequency_in);
        bind_accessor(_gain_out);
        bind_accessor(_udc_gain_out);
    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    udc_ctrl::sptr _udc;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<double> _gain_in;
    uhd::experts::data_reader_t<double> _rf_frequency_in;
    uhd::experts::data_reader_t<double> _lo_frequency_in;
    // Outputs
    uhd::experts::data_writer_t<double> _gain_out;
    uhd::experts::data_writer_t<double> _udc_gain_out;
};

// This expert is responsible for controlling the udc gain settings
class udc_gain_expert : public uhd::experts::worker_node_t
{
public:
    udc_gain_expert(const uhd::experts::node_retriever_t& db,
        udc_ctrl::sptr udc,
        const path path,
        const uhd::fs_path udc_fe_path, 
        const uhd::fs_path udc_base_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(udc_fe_path / "udc_gain_expert")
        , _udc(udc)
        , _path(path)
        , _udc_gain_in(db, udc_fe_path / "gains" / "all" / "value" / "desired")
        , _gain_in(db, fe_path / "transitional_gain" / "value")
        , _high_power_prot_in(db, udc_base_path / "high_power_prot" / "value")
        , _rf_frequency_in(db, udc_fe_path / "rf_freq" / "value" / "coerced")
        , _lo_frequency_in(db, udc_fe_path / "lo_freq" / "value")
        , _power_ref_in(db, fe_path / "transitional_power_ref" / "value")
        , _udc_gain_out(db, udc_fe_path / "gains" / "all" / "value" / "coerced")
        , _gain_out(db, fe_path / "gains" / "all" / "value" / "coerced")
        , _power_ref_out(db, fe_path / "power_ref" / "value" / "coerced")
    {
        bind_accessor(_udc_gain_in);
        bind_accessor(_gain_in);
        bind_accessor(_high_power_prot_in);
        bind_accessor(_rf_frequency_in);
        bind_accessor(_lo_frequency_in);
        bind_accessor(_power_ref_in);
        bind_accessor(_udc_gain_out);
        bind_accessor(_gain_out);
        bind_accessor(_power_ref_out);

    }

private:
    void resolve(void) override;

    udc_ctrl::sptr _udc;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<double> _udc_gain_in;
    uhd::experts::data_reader_t<double> _gain_in;
    uhd::experts::data_reader_t<bool> _high_power_prot_in;
    uhd::experts::data_reader_t<double> _rf_frequency_in;
    uhd::experts::data_reader_t<double> _lo_frequency_in;
    uhd::experts::data_reader_t<double> _power_ref_in;
    // Outputs
    uhd::experts::data_writer_t<double> _udc_gain_out;
    uhd::experts::data_writer_t<double> _gain_out;
    uhd::experts::data_writer_t<double> _power_ref_out;

};

} // namespace udc
