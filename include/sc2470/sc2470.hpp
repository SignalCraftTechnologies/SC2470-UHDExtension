//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/extension/extension.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/rf_control/core_iface.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <memory>

namespace udc {

/*! Interface for SC2470
 *
 * This interface contains all methods related directly to the SC2470.
 */
class sc2470 : public uhd::extension::extension
{
public:
    using sptr = std::shared_ptr<sc2470>;

    virtual ~sc2470() = default;

    /*!
     * Make an instance of the sc2470.
     * This assumes the UDC is connected to the matching  X410 radio channel and slot.
     * Additionally, the GPIO HDMI cables must be connected between devices.
     *
     * \param fargs Factory args that hold pointers to radio control and motherboard
     * controller
     * \returns Smart pointer to sc2470 class object
     */
    static sptr make(uhd::extension::extension::factory_args fargs);

    /*!
     * Get version string of the extension
     *
     * \returns Version string
    */
    virtual std::string get_extension_version(void) = 0;

    /*!
     * Enables or disables the gain updates on the signal conditioning accessory. If
     * updates are enabled, attenuation/gain settings will be applied to the SC2470 hardware.
     * If updates are disabled the configuration will be written to the SC2470, however, they
     * will not be applied until the gain updates are re-enabled. The attenuator latches are
     * available per channel and therefore can be configured individually.
     *
     * \param enable Enables or disables gain update for the channel specified
     */
    virtual void set_gain_updates(const bool enable, const size_t chan) = 0;

    /*!
     * Get the current status of the gain update latches of the SC2470 hardware.
     *
     * \returns The enable or disable state of the gain updates for the channel specified
     */
    virtual bool get_gain_updates(const size_t chan) = 0;

    /*!
     * Access registers using the Binary Protocol of the SC2470.
     * Not intended for typical use.
     *
     * \param data The register word to send.
     * \param chan The channel to send the data to.
     * \returns The value of the register specified.
     */
    virtual uint32_t direct_io(const uint32_t data, const size_t chan) = 0;

    /*!
     * Overrides default Intermediate Frequency that is transmitted from the x4xx device
     * and the SC2470.
     * \param if_hz The IF override value in Hertz. Precision is 1 mHz.
     * \param chan The channel to configure.
     */
    virtual void set_tx_if(const double if_hz, const size_t chan) = 0;

    /*!
     * Overrides default Intermediate Frequency that is transmitted from the x4xx device
     * and the SC2470.
     * \param chan The channel to configure.
     * \returns The IF override value
     */
    virtual double get_tx_if(const size_t chan) = 0;

    /*!
     * Overrides default Intermediate Frequency that is received by the x4xx device
     * and the SC2470.
     * \param if_hz The IF override value in Hertz. Precision is 1 mHz.
     * \param chan The channel to configure.
     */

    virtual void set_rx_if(const double if_hz, const size_t chan) = 0;

    /*!
     * Returned the intermediate Frequency that has been applied to the x4xx device
     * and the SC2470.
     * \param chan The channel to configure.
     * \returns The IF override value
     */
    virtual double get_rx_if(const size_t chan) = 0;

    /*!
     * Returned the state of the SC2470 bypass path
     * \param chan The channel to query.
     * \returns The Bypass state
     */
    virtual bool get_tx_bypass(const size_t chan) = 0;

    /*!
     * Returned the state of the SC2470 bypass path
     * \param chan The channel to query.
     * \returns The Bypass state
     */
    virtual bool get_rx_bypass(const size_t chan) = 0;

    /*!
     * Enables or disables high power protection feature.
     * Not intended for typical use, be very careful when changing this setting as 
     * hardware damage could occur. 
     * \param enable Enables or disables high power protection. (enabled by default)
    */
    virtual void set_high_power_prot(const bool enable) = 0;

    /*!
     * \returns The high power protection state
    */
    virtual bool get_high_power_prot(void) = 0;


    /*!
     * Queries the power control mode.  Set gain or power levels
     * to automatically set the power control level.  When active, internal gain settings
     * will be automatically configured to achieve the power reference setting.
     * When disabled, gain settings are controlled manually using gain get/set.
     * \param chan The channel to query.  
     * \returns The power control mode state
    */
    virtual bool get_rx_power_control_mode(const size_t chan) = 0;

    /*!
     * Queries the power control mode.  Set gain or power levels
     * to automatically set the power control level.  When active, internal gain settings
     * will be automatically configured to achieve the power reference setting.
     * When disabled, gain settings are controlled manually using gain get/set.
     * \param chan The channel to query.  
     * \returns The power control mode state
    */
    virtual bool get_tx_power_control_mode(const size_t chan) = 0;


    /*!
     * Get a list of available reference sources.  This includes internal and external
     * options with different frequencies
    */
    virtual std::vector<std::string> get_ref_sources(void) = 0;

    /*!
     * Sets the reference source to one of the available options.
     * \param src string matching one of the available reference sources.
    */
    virtual void set_ref_source(const std::string& src) = 0;

    /*!
     * Returns the currently active reference source
     * \returns The reference source
    */
    virtual std::string get_ref_source(void) = 0;

    /*!
     * Returns the detected daughterboard type
     * \returns The daughterboard type
    */
    virtual std::string get_daughterboard_type(void) = 0;

    /*!
     * Queries the front end (fe) rx power reference.  This is the power referenced
     * at the fe rf connector which will give 0dB full scale at the ADC, taking fe gain into account.
     * \returns The fe power reference
    */
    virtual double get_fe_rx_power_reference(const size_t chan) = 0;

    /*!
     * Queries the front end (fe) tx power reference.  This is the power referenced
     * at the fe rf connector given 0db full scale at the DAC, taking fe gain into account.
     * \returns The fe power reference
    */
    virtual double get_fe_tx_power_reference(const size_t chan) = 0;

    /*!
     * Queries the up-down converter (udc) rx gain.  This does not include the front end (fe) gain
     * \returns The udc gain
    */
    virtual double get_udc_rx_gain(const size_t chan) = 0;

    /*!
     * Queries the up-down converter (udc) tx gain.  This does not include the front end (fe) gain
     * \returns The udc gain
    */
    virtual double get_udc_tx_gain(const size_t chan) = 0;

    /*!
     * Queries the front end (fe) rx gain.  This does not include udc gain.
     * \returns The fe gain
    */
    virtual double get_fe_rx_gain(const size_t chan) = 0;

    /*!
     * Queries the front end (fe) tx gain.  This does not include udc gain.
     * \returns The fe power reference
    */
    virtual double get_fe_tx_gain(const size_t chan) = 0;

    /*!
     * Queries whether the channel is able to communicate with the SC2470 through
     * the HDMI cable's SPI connection.
     * \returns Whether the channel is connected
    */
    virtual bool get_is_connected(const size_t chan) = 0;

};

} // namespace udc
