//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "udc_phy.hpp"
#include "utility/version.hpp"
#include <uhd/features/spi_getter_iface.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/utils/log.hpp>

#include <vector>

namespace {

// GPIO Pin-outs
// https://uhd.readthedocs.io/en/latest/page_x400_gpio_api.html
struct GPIO
{
    static constexpr uint32_t PCLK            = 0; // HDMI Pin 1, Data[0]
    static constexpr uint32_t PCLK_MASK       = 1 << PCLK;
    static constexpr uint32_t RESERVED_1      = 1; // HDMI Pin 3, Data[1]
    static constexpr uint32_t RESERVED_1_MASK = 1 << RESERVED_1;
    static constexpr uint32_t SDI             = 2; // HDMI Pin 4, Data[2]
    static constexpr uint32_t SDI_MASK        = 1 << SDI;
    static constexpr uint32_t SDO             = 3; // HDMI Pin 6, Data[3]
    static constexpr uint32_t SDO_MASK        = 1 << SDO;
    static constexpr uint32_t CS_CH0          = 4; // HDMI Pin 7, Data[4]
    static constexpr uint32_t CS_CH0_MASK     = 1 << CS_CH0;
    static constexpr uint32_t CS_CH1          = 5; // HDMI Pin 9, Data[5]
    static constexpr uint32_t CS_CH1_MASK     = 1 << CS_CH1;
    static constexpr uint32_t ATR_CH0         = 6; // HDMI Pin 10, Data[6]
    static constexpr uint32_t ATR_CH0_MASK    = 1 << ATR_CH0;
    static constexpr uint32_t ATR_CH1         = 7; // HDMI Pin 12, Data[7]
    static constexpr uint32_t ATR_CH1_MASK    = 1 << ATR_CH1;
    static constexpr uint32_t LATCH_CH0       = 8; // HDMI Pin 13, Data[8]
    static constexpr uint32_t LATCH_CH0_MASK  = 1 << LATCH_CH0;
    static constexpr uint32_t LATCH_CH1       = 9; // HDMI Pin 15, Data[9]
    static constexpr uint32_t LATCH_CH1_MASK  = 1 << LATCH_CH1;
    static constexpr uint32_t HW_PRESENT      = 10; // HDMI Pin 16, Data[10]
    static constexpr uint32_t HW_PRESENT_MASK = 1 << HW_PRESENT;
    static constexpr uint32_t RESET           = 11; // HDMI Pin 19, Data[11]
    static constexpr uint32_t RESET_MASK      = 1 << RESET;

    static constexpr uint32_t OUTPUT_MASK = PCLK_MASK | SDO_MASK | CS_CH0_MASK | CS_CH1_MASK | ATR_CH0_MASK
                                            | ATR_CH1_MASK | LATCH_CH0_MASK | LATCH_CH1_MASK 
                                            | HW_PRESENT_MASK | RESET_MASK;
};

constexpr size_t SPI_INTERFACE_TOTAL = 2;
constexpr size_t CH0_SPI             = 0;
constexpr size_t CH1_SPI             = 1;

} // namespace

namespace udc {
udc_phy::udc_phy(uhd::rfnoc::radio_control::sptr radio, daughterboard::type db_type) : _radio(radio)
{
    _last_spi_transaction = std::chrono::high_resolution_clock::now();
    _db_type = db_type;
    const size_t shift = get_gpio_shift();

    // Assumes each slot is connected up to a maximum of 2x UDCs
    const auto slot = radio->get_slot_name();
    UHD_ASSERT_THROW(slot == "A" || slot == "B");
    const auto gpio_bank  = (slot == "A") ? "GPIO0" : "GPIO1";
    const auto ch0_source = (slot == "A") ? "DB0_RF0" : "DB1_RF0";
    const auto ch1_source = (slot == "A") ? "DB0_RF1" : "DB1_RF1";
    const auto spi_source = (slot == "A") ? "DB0_SPI" : "DB1_SPI";

    // Set the SPI/GPIO assignments
    std::vector<std::string> sources(12);
    sources[GPIO::PCLK]       = spi_source;
    sources[GPIO::RESERVED_1] = spi_source;
    sources[GPIO::SDI]        = spi_source;
    sources[GPIO::SDO]        = spi_source;
    sources[GPIO::CS_CH0]     = spi_source;
    sources[GPIO::CS_CH1]     = spi_source;
    sources[GPIO::HW_PRESENT] = spi_source;
    sources[GPIO::ATR_CH0]    = ch0_source;
    sources[GPIO::ATR_CH1]    = ch1_source;
    sources[GPIO::LATCH_CH0]  = ch0_source;     // We control this manually, channel doesn't matter
    sources[GPIO::LATCH_CH1]  = ch0_source;     // We control this manually, channel doesn't matter
    sources[GPIO::RESET]      = ch0_source;
    radio->get_mb_controller()->set_gpio_src(gpio_bank, sources);

    // Set the ATR lines
    radio->set_gpio_attr("GPIO", "CTRL", (GPIO::ATR_CH0_MASK | GPIO::ATR_CH1_MASK) << shift);
    radio->set_gpio_attr("GPIO", "ATR_0X", 0);
    radio->set_gpio_attr("GPIO", "ATR_RX", 0);
    radio->set_gpio_attr("GPIO", "ATR_TX", (GPIO::ATR_CH0_MASK | GPIO::ATR_CH1_MASK) << shift);
    radio->set_gpio_attr("GPIO", "ATR_XX", (GPIO::ATR_CH0_MASK | GPIO::ATR_CH1_MASK) << shift);

    // Set the data direction register
    const uint32_t previous_ddr = radio->get_gpio_attr("GPIO", "DDR");
    const uint32_t new_ddr      = (previous_ddr & ~(0xFFF << shift)) | (GPIO::OUTPUT_MASK << shift);
    radio->set_gpio_attr("GPIO", "DDR", new_ddr);

    // Set the peripheral configurations
    UHD_ASSERT_THROW(radio->has_feature<uhd::features::spi_getter_iface>());
    std::vector<uhd::features::spi_periph_config_t> periph_configs(SPI_INTERFACE_TOTAL);
    periph_configs[CH0_SPI] = {
        static_cast<uint8_t>(GPIO::CS_CH0 + shift),
        static_cast<uint8_t>(GPIO::SDI + shift),
        static_cast<uint8_t>(GPIO::SDO + shift),
        static_cast<uint8_t>(GPIO::PCLK + shift),
    };
    periph_configs[CH1_SPI] = {
        static_cast<uint8_t>(GPIO::CS_CH1 + shift),
        static_cast<uint8_t>(GPIO::SDI + shift),
        static_cast<uint8_t>(GPIO::SDO + shift),
        static_cast<uint8_t>(GPIO::PCLK + shift),
    };
    set_hw_present(true); // Required for UDC to know which port HDMI cable is hooked up to.
    _spi     = radio->get_feature<uhd::features::spi_getter_iface>().get_spi_ref(periph_configs);
    _divider = clock_to_divider(radio->get_rate(), 8.192e6);

    // Clear GPIO states
    radio->set_gpio_attr("GPIO", "OUT", 0);
    set_io(GPIO::CS_CH0_MASK, GPIO::CS_CH0_MASK);
    set_io(GPIO::CS_CH1_MASK, GPIO::CS_CH1_MASK);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    initialize_channels();
}

bool udc_phy::is_connected(const size_t chan) const
{
    return _channels[chan].is_connected;
}

uint32_t udc_phy::spi_send(const size_t chan, const proto::command command, const std::string caller = "")
{
    if (!_channels[chan].is_connected)
        return 0;

    // Send
    while (true)
    {
        const uint32_t response = spi_transaction(chan, command);
        if (response != udc::proto::command_status::FW_PROCESSING)
            break;
    }
    
    // Status
    uint32_t retryMax = 5;
    uint32_t retryCounter = 0;
    while (true)
    {
        const uint32_t response = spi_transaction(chan, proto::command(proto::command::id::NOP));
        udc::proto::command_status status = udc::proto::command_status(command, response);
        if (status.is_cmd_retry())
        {
            spi_transaction(chan, command);
        }
        else if (status.is_valid_response())
        {
            if (!status.is_cmd_pending())
            {
                if (status.is_cmd_error())
                {
                    UHD_LOG_WARNING(udc::NAME, "[" << udc::NAME << "] SPI Transaction is command error");
                    std::stringstream msg;
                    msg << "[" << udc::NAME << "] " << caller << ": SPI command failed!";
                    throw uhd::runtime_error(msg.str());
                }
                if (command.is_read())
                    return response;
                break;
            }
        }
        else
        {
            if(retryCounter < retryMax) {
                UHD_LOG_WARNING(udc::NAME, "[" << udc::NAME << "] SPI communication error, retrying!");
                retryCounter += 1;
            } else {
                std::stringstream msg;
                msg << "[" << udc::NAME << "] " << caller << ": SPI communication to sc2470 returned an error - too many retries!";
                UHD_LOG_ERROR(udc::NAME, msg.str());
                throw uhd::runtime_error(msg.str());
            }
        }
    }
    return 0;
}

void udc_phy::set_attn_latch(const size_t chan, const bool enable)
{
    const uint32_t mask = (chan == 0 ? GPIO::LATCH_CH0_MASK : GPIO::LATCH_CH1_MASK);
    set_io(mask, enable ? mask : 0);
}

bool udc_phy::get_attn_latch(const size_t chan)
{
    const uint32_t mask = (chan == 0 ? GPIO::LATCH_CH0_MASK : GPIO::LATCH_CH1_MASK);
    return get_io(mask);
}

void udc_phy::set_hw_present(const bool present)
{
    const uint32_t mask = GPIO::HW_PRESENT_MASK;
    set_io(mask, present ? 0 : mask); // active low
}

bool udc_phy::get_hw_present(void)
{
    const uint32_t mask = GPIO::HW_PRESENT_MASK;
    return ! (bool) get_io(mask);
}

daughterboard::type udc_phy::get_daughterboard_type(void)
{
    return _db_type;
}


void udc_phy::initialize_channels(void)
{
    reset_channels();
    discover_channels();
    validate_channels();
}

// Is this spi walking done because there wansn't a dedicated hdmi reset on thorium?
void udc_phy::reset_channels(void)
{
    // Deassert SPI Reset
    _radio->set_gpio_attr("GPIO", "OUT", GPIO::RESET_MASK << get_gpio_shift());
    //! Safe value to start, need to revisit startup delay when we have HW to test
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    for (size_t chan = 0; chan < daughterboard::channel_count(_db_type).value; chan++) {
        spi_transaction(chan, proto::command(proto::command::id::NOP));
    }
}

void udc_phy::discover_channels(void)
{
    bool found_channels = false;
    for (size_t chan = 0; chan < daughterboard::channel_count(_db_type).value; chan++) {
        spi_transaction(chan, proto::hw_status(proto::hw_status::id::SERIAL_1));

        // May get command_status::FW_PROCESSING as responses if the FW is ready fast enough
        // May get 0x00FFFFFF is data line is being held high, this is not valid either
        // Give it an arbitrary 3 chances to read back
        for (uint8_t attempts = 0; attempts < 3; attempts++)
        {
            const auto serial_rsp = spi_transaction(chan, proto::command(proto::command::id::NOP));
            if (serial_rsp > 0 && serial_rsp != udc::proto::command_status::FW_PROCESSING && serial_rsp != 0x00FFFFFF) 
            {
                found_channels = true;
                _channels[chan].is_connected = true;
                break;
            }
            else
            _channels[chan].is_connected = false;
        }
    }
    if (!found_channels) {
        UHD_LOG_WARNING(udc::NAME, "[" << udc::NAME << "] SPI communication did not discover any channels for Slot [" << _radio->get_slot_name() << "]");
    }
}

void udc_phy::validate_channels(void)
{
    // Toggle freq buffer registers, these are not applied unless APPLY_FREQ is called
    const proto::hw_control::id id = proto::hw_control::id::RFFREQ_H;
    for (int chan = 0; chan < daughterboard::channel_count(_db_type).value; chan++) {
        if (!_channels[chan].is_connected)
            continue;
        
        const uint16_t initial  = spi_send(chan, proto::hw_control(id), std::string(__func__)) & 0xFFFF;
        const uint16_t modified = ~(initial);
        spi_send(chan, proto::hw_control(id, modified), std::string(__func__));

        const uint16_t verified = spi_send(chan, proto::hw_control(id), std::string(__func__)) & 0xFFFF;
        spi_send(chan, proto::hw_control(id, initial), std::string(__func__));

        if (verified != modified) {
            std::stringstream msg;
            msg << "[" << udc::NAME << "] SPI communication failed to validate channels!";
            throw uhd::runtime_error(msg.str());
        }
    }
}

void udc_phy::set_io(uint32_t mask, const uint32_t value)
{
    mask       = mask << get_gpio_shift();
    auto state = _radio->get_gpio_attr("GPIO", "OUT");
    state      = (state & ~mask) | (value & mask);
    _radio->set_gpio_attr("GPIO", "OUT", state);
}

uint32_t udc_phy::get_io(uint32_t mask)
{
    auto state = _radio->get_gpio_attr("GPIO", "OUT");
    return state & (mask << get_gpio_shift());
}

udc_phy::spi_data udc_phy::spi_transaction(const size_t chan, const uint32_t command)
{
    auto time_now = std::chrono::high_resolution_clock::now();
    if ((_last_spi_transaction + SPI_US_DELAY_BETWEEN_TRANSACTIONS) > time_now)
        std::this_thread::sleep_for(SPI_US_DELAY_BETWEEN_TRANSACTIONS - (time_now - _last_spi_transaction));

    const uhd::spi_config_t config(uhd::spi_config_t::EDGE_RISE, _divider);
    udc_phy::spi_data result;
    if (chan == 0)
        result = _spi->transact_spi(CH0_SPI, config, command, 32, true);   // Note transact_spi only returns 24 bits
    else if (chan == 1)
        result = _spi->transact_spi(CH1_SPI, config, command, 32, true);   // Note transact_spi only returns 24 bits
    else    
        return 0;

    _last_spi_transaction = std::chrono::high_resolution_clock::now();
    return result;
}

size_t udc_phy::clock_to_divider(const double radio_clk, double spi_clk) const
{
    spi_clk = (spi_clk < 1e6) ? 1e6 : spi_clk;
    spi_clk = (spi_clk > 15e6) ? 15e6 : spi_clk;
    return static_cast<size_t>(((radio_clk / 2.0) / spi_clk) - 1.0);
}

uint32_t udc_phy::get_gpio_shift(void) const
{
    const auto slot = _radio->get_slot_name();
    return (slot == "A") ? 0 : 12;
}
} // namespace udc
