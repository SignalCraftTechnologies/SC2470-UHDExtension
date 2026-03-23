//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include "protocol.hpp"
#include "sc2470_constants.hpp"
#include "types/daughterboard.hpp"
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/serial.hpp>
#include <stdint.h>
#include <array>
#include <chrono>

namespace udc {

// This class contains the functionality to control the SC2444 via SPI and GPIO
class udc_phy final
{
private:
    static constexpr size_t SPI_MAX_DEV = 4;
    const std::chrono::microseconds SPI_US_DELAY_BETWEEN_TRANSACTIONS = std::chrono::microseconds(200);

    using spi_data = uint32_t;

    uhd::rfnoc::radio_control::sptr _radio;
    uhd::spi_iface::sptr _spi;
    size_t _divider;
    daughterboard::type _db_type;

    std::chrono::time_point<std::chrono::high_resolution_clock> _last_spi_transaction;

    struct channel
    {
        bool is_connected = false;
    };
    std::array<channel, daughterboard::MAX_CHANNELS> _channels;


public:
    using sptr = std::shared_ptr<udc_phy>;
    udc_phy(uhd::rfnoc::radio_control::sptr radio, daughterboard::type db_type);

    bool is_connected(const size_t chan) const;
    uint32_t spi_send(const size_t chan, const proto::command command, const std::string description);

    void set_attn_latch(const size_t chan, const bool enable);
    bool get_attn_latch(const size_t chan);
    void set_hw_present(const bool present);
    bool get_hw_present(void);
    daughterboard::type get_daughterboard_type(void);

private:
    void initialize_channels(void);
    void reset_channels(void);
    void discover_channels(void);
    void validate_channels(void);

    void set_io(uint32_t mask, uint32_t value);
    uint32_t get_io(uint32_t mask);

    spi_data spi_transaction(const size_t chan, const uint32_t command);

    size_t clock_to_divider(const double radio_clk, double spi_clk) const;
    uint32_t get_gpio_shift(void) const;
};

} // namespace udc
