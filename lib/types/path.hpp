//
// Copyright 2023 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/types/direction.hpp>
#include <iostream>
#include <tuple>

namespace udc {

struct path
{
    size_t chan;
    uhd::direction_t trx;

    friend std::ostream& operator<<(std::ostream& os, const path& path)
    {
        os << "CH" << path.chan << "-" << path.to_trx_string();
        return os;
    }

    std::string to_trx_string(void) const
    {
        return (trx == uhd::direction_t::TX_DIRECTION) ? "TX" : "RX";
    }

    bool operator<(const path& other) const
    {
        return std::tie(this->chan, this->trx) < std::tie(other.chan, other.trx);
    }
};

} // namespace udc
