//
// Copyright 2023 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <stdint.h>
#include <string>

namespace udc {

class version
{
private:
    uint8_t _major;
    uint8_t _minor;

public:
    version(const uint16_t serialized)
    {
        _major = static_cast<uint8_t>((serialized & 0xFF00) >> 8);
        _minor = static_cast<uint8_t>(serialized & 0x00FF);
    }
    version(uint8_t major, uint8_t minor) : _major(major), _minor(minor) {}

    bool operator==(const version& rhs) const
    {
        return (this->_major == rhs._major) && (this->_minor == rhs._minor);
    }

    bool operator<(const version& rhs) const
    {
        return (this->_major < rhs._major)    ? true
               : (this->_major == rhs._major) ? (this->_minor < rhs._minor)
                                              : false;
    }

    bool operator!=(const version& rhs) const
    {
        return !(rhs == *this);
    }

    bool operator>(const version& rhs) const
    {
        return rhs < *this;
    }

    bool operator<=(const version& rhs) const
    {
        return !(*this > rhs);
    }

    bool operator>=(const version& rhs) const
    {
        return !(*this < rhs);
    }

    std::string to_string() const
    {
        return std::to_string(_major) + "." + std::to_string(_minor);
    }
};

} // namespace udc
