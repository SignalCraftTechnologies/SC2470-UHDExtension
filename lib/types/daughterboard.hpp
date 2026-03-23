//
// Copyright 2025 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <stddef.h>

namespace udc {

namespace daughterboard {

enum class type
{
    zbx,        // X410
    fbx_400,    // X440 - xx_400 FPGA image
    fbx_1600,   // X440 - xx_1600 FPGA image
};


constexpr const char* to_string(type t)
{
    switch (t) {
        case type::zbx:      return "zbx (X410)";       // X410
        case type::fbx_400:  return "fbx_400 (X440)";   // X440 xx_400
        case type::fbx_1600: return "fbx_1600 (X440)";  // X440 xx_1600
    }
    return "<unknown>";
}


struct channel_count
{
    channel_count(type db_type)
    {
        switch (db_type)
        {
        case type::zbx:
            value = 2;
            break;
        case type::fbx_400:
            value = 4;
            break;
        case type::fbx_1600:
            value = 1;
            break;        
        default:
            value = 0;
            break;
        }
    }
    size_t value;
};

// Max possible channels from valid daughterboard types
static constexpr size_t MAX_CHANNELS = 4;

} // namespace daughterboard

} // namespace udc