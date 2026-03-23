//
// Copyright 2024 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once
#include "sc2470_constants.hpp"
#include <stdint.h>
#include <array>

// Defined as per SCT-SW1A61vc Binary Protocol
namespace udc { namespace proto {

class command
{
public:
    enum class id {
        NOP       = 0x00,
        TDD_GAIN  = 0x21,        
        ACT_PATH  = 0x23,
        HW_CTRL   = 0x24,
        LO_ACT_PATH = 0x33,
        LO_CTRL   = 0x34,
        HW_STATUS = 0x35,
        HEALTH    = 0x36,
    };

protected:
    id _id         = id::NOP;
    uint32_t _data = 0;
    bool _is_read  = false;

public:
    constexpr operator uint32_t() const
    {
        const uint32_t data = (_data & 0x00FFFFFF) | (static_cast<uint8_t>(_id) << 24);
        return _is_read ? (data | 0x80000000) : (data & 0x7FFFFFFF);
    }
    constexpr bool is_read(void) const
    {
        return _is_read;
    }
    constexpr command(const command::id id, const uint32_t value) : _id(id)
    {
        _data = (value & 0xFFFFFF);
    }
    constexpr command(const command::id id) : command(id, 0)
    {
        _is_read = (id != id::NOP);
    }
    constexpr command(const uint32_t data) : _data(data)
    {
        _id      = static_cast<id>((data >> 24) & 0xFF);
        _is_read = (data & 0x80000000);
    }
    constexpr command() = default;
};

class lo_control : public command
{
public:
    enum class id {
        REFPLL_CONFIG = 0x14,
    };
    constexpr lo_control(const lo_control::id id, const uint16_t value)
    {
        _id   = command::id::LO_CTRL;
        _data = (static_cast<uint8_t>(id) << 16) | value;
    }
    constexpr lo_control(const lo_control::id id) : lo_control(id, 0)
    {
        _is_read = true;
    }
};

class hw_control final : public command
{
public:
    enum class id {
        RF_PATH         = 0x04,
        TDD_PATH        = 0x05,
        EXT_TRIG        = 0x06,
        GAIN_NOW        = 0x07,
        APPLY_GAIN      = 0x08,
        RFFREQ_H        = 0x11,
        RFFREQ_M        = 0x12,
        RFFREQ_L        = 0x13,
        IFFREQ_H        = 0x14,
        IFFREQ_M        = 0x15,
        IFFREQ_L        = 0x16,
        LOFREQ_H        = 0x17,
        LOFREQ_M        = 0x18,
        LOFREQ_L        = 0x19,
        APPLY_FREQ      = 0x1A,
        LOAD_FREQ       = 0x1B,
        GAIN_LIM_MIN    = 0x1D,
        GAIN_LIM_MAX    = 0x1E,
        HW_RESET        = 0xFF,
    };

    constexpr hw_control(const hw_control::id id, const uint16_t value)
    {
        _id   = command::id::HW_CTRL;
        _data = (static_cast<uint8_t>(id) << 16) | value;
    }
    constexpr hw_control(const hw_control::id id) : hw_control(id, 0)
    {
        _is_read = true;
    }
};

class hw_status final : public command
{
public:
    enum class id {
        CH_ID    = 0x01,
        SERIAL_1 = 0x02,
        SERIAL_2 = 0x03,
        HW_REV   = 0x04,
    };

    constexpr hw_status(const hw_status::id id)
    {
        _id      = command::id::HW_STATUS;
        _data    = static_cast<uint8_t>(id) << 16;
        _is_read = true;
    }
};

class tdd_gain final : public command
{
private:
    static constexpr int RX_ATTN_EN_MASK = 0x100000;
    static constexpr int TX_ATTN_EN_MASK = 0x200000;
    static constexpr int GAIN_MASK = 0x0000FFFF;
    static constexpr int GAIN_SHIFT = 0;

public:
    struct rx_attn
    {
        const double step;
        const double setting;
    };
    struct tx_attn
    {
        const double step;
        const double setting;
    };

    constexpr tdd_gain(void)
    {
        _id      = command::id::TDD_GAIN;
        _is_read = true;
    }

    constexpr tdd_gain(const rx_attn& attn)
    {
        _id   = command::id::TDD_GAIN;
        _data = RX_ATTN_EN_MASK;
        _data |= (static_cast<int16_t>(attn.setting / attn.step) << GAIN_SHIFT) & GAIN_MASK;
    }
    constexpr tdd_gain(const tx_attn& attn)
    {
        _id   = command::id::TDD_GAIN;
        _data = TX_ATTN_EN_MASK;
        _data |= (static_cast<int16_t>(attn.setting / attn.step) << GAIN_SHIFT) & GAIN_MASK;
    }
};

struct freq_millihz
{
    const uint64_t HIGH_MASK = 0x0000FFFF00000000;
    const size_t HIGH_SHIFT  = 32;
    const uint64_t MED_MASK  = 0x00000000FFFF0000;
    const size_t MED_SHIFT   = 16;
    const uint64_t LOW_MASK  = 0x000000000000FFFF;
    const size_t LOW_SHIFT   = 0;

    uint64_t full;
    uint16_t high_bits;
    uint16_t med_bits;
    uint16_t low_bits;

    freq_millihz(double freq)
    {
        full = (uint64_t)(freq * 1000);
        high_bits = (uint16_t)((full & HIGH_MASK) >> HIGH_SHIFT);
        med_bits = (uint16_t)((full & MED_MASK) >> MED_SHIFT);
        low_bits = (uint16_t)((full & LOW_MASK) >> LOW_SHIFT);
    }
    freq_millihz(uint32_t high, uint32_t med, uint32_t low)
    {
        high_bits = (uint16_t)(high & 0xFFFF);
        med_bits = (uint16_t)(med & 0xFFFF);
        low_bits = (uint16_t)(low & 0xFFFF);
        full = ((((uint64_t)high_bits) << HIGH_SHIFT) & HIGH_MASK)
            | ((((uint64_t)med_bits) << MED_SHIFT) & MED_MASK)
            | ((((uint64_t)low_bits) << LOW_SHIFT) & LOW_MASK);
    }
    freq_millihz& operator=(const freq_millihz& new_vals)
    {
        full = new_vals.full;
        high_bits = new_vals.high_bits;
        med_bits = new_vals.med_bits;
        low_bits = new_vals.low_bits;
        return *this;
    }
};

struct gain_set
{
private:
    const uint32_t GAIN_MASK = 0x0000FFFF;
    const uint32_t GAIN_SHIFT = 0;
    const double GAIN_STEP = DEFAULT_GAIN_STEP;

public:
    double max;
    double min;
    bool is_range;
    uhd::meta_range_t range;

    gain_set(uint32_t min_gain, uint32_t max_gain)
    {
        int16_t max_steps = (int16_t)((max_gain & GAIN_MASK) >> GAIN_SHIFT);
        int16_t min_steps = (int16_t)((min_gain & GAIN_MASK) >> GAIN_SHIFT);

        max = ((double)max_steps) * GAIN_STEP;
        min = ((double)min_steps) * GAIN_STEP;

        if (min > max)
            min = max;

        range = {min, max, GAIN_STEP};        
    }
};

struct value
{
    enum act_path {
        ACT_PATH_RX = 0b0,
        ACT_PATH_TX = 0b1,
    };
    enum lo_act_path {
        LO_ACT_PATH_RX = 0b0,
        LO_ACT_PATH_TX = 0b1,
    };
    enum rf_path {
        RF_PATH_FDD = 0x00,
        RF_PATH_TDD = 0x01,
    };
    enum ref_source {
        REF_SOURCE_INT = 0b01,
        REF_SOURCE_EXT_10MHZ = 0b10,
        REF_SOURCE_EXT_100MHZ = 0b11,
    };
    enum ext_trig {
        EXT_TRIG_DISABLE = 0b00,
        EXT_TRIG_ENABLE  = 0b11,
    };
    enum apply_freq {
        APPLY_FREQ_RFFREQ = 0b100,
        APPLY_FREQ_IFFREQ = 0b010,
        APPLY_FREQ_LOFREQ = 0b001,
    };
    enum load_freq {
        LOAD_FREQ_RFFREQ = 0b100,
        LOAD_FREQ_IFFREQ = 0b010,
        LOAD_FREQ_LOFREQ = 0b001,
    };

    static constexpr uint16_t HW_RESET = 0xA57B;
};

class command_status final
{
private:
    static constexpr uint32_t STATUS_NOTHING_PENDING = 0;
    static constexpr uint32_t SENT_READ_MASK = 0x80000000;
    static constexpr uint32_t SENT_CMD_MASK = 0xFF000000;
    static constexpr uint32_t SENT_CMD_SHIFT = 24;    
    static constexpr uint32_t SENT_INDEX_MASK = 0x00FF0000;
    static constexpr uint32_t SENT_INDEX_SHIFT = 16;
    static constexpr uint32_t PENDING_CMD_MASK = 0x00FF0000;
    static constexpr uint32_t PENDING_CMD_SHIFT = 16;
    static constexpr uint32_t STATUS_PENDING_MASK = 0x00000002;
    static constexpr uint32_t STATUS_PENDING_SHIFT = 1;
    static constexpr uint32_t STATUS_ERROR_MASK = 0x00000001;
    static constexpr uint32_t STATUS_ERROR_SHIFT = 0;

    static constexpr uint32_t READ_ERROR_MASK = 0x00800000;
    
    bool _valid_ack = false;
    bool _pending = true;
    bool _error = false;
    bool _retry = false;
    bool _was_read  = false;
    uint32_t _received = 0;

public:
    static constexpr uint32_t FW_PROCESSING = 0x00FF55AA;

    constexpr command_status(const uint32_t sent, const uint32_t received)
    {
        _was_read = (sent & SENT_READ_MASK);
        _received = received;

        if (_was_read)
        {
            if (received == FW_PROCESSING)
            {
                _valid_ack = true;
                _pending = true;
            }
            else if (received & READ_ERROR_MASK)
            {
                _valid_ack = true;
                _pending = false;
                _error = true;
            }
            // These commands do not echo an index back (unlike like the other read commands)
            else if ((((sent & SENT_CMD_MASK) >> SENT_CMD_SHIFT) == static_cast<uint32_t>(command::id::TDD_GAIN))
                || (((sent & SENT_CMD_MASK) >> SENT_CMD_SHIFT) == static_cast<uint32_t>(command::id::ACT_PATH))
                || (((sent & SENT_CMD_MASK) >> SENT_CMD_SHIFT) == static_cast<uint32_t>(command::id::LO_ACT_PATH)))
            {
                _valid_ack = true;
                _pending = false;
                _error = false;
            }
            else if (((sent & SENT_INDEX_MASK) >> SENT_INDEX_SHIFT) == ((received & PENDING_CMD_MASK) >> PENDING_CMD_SHIFT))
            {
                _valid_ack = true;
                _pending = false;
                _error = false;
            }
            else
            {
                _valid_ack = false;
            }
        }
        else
        {
            if (((sent & SENT_CMD_MASK) >> SENT_CMD_SHIFT) == (received & PENDING_CMD_MASK) >> PENDING_CMD_SHIFT)
            {
                _valid_ack = true;
                _pending = ((received & STATUS_PENDING_MASK) >> STATUS_PENDING_SHIFT);
                _error = ((received & STATUS_ERROR_MASK) >> STATUS_ERROR_SHIFT);
            }
            else if (received == FW_PROCESSING)
            {
                _valid_ack = true;
                _pending = true;
            }
            else if (received == STATUS_NOTHING_PENDING)
            {
                _valid_ack = true;
                _retry = true;
            }    
            else
            {
                _valid_ack = false;
            }
        }
    }
    constexpr bool is_valid_response()
    {
        return _valid_ack;
    }
    constexpr bool is_cmd_pending()
    {
        return _pending;
    }
    constexpr bool is_cmd_error()
    {
        return _error;
    }
    constexpr bool is_cmd_retry()
    {
        return _retry;
    }
};

}} // namespace udc::proto
