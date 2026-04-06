#pragma once

#include <string>
#include <cstdint>

namespace udc 
{

namespace version
{
    const uint16_t major = 1;
    const uint16_t minor = 1;
    const uint16_t point = 0;
}

std::string get_version(void);
uint16_t get_version_major(void);
uint16_t get_version_min(void);
uint16_t get_version_point(void);

} // namespace udc
