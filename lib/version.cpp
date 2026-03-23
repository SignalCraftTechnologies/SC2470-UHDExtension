#include "version.hpp"

namespace udc
{

std::string get_version(void)
{
    return std::string(std::to_string(version::major) + "." + std::to_string(version::minor) + "." + std::to_string(version::point));
}

uint16_t get_version_major(void)
{
    return version::major;
}

uint16_t get_version_min(void)
{
    return version::minor;
}

uint16_t get_version_point(void)
{
    return version::point;
}

} // namespace udc
