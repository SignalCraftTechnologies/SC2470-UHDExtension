#pragma once

#include "types/path.hpp"
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/ranges.hpp>

namespace utility
{

double fixed_uhd_meta_range_clip(uhd::meta_range_t range, double value, bool do_step);

} // namespace utility