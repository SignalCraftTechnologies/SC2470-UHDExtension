#include "range_util.hpp"
#include "float_util.hpp"

namespace utility
{

/*
This util function is for clipping to a range and then correcting error
in the uhd::meta_range_t class where it sometimes steps outside.
*/
double fixed_uhd_meta_range_clip(uhd::meta_range_t range, double value, bool do_step)
{
    double clipped = range.clip(value, do_step);
    double start = range.start();
    double stop = range.stop();
    double step = range.step();

    if(!step) {
        return clipped;
    }
    
    if(step > (stop - start)) {
        return clipped;
    }

    while (true) { // uhd clip cannot be trusted to keep values within range when stepping.
        if (clipped < start) {
            clipped += step;
        } else if (clipped > stop) {
            clipped -= step;
        } else {
            break;
        }
    }
    return clipped;
}

} // namespace utility