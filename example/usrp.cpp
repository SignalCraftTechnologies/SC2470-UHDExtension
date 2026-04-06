#include <sc2470/sc2470.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <vector>
#include <string>

int main()
{
    std::cout << "This example configures the x4xx as a USRP with a SC2470 extension" << std::endl;
    std::cout << "Connect HDMI cable between SC2470 and x4xx" << std::endl;
    std::cout << "WARNING - Ensure appropriate attenuation on RF cabling between devices to avoid equipment damage.  See SCT documentation for more details" << std::endl;
    std::cout << "PROVIDE MGMT IP ADDRESS FOR x4xx(eg: 192.168.0.109)" << std::endl;
    std::string ipstr;
    std::getline(std::cin, ipstr);
    std::cout << "PROVIDE SC2470 Daughterboard Slot Id 0 or 1 (eg: 0)" << std::endl;
    std::string slotStr;
    std::getline(std::cin, slotStr);
    uint16_t slotId = (slotStr == "1" ? 1 : 0);

    // create USRP device
    std::string args;
    args.append("type=x4xx,extension=sc2470,addr=");
    args.append(ipstr);
    std::cout << "Making USRP device with SC2470 extension..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << "Getting SC2470 Extension Classes..." << std::endl;
    udc::sc2470::sptr rx_extension = usrp->get_extension<udc::sc2470>(uhd::direction_t::RX_DIRECTION, slotId);
    udc::sc2470::sptr tx_extension = usrp->get_extension<udc::sc2470>(uhd::direction_t::TX_DIRECTION, slotId);
    if(rx_extension->get_is_connected(slotId) && tx_extension->get_is_connected(slotId)) {
        std::cout << "SC2470 is connected over HDMI" << std::endl;
    } else {
        std::cout << "ERROR - SC2470 is not connected over HDMI" << std::endl;
        exit(1);
    }
    std::cout << "x4xx daughterboard detected as type: " << rx_extension->get_daughterboard_type() << std::endl;

    // Configure RF Frequency
    double freq_rf = 10e9;
    double freq_if = 750e6;
    uhd::tune_request_t tune_request = uhd::tune_request_t(freq_rf);
    uhd::tune_result_t rx_tune_result = usrp->set_rx_freq(tune_request, slotId);
    std::cout << "Actual RX RF Frequency (Hz): " << rx_tune_result.actual_rf_freq << std::endl;
    uhd::tune_result_t tx_tune_result = usrp->set_tx_freq(tune_request, slotId);
    std::cout << "Actual TX RF Frequency (Hz): " << tx_tune_result.actual_rf_freq << std::endl;
    // Override IF frequency
    rx_extension->set_rx_if(freq_if, slotId);
    tx_extension->set_tx_if(freq_if, slotId);
    std::cout << "Actual RX IF Frequency (Hz): " << rx_extension->get_rx_if(slotId) << std::endl;
    std::cout << "Actual TX IF Frequency (Hz): " << tx_extension->get_tx_if(slotId) << std::endl;

    // Set to minimum gain
    uhd::gain_range_t rx_gain_range = usrp->get_rx_gain_range(slotId);
    uhd::gain_range_t tx_gain_range = usrp->get_tx_gain_range(slotId);
    usrp->set_rx_gain(rx_gain_range.start(), slotId);
    usrp->set_tx_gain(tx_gain_range.start(), slotId);
    std::cout << "Actual RX Gain (dB): " << rx_extension->get_rx_gain(slotId) << std::endl;
    std::cout << "Actual TX Gain (dB): " << tx_extension->get_tx_gain(slotId) << std::endl;
    
    return 0;
}