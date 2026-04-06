#include <sc2470/sc2470.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <vector>
#include <string>

int main()
{
    std::cout << "This example configures the x4xx as a RFNoC Graph with a SC2470 extension" << std::endl;
    std::cout << "Connect HDMI cable between SC2470 and x4xx" << std::endl;
    std::cout << "WARNING - Ensure appropriate attenuation on RF cabling between devices to avoid equipment damage.  See SCT documentation for more details" << std::endl;
    std::cout << "PROVIDE MGMT IP ADDRESS FOR x4xx(eg: 192.168.0.109)" << std::endl;
    std::string ipstr;
    std::getline(std::cin, ipstr);
    std::cout << "PROVIDE SC2470 Daughterboard Slot Id 0 or 1 (eg: 0)" << std::endl;
    std::string slotStr;
    std::getline(std::cin, slotStr);
    uint16_t slotId = (slotStr == "1" ? 1 : 0);

    // create RFNOC device
    std::string args;
    args.append("type=x4xx,addr=");
    args.append(ipstr);
    std::cout << "Making RFNoC device with SC2470 extension..." << std::endl;
    std::shared_ptr<uhd::rfnoc::rfnoc_graph> rfnoc_graph = uhd::rfnoc::rfnoc_graph::make(args);
    std::cout << "Making SC2470 Extension Class..." << std::endl;
    std::string blockIdStr = "0/Radio#";
    blockIdStr.append(std::to_string(slotId));
    uhd::extension::extension::factory_args factory_args = {rfnoc_graph->get_block<uhd::rfnoc::radio_control>(blockIdStr), rfnoc_graph->get_mb_controller()};
    auto extension = udc::sc2470::make(factory_args);

    if(extension->get_is_connected(slotId)) {
        std::cout << "SC2470 is connected over HDMI" << std::endl;
    } else {
        std::cout << "ERROR - SC2470 is not connected over HDMI" << std::endl;
        exit(1);
    }
    std::cout << "x4xx daughterboard detected as type: " << extension->get_daughterboard_type() << std::endl;

    // Configure RF Frequency
    double freq_rf = 10e9;
    double freq_if = 750e6;
    double rx_freq_rf = extension->set_rx_frequency(freq_rf, slotId);
    double tx_freq_rf = extension->set_rx_frequency(freq_rf, slotId);
    extension->set_tx_frequency(freq_rf, slotId);
    std::cout << "Actual RX RF Frequency (Hz): " << rx_freq_rf << std::endl;
    std::cout << "Actual TX RF Frequency (Hz): " << tx_freq_rf << std::endl;
    // Override IF frequency
    extension->set_rx_if(freq_if, slotId);
    extension->set_tx_if(freq_if, slotId);
    std::cout << "Actual RX IF Frequency (Hz): " << extension->get_rx_if(slotId) << std::endl;
    std::cout << "Actual TX IF Frequency (Hz): " << extension->get_tx_if(slotId) << std::endl;

    // Set to minimum gain
    uhd::gain_range_t rx_gain_range = extension->get_rx_gain_range(slotId);
    uhd::gain_range_t tx_gain_range = extension->get_tx_gain_range(slotId);
    extension->set_rx_gain(rx_gain_range.start(), slotId);
    extension->set_tx_gain(tx_gain_range.start(), slotId);
    std::cout << "Actual RX Gain (dB): " << extension->get_rx_gain(slotId) << std::endl;
    std::cout << "Actual TX Gain (dB): " << extension->get_tx_gain(slotId) << std::endl;
    
    return 0;
}