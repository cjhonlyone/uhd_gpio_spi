//
// Copyright 2014-2016 Ettus Research LLC
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhd/rfnoc/ddc_block_control.hpp>
#include <uhd/rfnoc/defaults.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

#include <uhdgpiospi.cpp>

namespace po = boost::program_options;

constexpr int64_t UPDATE_INTERVAL = 1; // 1 second update interval for BW summary

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

typedef std::function<uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names,
    const char* sensor_name,
    get_sensor_fn_t get_sensor_fn,
    double setup_time)
{
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name)
        == sensor_names.end())
        return false;

    auto setup_timeout = std::chrono::steady_clock::now()
                         + std::chrono::milliseconds(int64_t(setup_time * 1000));
    bool lock_detected = false;

    std::cout << "Waiting for \"" << sensor_name << "\": ";
    std::cout.flush();

    while (true) {
        if (lock_detected and (std::chrono::steady_clock::now() > setup_timeout)) {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()) {
            std::cout << "+";
            std::cout.flush();
            lock_detected = true;
        } else {
            if (std::chrono::steady_clock::now() > setup_timeout) {
                std::cout << std::endl;
                throw std::runtime_error(
                    std::string("timed out waiting for consecutive locks on sensor \"")
                    + sensor_name + "\"");
            }
            std::cout << "_";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    return true;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, file, format, ant, subdev, ref, wirefmt, streamargs, block_id,
        block_props;
    size_t total_num_samps, spb, spp, radio_id, radio_chan, block_port;
    double rate, freq, gain, bw, total_time, setup_time;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("format", po::value<std::string>(&format)->default_value("sc16"), "File sample format: sc16, fc32, or fc64")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("spp", po::value<size_t>(&spp), "samples per packet (on FPGA and wire)")
        ("streamargs", po::value<std::string>(&streamargs)->default_value(""), "stream args")
        ("progress", "periodically display short-term bandwidth")
        ("stats", "show average bandwidth on exit")
        ("sizemap", "track packet size and display breakdown on exit")
        ("null", "run without writing to file")
        ("continue", "don't abort on a bad packet")

        ("args", po::value<std::string>(&args)->default_value(""), "USRP device address args")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")

        ("radio-id", po::value<size_t>(&radio_id)->default_value(0), "Radio ID to use (0 or 1).")
        ("radio-chan", po::value<size_t>(&radio_chan)->default_value(0), "Radio channel")
        ("rate", po::value<double>(&rate)->default_value(1e6), "RX rate of the radio block")
        ("freq", po::value<double>(&freq)->default_value(0.0), "RF center frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref), "reference source (internal, external, mimo)")
        ("skip-lo", "skip checking LO lock status")
        ("int-n", "tune USRP with integer-N tuning")

        ("block-id", po::value<std::string>(&block_id), "If block ID is specified, this block is inserted between radio and host.")
        ("block-port", po::value<size_t>(&block_port)->default_value(0), "If block ID is specified, this block is inserted between radio and host.")
        ("block-props", po::value<std::string>(&block_props), "These are passed straight to the block as properties (see set_properties()).")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "UHD/RFNoC RX samples to file " << desc << std::endl;
        std::cout << std::endl
                  << "This application streams data from a single channel of a USRP "
                     "device to a file.\n"
                  << std::endl;
        return ~0;
    }

    bool bw_summary = vm.count("progress") > 0;
    bool stats      = vm.count("stats") > 0;
    if (vm.count("null") > 0) {
        file = "";
    }
    bool enable_size_map        = vm.count("sizemap") > 0;
    bool continue_on_bad_packet = vm.count("continue") > 0;

    if (enable_size_map) {
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!"
                  << std::endl;
    }

    if (format != "sc16" and format != "fc32" and format != "fc64") {
        std::cout << "Invalid sample format: " << format << std::endl;
        return EXIT_FAILURE;
    }

    /************************************************************************
     * Create device and block controls
     ***********************************************************************/
    std::cout << std::endl;
    std::cout << "Creating the RFNoC graph with args: " << args << std::endl;
    auto graph = uhd::rfnoc::rfnoc_graph::make(args);

    // Create handle for radio object
    uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", radio_id);
    // This next line will fail if the radio is not actually available
    auto radio_ctrl = graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_id);
    std::cout << "Using radio " << radio_id << ", channel " << radio_chan << std::endl;

    uhd::rfnoc::block_id_t last_block_in_chain;
    size_t last_port_in_chain;
    uhd::rfnoc::ddc_block_control::sptr ddc_ctrl;
    size_t ddc_chan       = 0;
    bool user_block_found = false;

    { // First, connect everything dangling off of the radio
        auto edges = uhd::rfnoc::get_block_chain(graph, radio_ctrl_id, radio_chan, true);
        last_block_in_chain = edges.back().src_blockid;
        last_port_in_chain  = edges.back().src_port;
        if (edges.size() > 1) {
            uhd::rfnoc::connect_through_blocks(graph,
                radio_ctrl_id,
                radio_chan,
                last_block_in_chain,
                last_port_in_chain);
            for (auto& edge : edges) {
                if (uhd::rfnoc::block_id_t(edge.dst_blockid).get_block_name() == "DDC") {
                    ddc_ctrl =
                        graph->get_block<uhd::rfnoc::ddc_block_control>(edge.dst_blockid);
                    ddc_chan = edge.dst_port;
                }
                if (vm.count("block-id") && edge.dst_blockid == block_id) {
                    user_block_found = true;
                }
            }
        }
    }

    // If the user block is not in the chain yet, see if we can connect that
    // separately
    if (vm.count("block-id") && !user_block_found) {
        const auto user_block_id = uhd::rfnoc::block_id_t(block_id);
        if (!graph->has_block(user_block_id)) {
            std::cout << "ERROR! No such block: " << block_id << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Attempting to connect " << block_id << ":" << last_port_in_chain
                  << " to " << last_block_in_chain << ":" << block_port << "..."
                  << std::endl;
        uhd::rfnoc::connect_through_blocks(
            graph, last_block_in_chain, last_port_in_chain, user_block_id, block_port);
        last_block_in_chain = uhd::rfnoc::block_id_t(block_id);
        last_port_in_chain  = block_port;
        // Now we have to make sure that there are no more static connections
        // after the user-defined block
        auto edges = uhd::rfnoc::get_block_chain(
            graph, last_block_in_chain, last_port_in_chain, true);
        if (edges.size() > 1) {
            uhd::rfnoc::connect_through_blocks(graph,
                last_block_in_chain,
                last_port_in_chain,
                edges.back().src_blockid,
                edges.back().src_port);
            last_block_in_chain = edges.back().src_blockid;
            last_port_in_chain  = edges.back().src_port;
        }
    }
    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Lock mboard clocks
    if (vm.count("ref")) {
        graph->get_mb_controller(0)->set_clock_source(ref);
    }

    // set the center frequency
    if (vm.count("freq")) {
        std::cout << "Requesting RX Freq: " << (freq / 1e6) << " MHz..." << std::endl;
        uhd::tune_request_t tune_request(freq);
        if (vm.count("int-n")) {
            radio_ctrl->set_rx_tune_args(
                uhd::device_addr_t("mode_n=integer"), radio_chan);
        }
        radio_ctrl->set_rx_frequency(freq, radio_chan);
        std::cout << "Actual RX Freq: "
                  << (radio_ctrl->get_rx_frequency(radio_chan) / 1e6) << " MHz..."
                  << std::endl
                  << std::endl;
    }

    // set the rf gain
    if (vm.count("gain")) {
        std::cout << "Requesting RX Gain: " << gain << " dB..." << std::endl;
        radio_ctrl->set_rx_gain(gain, radio_chan);
        std::cout << "Actual RX Gain: " << radio_ctrl->get_rx_gain(radio_chan) << " dB..."
                  << std::endl
                  << std::endl;
    }

    // set the IF filter bandwidth
    if (vm.count("bw")) {
        std::cout << "Requesting RX Bandwidth: " << (bw / 1e6) << " MHz..." << std::endl;
        radio_ctrl->set_rx_bandwidth(bw, radio_chan);
        std::cout << "Actual RX Bandwidth: "
                  << (radio_ctrl->get_rx_bandwidth(radio_chan) / 1e6) << " MHz..."
                  << std::endl
                  << std::endl;
    }

    // set the antenna
    if (vm.count("ant")) {
        radio_ctrl->set_rx_antenna(ant, radio_chan);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1000 * setup_time)));

    // check Ref and LO Lock detect
    if (not vm.count("skip-lo")) {
        check_locked_sensor(
            radio_ctrl->get_rx_sensor_names(radio_chan),
            "lo_locked",
            [&](const std::string& sensor_name) {
                return radio_ctrl->get_rx_sensor(sensor_name, radio_chan);
            },
            setup_time);
        if (ref == "external") {
            check_locked_sensor(
                graph->get_mb_controller(0)->get_sensor_names(),
                "ref_locked",
                [&](const std::string& sensor_name) {
                    return graph->get_mb_controller(0)->get_sensor(sensor_name);
                },
                setup_time);
        }
    }

    if (vm.count("spp")) {
        std::cout << "Requesting samples per packet of: " << spp << std::endl;
        radio_ctrl->set_property<int>("spp", spp, radio_chan);
        spp = radio_ctrl->get_property<int>("spp", radio_chan);
        std::cout << "Actual samples per packet = " << spp << std::endl;
    }

    /************************************************************************
     * Set up streaming
     ***********************************************************************/
    uhd::device_addr_t streamer_args(streamargs);

    // create a receive streamer
    uhd::stream_args_t stream_args(
        format, "sc16"); // We should read the wire format from the blocks
    stream_args.args = streamer_args;
    std::cout << "Using streamer args: " << stream_args.args.to_string() << std::endl;
    auto rx_stream = graph->create_rx_streamer(1, stream_args);

    // Connect streamer to last block and commit the graph
    graph->connect(last_block_in_chain, last_port_in_chain, rx_stream, 0);
    graph->commit();
    std::cout << "Active connections:" << std::endl;
    for (auto& edge : graph->enumerate_active_connections()) {
        std::cout << "* " << edge.to_string() << std::endl;
    }

    /************************************************************************
     * Set up sampling rate and (optional) user block properties. We do this
     * after commit() so we can use the property propagation.
     ***********************************************************************/
    // set the sample rate
    if (rate <= 0.0) {
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Requesting RX Rate: " << (rate / 1e6) << " Msps..." << std::endl;
    if (ddc_ctrl) {
        std::cout << "Setting rate on DDC block!" << std::endl;
        rate = ddc_ctrl->set_output_rate(rate, ddc_chan);
    } else {
        std::cout << "Setting rate on radio block!" << std::endl;
        rate = radio_ctrl->set_rate(rate);
    }
    std::cout << "Actual RX Rate: " << (rate / 1e6) << " Msps..." << std::endl
              << std::endl;

    if (vm.count("block-props")) {
        std::cout << "Setting block properties to: " << block_props << std::endl;
        graph->get_block(uhd::rfnoc::block_id_t(block_id))
            ->set_properties(uhd::device_addr_t(block_props));
    }


    #define GPIO_SCK_MASK      (0x00000001 << 0)
    #define GPIO_SCS_MASK      (0x00000001 << 1)
    #define GPIO_SDO_MASK      (0x00000001 << 2)
    #define GPIO_SDI_MASK      (0x00000001 << 3)
    
    /* Front GPIO SPI*/
    UhdGpioSpi *UhdGpioSpiInstance;
    UhdGpioSpiInstance = new UhdGpioSpi(radio_ctrl,GPIO_SCK_MASK, GPIO_SDO_MASK, GPIO_SDI_MASK, GPIO_SCS_MASK);
    UhdGpioSpiInstance->set_cpol(false);
    UhdGpioSpiInstance->set_cpha(false);
    UhdGpioSpiInstance->set_frequency(1e6);
    UhdGpioSpiInstance->init();

    uint8_t spi_write_data[4];
    uint8_t spi_read_data[4];

    spi_write_data[0] = 0x11;
    spi_write_data[1] = 0x21;
    spi_write_data[2] = 0x14;
    spi_write_data[3] = 0x51;

    spi_read_data[0] = 0x00;
    spi_read_data[1] = 0x00;
    spi_read_data[2] = 0x00;
    spi_read_data[3] = 0x00;

    UhdGpioSpiInstance->write_and_read(spi_write_data, spi_read_data, 4);

    for (uint8_t i = 0 ;i < 4;i++)
    {
        std::cout << std::hex << uint32_t(spi_read_data[i]) << std::dec << std::endl;
    }
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
