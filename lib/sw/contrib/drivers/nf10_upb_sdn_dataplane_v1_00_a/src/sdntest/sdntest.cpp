/*
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project
 *
 * Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
 *
 * Project Group "On-the-Fly Networking for Big Data"
 * SFB 901 "On-The-Fly Computing"
 *
 * University of Paderborn
 * Computer Engineering Group
 * Pohlweg 47 - 49
 * 33098 Paderborn
 * Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sdntest.hpp"
#include <stdint.h>
#include <stddef.h>
#include <iostream>
#include <sstream>
#include <exception>
#include <string>
#include <iomanip>
#include <vector>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/attributes/clock.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/date_time.hpp>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <errors.hpp>
#include <axi_bus_connector.hpp>
#include <platform_flash_xl_emc.hpp>
#include <sdn_dataplane.hpp>

namespace upb {

namespace b = boost;
namespace bpo = boost::program_options;
namespace bl = boost::log;
namespace ble = boost::log::expressions;
namespace bpt = boost::posix_time;

sdntest::sdntest()
{
}

} // end namespace upb

int main(int argc, char *argv[])
{
	using namespace upb;

	bl::core::get()->add_global_attribute("TimeStamp", bl::attributes::local_clock());

	bl::add_console_log (
		std::clog,
		bl::keywords::format =
		(
			ble::stream
				<< ble::format_date_time< bpt::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
				<< ": <" << bl::trivial::severity
				<< "> " << ble::smessage
		)
	);

	bl::core::get()->set_filter
	(
		bl::trivial::severity >= bl::trivial::trace
	);

	try {

		bpo::options_description desc("UPB SDN Library Test Program\n\nOptions");
		desc.add_options()
			("help", "Show program options")
			("device,d", bpo::value<std::string>(), "Specify NetFPGA device (e.g. \"/dev/nf10a\")")
			("read,r", bpo::value<std::string>(), "Read data from AXI-4-Lite Bus on the FPGA (specify address)")
			("write,w", bpo::value<std::vector<std::string>>(), "Write to AXI-4-Lite Bus on the FPGA (specify address and value)")
			("flash,f", bpo::value<std::string>(), "Program flash memory A of NetFPGA (specify .bin file name")
			("flashb,b", bpo::value<std::string>(), "Program flash memory B of NetFPGA (specify .bin file name")
			("clear,c", "Clear all flow tables")
		;

		bpo::variables_map vm;
		bpo::store(bpo::command_line_parser(argc, argv).options(desc).run(), vm);
		bpo::notify(vm);

		if (vm.count("help")) {
		    std::cout << desc << std::endl;
		    return 1;
		}

		if (!vm.count("device")) {
			std::cout << "Error: NetFPGA device was not set" << std::endl << std::endl;
			std::cout << desc << std::endl;
			return 1;
		}

		b::shared_ptr<axi_bus_connector> axi_bus = b::make_shared<axi_bus_connector>(vm["device"].as<std::string>().c_str());

		if (vm.count("read")) {

			uint32_t address;
			std::stringstream ss(vm["read"].as<std::string>());
			ss >> std::hex >> address;

			if (ss.fail()) {
				std::cerr << "Wrong format for address \"" << vm["read"].as<std::string>() << "\". You can use decimal or hexidecimal (0x12345678) notation." << std::endl;
				return 1;
			}

			std::cout << "0x" << std::setw(8) << std::hex << std::setfill('0') << axi_bus->read(address) << std::endl;;
		}

		if (vm.count("write")) {

			std::vector<std::string> params(vm["write"].as<std::vector<std::string>>());

			if (params.size() == 2) {

				uint32_t address, value;
				std::stringstream ss1(params[0]);
				ss1 >> std::hex >> address;
				std::stringstream ss2(params[1]);
				ss2 >> std::hex >> value;

				if (ss1.fail() || ss2.fail()) {
					std::cerr << "Wrong format for address and/or data. You can use decimal or hexidecimal (0x12345678) notation." << std::endl;
					return 1;
				}

				axi_bus->write(address, value);

			} else {
				std::cerr << "AXI write: Please specify address and data to be written" << std::endl;
			}
		}

		if (vm.count("flash")) {
			platform_flash_xl_emc flash(axi_bus, 0x80000000);
			flash.write(0, vm["flash"].as<std::string>());
		}

		if (vm.count("flashb")) {
			platform_flash_xl_emc flash(axi_bus, 0x80000000);
			flash.write(1, vm["flashb"].as<std::string>());
		}

		if (vm.count("clear")) {
			b::shared_ptr<sdn_dataplane> dp = b::make_shared<sdn_dataplane>(axi_bus);
		}

	}
	catch (const errors &e) {
		std::cerr << "Error: " << errors_text::error_text[(uint32_t)e] << std::endl;
		return 1;
	}
    catch(const std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        return 1;
    }
    catch(...) {
        std::cerr << "Unknown exception\n";
        return 1;
    }

    return 0;
}
