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

#include "logging.hpp"
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/attributes/clock.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/date_time.hpp>

namespace upb {

namespace bl = boost::log;
namespace ble = boost::log::expressions;
namespace bpt = boost::posix_time;

void logging::init_logging()
{
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
}

void logging::log_level_trace()
{
	bl::core::get()->set_filter
	(
		bl::trivial::severity >= bl::trivial::trace
	);
}

void logging::log_level_debug()
{
	bl::core::get()->set_filter
	(
		bl::trivial::severity >= bl::trivial::debug
	);
}

void logging::log_level_info()
{
	bl::core::get()->set_filter
	(
		bl::trivial::severity >= bl::trivial::info
	);
}


}
