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

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread.hpp>

namespace upb {

namespace b = boost;
namespace bs2 = boost::signals2;

/**
 * Helper class the provide a thread which periodically calls a signal
 */
class poll_thread {

public:
	typedef b::shared_ptr<b::recursive_mutex> mutex_t;
	typedef bs2::signal<void ()> signal_t;

	/**
	 * Construct the polling thread
	 * @param mutex A boost recursive_mutex which is locked during the signal invcation
	 * @param poll_interval_ms The polling interval in milliseconds
	 */
	poll_thread(mutex_t &mutex, uint32_t poll_interval_ms);

	/**
	 * The destuctor terminates the thread
	 */
	~poll_thread();

private:
	mutex_t mutex;
	uint32_t poll_interval_ms;
	b::thread thread;
	signal_t signal;

public:
	/**
	 * The "run"-function must be public for boost::thread
	 */
	void operator()();

public:
	/**
	 * Returns the signal that new slots can be added
	 * @return The boost::signals2 signal
	 */
	signal_t& get_signal() {
		return signal;
	}

};


}
