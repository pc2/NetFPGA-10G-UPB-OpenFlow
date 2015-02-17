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

#include <stddef.h>
#include <stdint.h>
#include <flow.hpp>

#define div_round_up(x,y) (x/y + (x%y?1:0))

namespace upb {

/**
 * Objects of this class hold the hardware representation of the flow entry
 */
class hw_flow {

public:
	/**
	 * Size of the tuple
	 */
	const static uint32_t hw_tuple_size_bits = 243;
	const static uint32_t hw_tuple_size_32bits = div_round_up(hw_tuple_size_bits, 32);

	typedef uint32_t hw_tuple[hw_tuple_size_32bits]; // a hardware flow has 243 bits => use 8 * uint_32 (256 bit) in software; the size of this array must ALWAYS be a multiple of 32 bits

	/**
	 * The key part of the flow entry
	 */
	hw_tuple key;
	/**
	 * The mask part of the flow entry
	 */
	hw_tuple mask; // each set bit in the mask specifies an exact match with the corresponding bit in the key ('0' = wildcard)

	/**
	 * Constructs a hardware flow from a (software) flow
	 */
	hw_flow(const flow &flow);

private:
	void convert_tuple(hw_tuple &tuple_hw, const flow::tuple &tuple_sw);

};

#undef div_round_up

}
