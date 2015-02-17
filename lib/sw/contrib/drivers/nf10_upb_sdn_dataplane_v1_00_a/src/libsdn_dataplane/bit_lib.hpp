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

#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

namespace upb {

	/**
	 * Copies dst_end-dst_begin+1 bits from the source to the destination's bit area [dst_begin,dst_end].
	 * The amount of the bits copied must be 32 bits at most.
	 * Source and destination data structures sizes must be multiples of 32 bits.
	 * @param dst Reference to the destination data structure
	 * @param dst_begin First bit pos of the destination structure. Range: [0,63]
	 * @param dst_end Last bit pos of the destination structure. Range: [0,63]
	 * @param src Reference to the source data structure
	 */
	template <class D, class S>
	inline void transfer_max_32_bits(D &dst, uint32_t dst_begin, uint32_t dst_end, const S &src)
	{
		assert(dst_end >= dst_begin);
		assert(((dst_end >> 5) + 1) <= sizeof(D) * 4); // the last bit must lie in a valid 32 bit word
		assert((((dst_end - dst_begin) >> 5) + 1) <= sizeof(S) * 4); // the last bit must lie in a valid 32 bit word
		assert(dst_end <= 63);

		uint32_t size_bits = dst_end - dst_begin + 1;
		assert(size_bits <= 32);

		const uint32_t *src_p = (const uint32_t *)&src;

		if (dst_end > 31) { // access is beyond the 32 bit border => use 64 bit access

			uint64_t *dst_p = (uint64_t *)&dst;

			// "mask" marks all bits that will be overwritten in the destination
			uint64_t mask = (uint64_t)-1 >> (63 - dst_end);
			mask &= (uint64_t)-1 << dst_begin;

			*dst_p &= ~mask; // turn off bits
			*dst_p |= mask & ((uint64_t)*src_p << dst_begin); // set bit range

		} else { // access is within the 32 bit border (dst_end <= 31, dst_begin <= 31)

			uint32_t *dst_p = (uint32_t *)&dst;

			// "mask" marks all bits that will be overwritten in the destination
			uint32_t mask = (uint32_t)-1 >> (31 - dst_end);
			mask &= (uint32_t)-1 << dst_begin;

			*dst_p &= ~mask; // turn off bits
			*dst_p |= mask & ((uint32_t)*src_p << dst_begin); // set bit range

		}
	}

	/**
	 * Copies dst_end-dst_begin+1 bits from the source to the destination's bit area [dst_begin,dst_end].
	 * Source and destination data structures sizes must be multiples of 32 bits.
	 * @param dst Reference to the destination data structure.
	 * @param dst_begin First bit pos of the destination structure
	 * @param dst_end Last bit pos of the destination structure
	 * @param src Reference to the source data structure
	 */
	template <class D, class S>
	inline void transfer_bits(D &dst, uint32_t dst_begin, uint32_t dst_end, const S &src)
	{
		assert(dst_end >= dst_begin);
		assert(((dst_end >> 5) + 1) <= sizeof(D) * 4); // the last bit must lie in a valid 32 bit word
		assert((((dst_end - dst_begin) >> 5) + 1) <= sizeof(S) * 4); // the last bit must lie in a valid 32 bit word

		const uint32_t *src_p = (const uint32_t *)&src;
		uint32_t *dst_p = (uint32_t *)&dst;

		for (uint32_t block32 = dst_begin; block32 <= dst_end; block32 += 32) {

			transfer_max_32_bits<>(
				*(uint64_t*)(dst_p + (block32 >> 5)),
				dst_begin & 31,
				(dst_begin & 31) + std::min(dst_end - block32, 31U),
				*src_p
			);

			src_p++;
		}
	}

	/**
	 * Extracts src_end-src_begin+1 bits from src
	 * The amount of the bits copied must be 32 bits at most.
	 * Source data structure's size must be multiples of 32 bits.
	 * @param src Reference to the source data structure
	 * @param src_begin First bit pos of the source data structure
	 * @param src_end Last bit pos of the source data structure
	 * @return The extracted value
	 */
	template <class S>
	inline uint32_t get_bits(const S &src, uint32_t src_begin, uint32_t src_end)
	{
		assert(src_end >= src_begin);
		assert(((src_end >> 5) + 1) <= sizeof(S) * 4); // the last bit must lie in a valid 32 bit word
		assert(src_end - src_begin < 32); // only 32 bits are allowed

		const uint32_t *src_p = (const uint32_t *)&src;

		uint32_t src_begin_32bitpos = src_begin >> 5;
		uint32_t src_end_32bitpos = src_end >> 5;
		uint32_t src_begin_bitpos = src_begin & 31;
		uint32_t src_end_bitpos = src_end & 31;

		if (src_begin_32bitpos == src_end_32bitpos) { // src_begin and src_end lie within the same 32 bit word

			uint32_t value;
			value = *(src_p + src_begin_32bitpos);
			value &= (uint32_t)-1 >> (31 - src_end_bitpos); // mask unused upper bits
			value >>= src_begin_bitpos; // shift data to bit position 0

			return value;

		} else {

			uint64_t value;
			value = *((uint64_t*)(src_p + src_begin_32bitpos)); // get a 64 bit value
			value &= (uint64_t)-1 >> (31 - src_end_bitpos); // mask unused upper bits
			value >>= src_begin_bitpos; // shift data to bit position 0

			return (uint32_t)value;
		}
	}

}
