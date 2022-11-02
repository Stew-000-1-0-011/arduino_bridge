#pragma once

#include <vector>

#include "utility.hpp"

namespace arduino_bridge
{
	inline std::vector<u8> pack_frame(const std::vector<u8>& frame) noexcept
	{
		std::vector<u8> ret(frame.size() + 2, 0);
		
		u8 * previous_zero_element = &ret[0];
		/// TODO: 1始まり？0？
		size_t count = 0;
		for(size_t i = 0; i < frame.size(); ++i)
		{
			if(frame[i] != 0)
			{
				ret[i + 1] = frame[i];
				++count;
			}
			else
			{
				*previous_zero_element = count;
				previous_zero_element = &ret[i + 1];
				count = 0;
			}
		}

		return ret;
	}

	inline std::vector<u8> unpack_frame(const std::vector<u8>& cobsed_frame) noexcept
	{
		std::vector<u8> ret(cobsed_frame.size() - 2);
		
		size_t next_zero = cobsed_frame[0];
		for(size_t i = 0; i < ret.size(); ++i)
		{
			if(next_zero == 0)
			{
				next_zero = cobsed_frame[i + 1];
				ret[i] = 0;
			}
			else
			{
				ret[i] = cobsed_frame[i + 1];
				--next_zero;
			}
		}

		return ret;
	}
}