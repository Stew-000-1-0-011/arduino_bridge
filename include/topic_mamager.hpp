#pragma once

#include <string>
#include <unordered_map>

#include <CRSLib/include/std_int.hpp>

namespace arduino_bridge
{
	using CRSLib::IntegerTypes;

	class TopicManager final
	{
		struct TopicInfo
		{
			u8 size;
			u8 id{};
		};

		std::unordered_map<std::string, u8> topics{};


	};
}