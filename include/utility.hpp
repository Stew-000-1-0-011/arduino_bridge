#pragma once

#include <vector>

#include <CRSLib/include/std_int.hpp>

namespace arduino_bridge
{
	using namespace CRSLib::IntegerTypes;

	enum class TopicId : u8
	{
		bridge_command = 0x0,
		emergency = 0x1,
		normal_id_start,
		normal_id_end = 0xFE,
		null = 0xFF
	};

	enum class BridgeCommand : u8
	{
		handshake = 0x0,
		request_publisher = 0x1,
		request_subscriber = 0x2
	};

	struct TopicData
	{
		TopicId id;
		u8 size;
	};
}