#pragma once

#include <vector>

#include <CRSLib/include/std_int.hpp>

namespace arduino_bridge
{
	using namespace CRSLib::IntegerTypes;
	namespace asio = boost::asio;

	enum class TopicId : u8
	{
		for_bridge = 0x0,
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

	using DataField = std::vector<u8>;
	using Frame = std::vector<u8>;
}