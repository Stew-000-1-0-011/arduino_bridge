#pragma once

#include <type_traits>
#include <vector>

#include "std_int.hpp"


namespace arduino_bridge
{
	template<class Enum>
	constexpr auto to_underlying(const Enum x) noexcept -> decltype(std::enable_if_t<std::is_enum<Enum>::value, typename std::underlying_type<Enum>::type>())
	{
		return static_cast<std::underlying_type_t<Enum>>(x);
	}

	template<class Lambda, int=(Lambda{}(), 0)>
	constexpr bool is_constexpr(Lambda){return true;}
	constexpr bool is_constexpr(...) {return false;}

	

	struct TopicId final
	{
		enum type : u8
		{
			bridge_command = 0x0,
			emergency = 0x1,
			normal_id_start,
			normal_id_end = 0xFE,
			null = 0xFF
		};
	};

	enum class BridgeCommand : u8
	{
		handshake = 0x0,
		request_publisher = 0x1,
		request_subscriber = 0x2
	};

	struct TopicData
	{
		TopicId::type id;
		u8 size;
	};
}