#pragma once

#include "utility.hpp"
#include "serial.hpp"

namespace arduino_bridge
{
	class ArduinoSerial final : public Serial
	{
		std::vector<TopicId::type> topic_ids_{};

		using Serial::Serial(const std::string&);

	public:
		using Serial::Serial(Serial&&);
	
		[[nodiscard]] static std::future<std::optional<ArduinoSerial>> make(const std::string& port_filepath) noexcept
		{
			constexpr auto make_if_can_handshake = [port_filepath]() noexcept
			{
				ArduinoSerial serial{port_filepath};
				
				if(serial.handShake())
				{
					return std::move(serial);
				}
				else
				{
					return {};
				}
			};

			return std::async(std::launch::async, make_if_can_handshake);
		}

		

	private:
		// read/writeを使うため、スレッドセーフでない。
		bool handShake() noexcept
		{
			const DynamicSerialData poo_poo_cushion =
			 {
				TopicId::bridge_command,
				CRSLib::to_underlying(BridgeCommand::handshake),
				'H', 'e', 'l', 'l', 'o', 'C', 'R', 'S'
			 };

			DynamicSerialData farting_sound(poo_poo_cushion.size(), 0);
			boost::system::error_code ec;

			write(poo_poo_cushion, ec);
			if(!ec) read(farting_sound, ec);
			
			if(!ec || farting_sound != poo_poo_cushion) return false;
			else return true;
		}
	};
}