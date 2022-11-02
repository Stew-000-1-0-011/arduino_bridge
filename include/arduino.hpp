#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <algorithm>
#include <future>

#include <boost/optional.hpp>

#include "utility.hpp"
#include "serial.hpp"
#include "pack_frame.hpp"

namespace arduino_bridge
{
	class Arduino final
	{
		std::vector<TopicId::type> subscribed_topic_ids{};
		mutable std::mutex subscribed_topic_ids_mutex{};

	public:
		std::unique_ptr<Serial> serial;

		Arduino(Arduino&& obj) noexcept:
			subscribed_topic_ids{std::move(subscribed_topic_ids)},
			serial{std::move(obj.serial)}
		{}

		~Arduino()
		{
			serial->deactivate();
		}

	private:
		Arduino(const std::string& port_filepath) noexcept(false):
			serial{std::make_unique<Serial>(port_filepath)}
		{}

	public:
		bool is_subscribed(const TopicId::type id) const noexcept
		{
			const auto&& lock = std::lock_guard<decltype(subscribed_topic_ids_mutex)>(subscribed_topic_ids_mutex);
			const auto iter = std::find(subscribed_topic_ids.cbegin(), subscribed_topic_ids.cend(), id);
			return iter != subscribed_topic_ids.cend();
		}

		void listen(const TopicId::type id) noexcept
		{
			const auto&& lock = std::lock_guard<decltype(subscribed_topic_ids_mutex)>(subscribed_topic_ids_mutex);
			subscribed_topic_ids.push_back(id);
		}

		void plug_ear(const TopicId::type id) noexcept
		{
			const auto&& lock = std::lock_guard<decltype(subscribed_topic_ids_mutex)>(subscribed_topic_ids_mutex);
			subscribed_topic_ids.push_back(id);
			const auto iter = std::find(subscribed_topic_ids.begin(), subscribed_topic_ids.end(), id);
			if(iter != subscribed_topic_ids.end())
			{
				subscribed_topic_ids.erase(iter);
			}
		}

		static std::future<boost::optional<Arduino>> make(const std::string& port_filepath) noexcept
		{
			return std::async(
				[port_filepath]() -> boost::optional<Arduino>
				{
					try
					{
						Arduino ret{port_filepath};
						if(ret.handshake())
						{
							return ret;
						}
						else return boost::none;
					}
					catch(boost::system::system_error&)
					{
						return boost::none;
					}
				}
			);
		}

	private:
		bool handshake() noexcept
		{
			std::promise<bool> prom{};
			auto futr = prom.get_future();

			const std::vector<u8> poo_poo_cushion =
			{
				to_underlying(TopicId::bridge_command),
				to_underlying(BridgeCommand::handshake),
				'H','e','l','l','o','C','R','S'
			};

			std::vector<u8> fart_sound{};

			// ここ、他にこのArduinoからasync_receiveしているやつがいるとそいつにhandshakeの受信フレームを消費されてどうにもならない。
			// だから呼び出しは気を付けること。というか接続復帰にも使わねぇしprivateでいいんじゃねえかな。いいか。
			serial->async_receive(fart_sound,
			[poo_poo_cushion, prom = std::move(prom)](const std::vector<u8>& received_frame) mutable
			{
				prom.set_value(received_frame == poo_poo_cushion);
			});
			
			if(futr.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
			{
				return false;
			}
			else
			{
				return futr.get();
			}
		}
	};
}