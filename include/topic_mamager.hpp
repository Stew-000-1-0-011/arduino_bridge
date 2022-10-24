#pragma once

#include <string>
#include <map>
#include <unordered_map>

#include <boost/thread/shared_mutex.hpp>
#include <boost/json/string_view.hpp>
#include <boost/asio.hpp>

#include <CRSLib/include/std_int.hpp>

namespace arduino_bridge
{
	using namespace CRSLib::IntegerTypes;
	namespace asio = boost::asio;

	enum class TopicId : u8
	{
		advertise = 0x0,
		subscribe = 0x1,
		emergency = 0x2,
		normal_id_start,
		normal_id_end = 0xFE,
		null = 0xFF
	};

	class TopicManager final
	{
		struct TopicData
		{
			const u8 id;
			const u8 size;
			std::vector<asio::serial_port *> ports;
			boost::shared_mutex mutex{};
		};


		std::unordered_map<std::string, TopicData> name_to_data{};
		std::map<u8, std::string>  id_to_name{};
		boost::shared_mutex mutex{};

	public:

		// registerが予約語なので...
		bool regist(const boost::string_view topic_name, const TopicId size, asio::serial_port *const port)
		{
			{
				boost::lock_guard lock{mutex};

				const auto iter = name_to_data.find(topic_name);
				if(iter != name_to_data.end())
				{
					if(iter->second.size != size)
					{
						return false;
					}
					else
					{
						iter->second.size += 1;
					}
				}
				else
				{
					name_to_data[topic_name] = {}
				}
			}

			return true;
		}

		TopicId get_untied_id() const noexcept
		{

		}

		void unregist(const boost::string_view topic_name, asio::serial_port *const port) noexcept
		{}

	private:
		TopicId get_untied_id_nonlock() const noexcept
		{
			if(id_to_name.empty())
			{
				return TopicId::normal_id_start;
			}

			auto iter_prev = id_to_name.cbegin();

			for(auto iter_next = id_to_name.cbegin() + 1; iter_next != id_to_name.cend(); ++iter_prev, ++iter_next)
			{
				if(iter_prev->first + 1 < iter_next->first && iter_prev->first + 1 != TopicId::null)
				{
					return static_cast<TopicId>(iter_prev->first + 1);
				}
			}

			if(iter_prev->first + 1 )
		}
	};
}