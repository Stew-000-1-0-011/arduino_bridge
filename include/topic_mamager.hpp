#pragma once

#include <string>
#include <map>
#include <unordered_map>

#include <boost/thread/shared_mutex.hpp>
#include <boost/json/string_view.hpp>
#include <boost/asio.hpp>

#include <CRSLib/include/std_int.hpp>
#include <CRSLib/include/utility.hpp>

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

		// 本来は例外を使うべきなのかもしれないが、tryされなそうなのでやめた。
		// registerが予約語なので...
		bool regist(const boost::string_view topic_name, const u8 size, asio::serial_port *const port) noexcept
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
						iter->second.ports.push_back(port);
					}
				}
				else
				{
					TopicId untied_id = get_untied_id_nonlock();
					if(untied_id == TopicId::null) return false;
					name_to_data[topic_name] = {untied_id, size, {port}};
				}
			}

			return true;
		}

		TopicId get_untied_id() const noexcept
		{
			boost::shared_lock lock{mutex};
			return get_untied_id_nonlock();
		}

		bool unregist(const boost::string_view topic_name, asio::serial_port *const port) noexcept
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
						for(auto port_iter = iter->second.ports.begin(), port_iter != iter->second.ports.end(); ++port_iter)
						{
							if(*port_iter == port)
							{
								iter->second.ports.erase(port_iter);
								break;
							}
						}
					}
				}
			}
			
			return false;
		}

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
				if(iter_prev->first + 1 < iter_next->first && iter_prev->first + 1 != CRSLib::to_underlying(TopicId::null))
				{
					return static_cast<TopicId>(iter_prev->first + 1);
				}
			}

			return static_cast<TopicId>(iter_prev->first + 1);
		}
	};
}