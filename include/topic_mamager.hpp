#pragma once

#include <string>
#include <map>
#include <unordered_map>

#include <boost/thread/shared_mutex.hpp>
#include <boost/json/string_view.hpp>
#include <boost/asio.hpp>

#include "utility.hpp"

namespace arduino_bridge
{
	class TopicManager final
	{
		std::unordered_map<std::string, TopicData> name_to_data{};
		std::map<TopicId, std::string> id_to_name{};
		boost::shared_mutex mutex{};

	public:
		TopicManager() = default;
		TopicManager(const TopicManager&) = delete;
		TopicManager& operator=(const TopicManager&) = delete;
		TopicManager(TopicManager&) = delete;
		TopicManager& operator=(TopicManager&&) = delete;

		// 本来は例外を使うべきなのかもしれないが、tryされなそうなのでやめた。
		// registerが予約語なので...
		bool regist(const boost::string_view topic_name, const u8 size) noexcept
		{
			{
				boost::lock_guard lock{mutex};

				const auto iter = name_to_data.find(topic_name);
				if(iter != name_to_data.end())
				{
					if(iter->second.size != size) return false;
					else return true;
				}
				else
				{
					TopicId untied_id = get_untied_id_nonlock();
					if(untied_id == TopicId::null)
					{
						return false;
					}
					else
					{
						name_to_data[topic_name] = {untied_id, size};
						id_to_name[untied_id] = topic_name;
						return true;
					}
				}
			}
		}

		bool unregist(const boost::string_view topic_name) noexcept
		{
			{
				boost::lock_guard lock{mutex};

				const auto iter = name_to_data.find(topic_name);
				if(iter != name_to_data.end())
				{
					name_to_data.erase(iter);
					return true;
				}
				else return false;
			}
		}

		boost::string_view get_topic_name(const TopicId id)
		{
			boost::shared_lock_guard lock{mutex};
			return id_to_name.at(id);
		}

		TopicData get_topic_data(boost::string_view topic_name)
		{
			boost::shared_lock_guard lock{mutex};
			return name_to_data.at(topic_name);
		}

		static TopicManager& get_instance() noexcept
		{
			static TopicManager instance{};
			return instance;
		}


	private:
		TopicId get_untied_id() const noexcept
		{
			boost::shared_lock_guard lock{mutex};
			return get_untied_id_nonlock();
		}

		TopicId get_untied_id_nonlock() const noexcept
		{
			if(id_to_name.empty())
			{
				return TopicId::normal_id_start;
			}

			auto iter_prev = id_to_name.cbegin();

			for(auto iter_next = id_to_name.cbegin() + 1; iter_next != id_to_name.cend(); ++iter_prev, ++iter_next)
			{
				if(iter_prev->first + 1 < iter_next->first && CRSLib::to_underlying(iter_prev->first) + 1 != CRSLib::to_underlying(TopicId::null))
				{
					return static_cast<TopicId>(iter_prev->first + 1);
				}
			}

			return static_cast<TopicId>(iter_prev->first + 1);
		}
	};
}