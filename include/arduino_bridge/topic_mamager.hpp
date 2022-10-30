#pragma once

#include <string>
#include <map>
#include <unordered_map>
#include <utility>

#include <boost/thread/shared_mutex.hpp>
#include <boost/asio.hpp>

#include "utility.hpp"

namespace arduino_bridge
{
	class TopicManager final
	{
		std::unordered_map<std::string, TopicData> name_to_data{std::make_pair(std::string{"Emergency"}, TopicData{TopicId::emergency, 0}), std::make_pair(std::string{"Null"}, TopicData{TopicId::null, 0})};
		std::map<TopicId::type, std::string> id_to_name{std::make_pair(TopicId::emergency, std::string{"Emergency"}), std::make_pair(TopicId::null, std::string{"Null"})};
		mutable boost::shared_mutex mutex{};

	public:
		TopicManager() = default;
		TopicManager(const TopicManager&) = delete;
		TopicManager& operator=(const TopicManager&) = delete;
		TopicManager(TopicManager&) = delete;
		TopicManager& operator=(TopicManager&&) = delete;

		// 本来は例外を使うべきなのかもしれないが、tryされなそうなのでやめた。
		// registerが予約語なので...
		bool regist(const std::string& topic_name, const u8 size) noexcept
		{
			{
				const auto&& lock = boost::make_lock_guard(mutex);

				const auto iter = name_to_data.find(topic_name);
				if(iter != name_to_data.end())
				{
					if(iter->second.size != size) return false;
					else return true;
				}
				else
				{
					TopicId::type untied_id = get_untied_id_nonlock();
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

		bool unregist(const std::string& topic_name) noexcept
		{
			{
				const auto&& lock = boost::make_lock_guard(mutex);

				const auto iter = name_to_data.find(topic_name);
				if(iter != name_to_data.end())
				{
					name_to_data.erase(iter);
					return true;
				}
				else return false;
			}
		}

		std::string& get_topic_name(const TopicId::type id) noexcept
		{
			const auto&& lock = boost::shared_lock<decltype(mutex)>(mutex);
			return id_to_name.at(id);
		}

		u8 get_topic_size(const TopicId::type id)
		{
			const auto&& lock = boost::shared_lock<decltype(mutex)>(mutex);
			return name_to_data.at(id_to_name.at(id)).size;
		}

		TopicData get_topic_data(const std::string& topic_name)
		{
			const auto&& lock = boost::shared_lock<decltype(mutex)>(mutex);
			return name_to_data.at(topic_name);
		}

		static TopicManager& get_instance() noexcept
		{
			static TopicManager instance{};
			return instance;
		}


	private:
		TopicId::type get_untied_id() const noexcept
		{
			const auto&& lock = boost::shared_lock<decltype(mutex)>(mutex);
			return get_untied_id_nonlock();
		}

		TopicId::type get_untied_id_nonlock() const noexcept
		{
			if(id_to_name.empty())
			{
				return TopicId::normal_id_start;
			}

			auto iter_prev = id_to_name.cbegin();
			auto iter_next = id_to_name.cbegin();
			++iter_next;
			for(; iter_next != id_to_name.cend(); ++iter_prev, ++iter_next)
			{
				if(iter_prev->first + 1 < iter_next->first && iter_prev->first + 1 != TopicId::null)
				{
					return static_cast<TopicId::type>(iter_prev->first + 1);
				}
			}

			return static_cast<TopicId::type>(iter_prev->first + 1);
		}
	};
}