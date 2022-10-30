#pragma once

#include <cstring>

#include <arduino_bridge/Frame.h>

#include "topic_mamager.hpp"

namespace arduino_bridge
{
	// Frameというメッセージに変換(非常に紛らわしい)
	inline Frame convert(const std::vector<u8>& frame) noexcept
	{
		Frame message{};

		message.topic_name = TopicManager::get_instance().get_topic_name(static_cast<TopicId::type>(frame[0]));
		message.data.resize(frame.size() - 1);
		std::memcpy(message.data.data(), frame.data() + 1, message.data.size());

		return message;
	}

	inline std::vector<u8> convert(const Frame& message) noexcept
	{
		std::vector<u8> frame(1 + message.data.size());

		frame[0] = TopicManager::get_instance().get_topic_data(message.topic_name).id;
		std::memcpy(frame.data() + 1, message.data.data(), message.data.size());

		return frame;
	}
}