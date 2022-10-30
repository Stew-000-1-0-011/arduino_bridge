#pragma once

#include <ros/ros.h>

#include <arduino_bridge/Frame.h>

#include "utility.hpp"
#include "frame_message_convertor.hpp"

namespace arduino_bridge
{
	class Ros
	{
		ros::Publisher * arduino_rx_pub;
		
	public:
		Ros(ros::Publisher *const arduino_rx_pub) noexcept:
			arduino_rx_pub{arduino_rx_pub}
		{}

		void send(const std::vector<u8>& frame) noexcept
		{

			arduino_rx_pub->publish(convert(frame));
		}
	};
}