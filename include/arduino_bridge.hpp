#pragma once

#include <vector>

#include <ros/ros.h>

#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include "arduino.hpp"
#include "ros.hpp"

namespace arduino_bridge
{
	class ArduinoBridge final : nodelet::Nodelet
	{
		ros::NodeHandle nh_;
		ros::Subscriber arduino_tx_sub_;
		ros::Publisher arduino_rx_pub_;

		// マスタに繋がれてるやつら。スターウォーズより
		// どうでもいいけどスターウォーズでは弟子は一人だけしかとれないらしいので、パダワンに複数形は無いらしい。
		std::vector<Arduino> arduino_padawans{};
		Ros ros_padawan{&arduino_rx_pub_};

		void onInit() override
		{
			nh_ = getMTNodeHandle();
			arduino_tx_sub_ = nh_.subscribe<arduino_bridge::Frame>("arduino_tx", 100, &ArduinoBridge::arduinoTxCallback, this);
			arduino_rx_pub_ = nh_.advertise<arduino_bridge::Frame>("arduino_rx", 100);

			for(auto& arduino : arduino_padawans)
			{
				constexpr auto arduino_listen = [&arduino]()
				{
					while(!arduino.requested_stop)
					{
						boost::system::error_code ec{};

						TopicId id = static_cast<TopicId>(arduino.serial.read_1byte(ec));
						if(ec)
						{
							return;
						}
						
						
					}
					arduino.has_stoped = true;
				};

				std::thread([](){}).detach();
			}
		}

	private:
		static std::vector<std::string> scanPortDirectory() noexcept
		{
			std::vector<std::string> ports;

			auto begin = filesystem::directory_iterator(filesystem::absolute("/dev"));
			auto end = filesystem::directory_iterator();
			for(const auto& directory : boost::make_iterator_range(begin, end)){
				if(!filesystem::is_symlink(directory.status()) && filesystem::is_regular_file(directory.status())){
					if(directory.path().filename().string().substr(0,3) == "tty" || directory.path().extension() == ""){
						ports.push_back(directory.path().string());
					}
				}
			}

			return ports;
		}

		void bridgeCommandCallback(const BridgeCommand command)
		{
			switch(command)
			{
			case BridgeCommand::request_publisher:
				requestedPublisher();
				break;

			case BridgeCommand::request_subscriber:
				requestedSubscriber();
				break;

			case BridgeCommand::handshake:
				NODELET_ERROR("handshake message is reseived at unexpected moment.");
				break;

			default:
				NODELET_ERROR("unknown bridge command is reseived.");
				break;
			}
		}

		void normalIdAndEmergencyCallback(const Frame& data)
		{

		}

		//callback function for arduino_tx topic
		void arduinoTxCallback(const arduino_bridge::Frame::ConstPtr& msg){
			//TODO : write data to serial port.
			//search the topic id in arduino_serials_.
			for(const auto& arduino_serial : arduino_serials_){
				for(const auto& topic_id : arduino_serial.topic_id_){
					if(msg->data[0] == topic_id){
						//if the topic id is matched, write the data to serial port.
						boost::asio::async_write(arduino_serial.port_, msg->data, boost::bind(&ArduinoBridge::writeHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
					}
				}
			}
		}
	};
}

PLUGINLIB_EXPORT_CLASS(arduino_bridge::ArduinoBridge, nodelet::Nodelet);
