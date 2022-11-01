/// TODO: arduino_padawansの排他制御
/// TODO: publisher, subscriberの登録解除の受理
/// TODO: その他割と雑な排他制御のブラッシュアップ
// (必要な箇所にない、不必要な箇所にある、二重ロックを考えてない、shared_lockでは不十分..など)

#pragma once

#include <vector>
#include <list>
#include <string>

#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include "utility.hpp"
#include "topic_mamager.hpp"
#include "arduino.hpp"
#include "ros.hpp"

namespace arduino_bridge
{
	class ArduinoBridge final : public nodelet::Nodelet
	{
		ros::NodeHandle nh_;
		ros::Subscriber arduino_tx_sub_;
		ros::Publisher arduino_rx_pub_;

		// マスタに繋がれてるやつら。スターウォーズより
		// どうでもいいけどスターウォーズでは弟子は一人だけしかとれないらしいので、パダワンに複数形は無いらしい。

		// // あるメモリ上の要素やイテレータはシーケンスの変更により無効となり得る。苦肉の策だがこうした。あるいはlistなどでもよかったかもしれない。
		// -> listにした。どうせメモリは連続させられないし、ランダムアクセスするわけでもないからだ。
		std::list<Arduino> arduino_padawans{};
		mutable boost::shared_mutex arduino_padawans_mutex{};
		Ros ros_padawan{&arduino_rx_pub_};

		// 流石にonInitが終わる前からrosのsubscriberやらが起動することはないだろうと踏んでコードを書いている。
		void onInit() override
		{
			nh_ = getMTNodeHandle();
			arduino_tx_sub_ = nh_.subscribe<arduino_bridge::Frame>("arduino_tx", 100, &ArduinoBridge::arduinoTxCallback, this);
			arduino_rx_pub_ = nh_.advertise<arduino_bridge::Frame>("arduino_rx", 100);
			
			const auto arduino_dirs = scanPortDirectory();
			// for(const auto& dir : arduino_dirs)
			// {
			// 	ROS_INFO("Found arduino at %s", dir.c_str());
			// }
			// ROS_INFO_STREAM(arduino_dirs.size());

			std::vector<decltype(Arduino::make(arduino_dirs[0]))> arduino_futures{};
			for(const auto& arduino_dir : arduino_dirs)
			{
				arduino_futures.emplace_back(Arduino::make(arduino_dir));
			}
			for(auto& arduino_future : arduino_futures)
			{
				auto arduino_opt = arduino_future.get();
				if(arduino_opt) arduino_padawans.emplace_back(std::move(*arduino_opt));
			}
			
			// run arduino listens.
			{
				const auto&& lock = boost::make_lock_guard(arduino_padawans_mutex);

				for(auto& arduino : arduino_padawans)
				{
					const auto arduino_listen = [this, &arduino]()
					{
						while(!arduino.requested_stop)
						{
							std::vector<u8> frame(1);

							// get topic_id
							boost::system::error_code ec{};
							arduino.serial.read_byte(frame.data(), 1, ec);
							if(ec)
							{
								arduino.requested_stop = true;
								return;
							}

							if(frame[0] == TopicId::bridge_command)
							{
								bridgeCommandReceive(arduino);
							}
							else
							{
								normalIdAndEmergencyReceive(arduino, frame);
							}
						}

						// notify reading has been stoped.
						arduino.reading_has_stoped = true;
					};

					// run.
					std::thread(arduino_listen).detach();
				}
			}
		}

		~ArduinoBridge()
		{
			{
				const auto&& lock = boost::make_lock_guard(arduino_padawans_mutex);
				for(auto& arduino : arduino_padawans)
				{
					arduino.requested_stop = true;
				}
			}

			/// TODO: これほんとにデッドロックしないんか...？
			{
				const auto&& lock = boost::shared_lock<decltype(arduino_padawans_mutex)>(arduino_padawans_mutex);
				for(const auto& arduino : arduino_padawans)
				{
					while(!arduino.has_stoped());
				}
			}
		}

	private:
		//callback function for arduino_tx topic
		void arduinoTxCallback(const arduino_bridge::Frame::ConstPtr& msg)
		{
			NODELET_INFO("arduino_tx callback");
			const auto&& lock = boost::make_lock_guard(arduino_padawans_mutex);
			for(auto& other : arduino_padawans)
			{
				other.send(convert(*msg));
			}
		}

		static std::vector<std::string> scanPortDirectory() noexcept
		{
			std::vector<std::string> ports;

			auto begin = boost::filesystem::directory_iterator(boost::filesystem::absolute("/dev"));
			auto end = boost::filesystem::directory_iterator();
			for(const auto& directory : boost::make_iterator_range(begin, end)){
				if(directory.path().filename().string().substr(0,3) == "tty"){
					ports.push_back(directory.path().string());
				}
			}

			return ports;
		}

		void bridgeCommandReceive(Arduino& arduino) noexcept
		{
			BridgeCommand command;
			boost::system::error_code ec;
			arduino.serial.read_byte(&command, 1, ec);
			if(ec)
			{
				arduino.requested_stop = true;
			}

			switch(command)
			{
			case BridgeCommand::request_publisher:
				requestedPublisher(arduino);
				break;

			case BridgeCommand::request_subscriber:
				requestedSubscriber(arduino);
				break;

			case BridgeCommand::handshake:
				NODELET_ERROR("handshake message is reseived at unexpected moment.");
				break;

			default:
				NODELET_ERROR("unknown bridge command is reseived.");
				break;
			}
		}

		void requestedPublisher(Arduino& arduino) noexcept
		{
			requestedPublisherSubscriberInner(arduino, true);
		}

		void requestedSubscriber(Arduino& arduino) noexcept
		{
			requestedPublisherSubscriberInner(arduino, false);
		}

		void requestedPublisherSubscriberInner(Arduino& arduino, const bool is_pub) noexcept
		{
			std::string topic_name{};
			boost::system::error_code ec;
			arduino.serial.read_until(topic_name, '\n', ec);
			if(ec)
			{
				arduino.requested_stop = true;
			}

			// 改行を消す.
			topic_name.pop_back();

			u8 topic_size;
			arduino.serial.read_byte(&topic_size, 1, ec);
			if(ec)
			{
				arduino.requested_stop = true;
			}

			if(!TopicManager::get_instance().regist(topic_name, topic_size))
			{
				NODELET_ERROR("arduino_bridge::ArduinoBridge: fail to register a topic.");
				return;
			}

			TopicId::type id = TopicManager::get_instance().get_topic_data(topic_name).id;
			if(!is_pub) arduino.listen(id);
			
			// async_writeのgather操作が無駄に...
			std::vector<u8> frame(2 + topic_name.size() + 2);
			frame[0] = TopicId::bridge_command;
			frame[1] = to_underlying(is_pub ? BridgeCommand::request_publisher : BridgeCommand::request_subscriber);
			for(size_t i = 0; i < topic_name.size(); ++i)
			{
				frame[2 + i] = topic_name[i];
			}
			frame[2 + topic_name.size()] = '\n';
			frame[2 + topic_name.size() + 1] = id;

			arduino.send(frame);
		}

		void normalIdAndEmergencyReceive(Arduino& arduino, std::vector<u8>& frame) noexcept
		{
			// make frame
			const size_t topic_size = TopicManager::get_instance().get_topic_size(static_cast<TopicId::type>(frame[0]));
			frame.resize(topic_size);

			boost::system::error_code ec;
			arduino.serial.read_byte(frame.data() + 1, topic_size, ec);
			if(ec)
			{
				arduino.requested_stop = true;
			}

			// send frame
			{
				const auto&& lock = boost::make_lock_guard(arduino_padawans_mutex);
				for(auto& other : arduino_padawans)
				{
					other.send(frame);
				}
			}
			ros_padawan.send(frame);
		}
	};
}