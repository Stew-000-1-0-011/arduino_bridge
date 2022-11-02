/// TODO: arduinosの排他制御
/// TODO: publisher, subscriberの登録解除の受理
/// TODO: その他割と雑な排他制御のブラッシュアップ
// (必要な箇所にない、不必要な箇所にある、二重ロックを考えてない、shared_lockでは不十分..など)

#pragma once

#include <vector>
#include <list>
#include <mutex>

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
		ros::Subscriber arduino_tx_sub;
		ros::Publisher arduino_rx_pub;

		// // あるメモリ上の要素やイテレータはシーケンスの変更により無効となり得る。苦肉の策だがこうした。あるいはlistなどでもよかったかもしれない。
		// -> listにした。どうせメモリは連続させられないし、ランダムアクセスするわけでもないからだ。
		std::list<Arduino> arduinos{};
		mutable std::mutex arduinos_mutex{};
		Ros ros_ros{&arduino_rx_pub};
		asio::thread_pool arduino_listens_pool{};

		// 流石にonInitが終わる前からrosのsubscriberやらが起動することはないだろうと踏んでコードを書いている。
		void onInit() override
		{
			auto nh = getNodeHandle();
			arduino_tx_sub = nh.subscribe<arduino_bridge::Frame>("arduino_tx", 100, &ArduinoBridge::arduinoTxCallback, this);
			arduino_rx_pub = nh.advertise<arduino_bridge::Frame>("arduino_rx", 100);
			
			const auto arduino_dirs = scanPortDirectory();

			std::vector<std::future<boost::optional<Arduino>>> arduino_futures{};
			for(const auto& arduino_dir : arduino_dirs)
			{
				arduino_futures.emplace_back(Arduino::make(arduino_dir));
			}

			for(auto& arduino_future : arduino_futures)
			{
				auto arduino_opt = arduino_future.get();
				if(arduino_opt) arduinos.emplace_back(std::move(*arduino_opt));
			}

			// run arduino listens.
			{
				const auto&& lock = std::lock_guard<decltype(arduinos_mutex)>(arduinos_mutex);

				for(auto& arduino : arduinos)
				{
					const auto arduino_listen = [this, &arduino]()
					{
						std::vector<u8> buffer{};
						recursive_receive(arduino, buffer);
					};

					// run.
					asio::post(arduino_listens_pool, arduino_listen);
				}
			}
		}

		~ArduinoBridge()
		{
			arduino_listens_pool.join();
		}

	private:
		void recursive_receive(Arduino& arduino, std::vector<u8>& buffer) noexcept
		{
			arduino.serial->async_receive(buffer,
				[this, &arduino, &buffer](const std::vector<u8>& rx_frame)
				{
					processFrame(arduino, rx_frame);
					recursive_receive(arduino, buffer);
				}
			);
		}

		//callback function for arduino_tx topic
		void arduinoTxCallback(const arduino_bridge::Frame::ConstPtr& msg)
		{
			try
			{
				const auto frame = convert(*msg);

				const auto&& lock = std::lock_guard<decltype(arduinos_mutex)>(arduinos_mutex);
				for(auto& arduino : arduinos)
				{
					arduino.serial->async_send(frame);
				}
			}
			catch(...)
			{
				NODELET_ERROR_STREAM("arduino_bridge::ArduinoBridge: Unknown Topic: " << msg->topic_name);
			}
		}

		void processFrame(Arduino& arduino, const std::vector<u8>& frame)
		{
			if(frame[0] == to_underlying(TopicId::bridge_command))
			{
				bridgeCommandReceive(arduino, frame);
			}
			else
			{
				normalIdAndEmergencyReceive(frame);
			}
		}

		void bridgeCommandReceive(Arduino& arduino, const std::vector<u8>& frame) noexcept
		{
			switch(static_cast<BridgeCommand>(frame[1]))
			{
			case BridgeCommand::request_publisher:
				requestedPublisher(arduino, frame);
				break;

			case BridgeCommand::request_subscriber:
				requestedSubscriber(arduino, frame);
				break;

			case BridgeCommand::handshake:
				NODELET_ERROR("handshake message is reseived at unexpected moment.");
				break;

			default:
				NODELET_ERROR("unknown bridge command is reseived.");
				break;
			}
		}

		void requestedPublisher(Arduino& arduino, const std::vector<u8>& frame) noexcept
		{
			requestedPublisherSubscriberInner(arduino, frame, true);
		}

		void requestedSubscriber(Arduino& arduino, const std::vector<u8>& frame) noexcept
		{
			requestedPublisherSubscriberInner(arduino, frame, false);
		}

		void requestedPublisherSubscriberInner(Arduino& arduino, const std::vector<u8>& frame, const bool is_pub) noexcept
		{
			std::string topic_name{};
			for(size_t i = 0; frame[i + 2] != '\n'; ++i)
			{
				topic_name.push_back(frame[i + 2]);
			}

			u8 topic_size = frame.back();

			if(!TopicManager::get_instance().regist(topic_name, topic_size))
			{
				NODELET_ERROR("arduino_bridge::ArduinoBridge: fail to register a topic.");
				return;
			}

			TopicId::type id = TopicManager::get_instance().get_topic_data(topic_name).id;
			if(!is_pub) arduino.listen(id);
			
			// async_writeのgather操作が無駄に...
			std::vector<u8> tx_frame(2 + topic_name.size() + 2);
			tx_frame[0] = TopicId::bridge_command;
			tx_frame[1] = to_underlying(is_pub ? BridgeCommand::request_publisher : BridgeCommand::request_subscriber);
			for(size_t i = 0; i < topic_name.size(); ++i)
			{
				tx_frame[2 + i] = topic_name[i];
			}
			tx_frame[2 + topic_name.size()] = '\n';
			tx_frame[2 + topic_name.size() + 1] = id;

			arduino.serial->async_send(tx_frame);
		}

		void normalIdAndEmergencyReceive(const std::vector<u8>& frame) noexcept
		{
			// send frame
			{
				const auto&& lock = std::lock_guard<decltype(arduinos_mutex)>(arduinos_mutex);
				for(auto& other : arduinos)
				{
					other.serial->async_send(frame);
				}
			}
			ros_ros.send(frame);
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
	};
}