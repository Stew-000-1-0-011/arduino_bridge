// どうすれば読み出し処理を上手く記述できたんだろうか...

#pragma once

#include <vector>
#include <string>
#include <thread>
#include <future>
// #include <optional> c++17! **(ROSへの呪詛)**!
#include <atomic>
#include <algorithm>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/optional.hpp>

#include "utility.hpp"
#include "topic_mamager.hpp"

namespace arduino_bridge
{
	namespace asio = boost::asio;

	class Arduino final
	{
	public:
		class Serial final
		{
			static constexpr unsigned int baud_rate_ = 115200;

			// nodeletが起動している必要がある。
			static void print_error_code(const boost::system::error_code& ec) noexcept
			{
				// 上手く表示されない？知るか！
				std::cerr << "arduino_bridge::Arduino::Serial: " + ec.message() << std::endl;
			}

			asio::io_context io_context_{};
			asio::executor_work_guard<decltype(io_context_.get_executor())> work_guard_{io_context_.get_executor()};

			// // async_read, async_writeはそれぞれ同時に1つしか動かせない。
			// asio::io_context::strand write_strand_{io_context_};

			// // read, writeそれぞれに一つ。
			// asio::thread_pool thread_pool_{2};

			asio::serial_port port_{io_context_};
			mutable boost::mutex write_port_mutex{};

		public:
			Serial(const std::string& port_filepath) noexcept
			{
				// set baudrate.
				port_.set_option(asio::serial_port_base::baud_rate(baud_rate_));
				// open.
				port_.open(port_filepath);

				// asio::post(thread_pool_, [this]{io_context_.run()});
				std::thread([this]{io_context_.run();}).detach();
			}

			Serial(const Serial&) = delete;
			Serial(Serial&&) = default;

			~Serial()
			{
				port_.close();
			}

			template<class F>
			void async_write(const std::vector<u8>& data, F&& completion_handler) noexcept
			{
				// asio::post(write_strand_, [&data]{asio::async_write(port_, asio::buffer(data), [](const auto& ec){print_error_code(ec);});});
				const auto&& lock = boost::make_lock_guard(write_port_mutex);
				asio::async_write(port_, asio::buffer(data),
				 [completion_handler = std::move(completion_handler)](const auto& ec, const size_t){print_error_code(ec); completion_handler(ec);}
				);
			}

			void read_byte(void *const data, const size_t data_size, boost::system::error_code& ec) noexcept
			{
				asio::read(port_, asio::buffer(data, data_size), ec);
				if(ec)
				{
					print_error_code(ec);
				}
			}

			void read_until(std::string& str, const char delimiter, boost::system::error_code& ec) noexcept
			{
				asio::read_until(port_, asio::dynamic_buffer(str), delimiter, ec);
				if(ec)
				{
					print_error_code(ec);
				}
			}

			friend class Arduino;
		} serial;

		std::atomic<bool> requested_stop{false};
		std::atomic<bool> reading_has_stoped{false};
		std::atomic<bool> writing_has_stoped{false};

		bool is_subscribed(const TopicId::type id) const noexcept
		{
			const auto&& lock = boost::shared_lock<decltype(subscribed_topic_ids_mutex)>(subscribed_topic_ids_mutex);
			const auto iter = std::find(subscribed_topic_ids.cbegin(), subscribed_topic_ids.cend(), id);
			return iter != subscribed_topic_ids.cend();
		}

		void send(const std::vector<u8>& frame) noexcept
		{
			if(!requested_stop && is_subscribed(static_cast<TopicId::type>(frame[0])))
			{
				boost::system::error_code ec;
				serial.async_write(frame, [this](const boost::system::error_code&){requested_stop = true;});
				if(ec)
				{
					requested_stop = true;
				}
			}
			// request\stopが他の処理からも変更されうることに注意
			if(requested_stop) writing_has_stoped = true;
		}

		void listen(const TopicId::type id) noexcept
		{
			const auto&& lock = boost::make_lock_guard(subscribed_topic_ids_mutex);
			subscribed_topic_ids.push_back(id);
		}

		void plug_ear(const TopicId::type id) noexcept
		{
			const auto&& lock = boost::make_lock_guard(subscribed_topic_ids_mutex);
			subscribed_topic_ids.push_back(id);
			const auto iter = std::find(subscribed_topic_ids.begin(), subscribed_topic_ids.end(), id);
			if(iter != subscribed_topic_ids.end())
			{
				subscribed_topic_ids.erase(iter);
			}
		}

		bool has_stoped() const noexcept
		{
			return reading_has_stoped && writing_has_stoped;
		}

	private:
		std::vector<TopicId::type> subscribed_topic_ids{};
		mutable boost::shared_mutex subscribed_topic_ids_mutex{};


		Arduino(const std::string& port_filepath) noexcept:
			serial{port_filepath}
		{}

		// read/writeの排他制御をしていないため、make以外からは呼び出せない。
		bool handshake() noexcept
		{
			const std::vector<u8> poo_poo_cushion =
			 {
				TopicId::bridge_command,
				to_underlying(BridgeCommand::handshake),
				'H', 'e', 'l', 'l', 'o', 'C', 'R', 'S'
			 };

			std::vector<u8> farting_sound(poo_poo_cushion.size(), 0);

			boost::system::error_code ec;
			asio::write(serial.port_, asio::buffer(poo_poo_cushion), ec);
			if(!ec) asio::read(serial.port_, asio::buffer(farting_sound), ec);
			
			if(!ec || farting_sound != poo_poo_cushion) return false;
			else return true;
		}

	public:
		template<class F>
		static std::future<boost::optional<Arduino>> make(const std::string& port_filepath) noexcept
		{
			return std::async(
			 [port_filepath]()
			 {
			 	Arduino ret{port_filepath};

			 	if(ret.handshake())
			 	{
			 		return std::move(ret);
			 	}
			 	else return {};
			 }
			);
		}
	};
}