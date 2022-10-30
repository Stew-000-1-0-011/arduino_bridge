#pragma once

#include <vector>
#include <thread>
#include <future>
#include <optional>
#include <atomic>
#include <algorithm>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>

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
				NODELET_ERROR_STREAM("arduino_bridge::Arduino::Serial: " << ec.message());
			}

			asio::io_context io_context_{};
			asio::executor_work_guard work_guard_{io_context_};

			// // async_read, async_writeはそれぞれ同時に1つしか動かせない。
			// asio::io_context::strand write_strand_{io_context_};

			// // read, writeそれぞれに一つ。
			// asio::thread_pool thread_pool_{2};

			asio::serial_port port_{io_context_};
			boost::mutex write_port_mutex{};

		public:
			Serial(const boost::string_view port_filepath) noexcept
			{
				// set baudrate.
				port_->set_option(asio::serial_port_base::baud_rate(baud_rate_));
				// open.
				port_->open(port_filepath);

				// asio::post(thread_pool_, [this]{io_context_.run()});
				std::thread([this]{io_context_.run();}).detach();
			}

			Serial(const Serial&) = delete;
			Serial(Serial&&) = default;

			~Serial()
			{
				port_->close();
			}

			void async_write(const std::vector<u8>& data, boost::system::error_code ec) noexcept
			{
				// asio::post(write_strand_, [&data]{asio::async_write(port_, asio::buffer(data), [](const auto& ec){print_error_code(ec);});});
				boost::lock_guard lock{write_port_mutex};
				asio::async_write(port_, asio::buffer(data), [](const auto& ec){print_error_code(ec);});
			}

			std::vector<u8> read_bytes(const size_t data_size, boost::system::error_code& ec) noexcept
			{
				std::vector<u8> ret(data_size, 0);
				asio::read(port_, asio::buffer(ret), ec);
				if(ec)
				{
					print_error_code(ec);
				}
				return ret;
			}

			u8 read_1byte(boost::system::error_code& ec) noexcept
			{
				std::array<u8, 1> arr{};
				asio::read(port_, asio::buffer(arr), ec);
				if(ec)
				{
					print_error_code(ec);
				}
				return arr[0];
			}
		} serial;

		std::atomic<bool> requested_stop{false};
		std::atomic<bool> has_stoped{false};

		bool is_subscribed(const TopicId id) const noexcept
		{
			boost::shared_lock_guard lock{subscribed_topic_ids_mutex};
			const auto iter = std::find(subscribed_topic_ids.cbegin(), subscribed_topic_ids.cend(), id);
			return iter != subscribed_topic_ids.cend();
		}

		void send(const std::vector<u8>& frame) noexcept
		{
			if(is_subscribed(static_cast<TopicId>(frame[0])))
			{
				serial.async_write(frame);
			}
		}

		void listen(const TopicId id) noexcept
		{
			boost::lock_guard lock{subscribed_topic_ids_mutex};
			subscribed_topic_ids.push_back(id);
		}

		void plug_ear(const TopicId id) noexcept
		{
			boost::lock_guard lock{subscribed_topic_ids_mutex};
			subscribed_topic_ids.push_back(id);
			const auto iter = std::find(subscribed_topic_ids.begin(), subscribed_topic_ids.end(), id);
			if(iter != subscribed_topic_ids.end())
			{
				subscribed_topic_ids.erase(iter);
			}
		}

	private:
		std::vector<TopicId> subscribed_topic_ids{};
		mutable boost::shared_mutex subscribed_topic_ids_mutex{};


		Arduino(const boost::string_view port_filepath) noexcept:
			serial{port_filepath}
		{}

		bool handshake() noexcept
		{
			const std::vector<u8> poo_poo_cushion =
			 {
				CRSLib::to_underlying(TopicId::bridge_command),
				CRSLib::to_underlying(BridgeCommand::handshake),
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
		static std::future<std::optional<Arduino>> make(const boost::string_view port_filepath) noexcept
		{
			Arduino ret{port_filepath};

			if(ret.handshake())
			{
				return std::move(ret);
			}
			else return {};
		}
	};
}