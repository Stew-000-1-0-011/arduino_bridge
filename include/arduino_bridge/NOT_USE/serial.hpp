// arduino_bridgeでのみ使われる想定で作る。
// boostのシリアル通信のための諸機能をarduino_bridgeで使う部分に絞ってラップしたようなクラスとなっている。
// クラスArduinoの中のシリアル通信に使われる部分だけを

#pragma once

#include <cstring>
#include <array>
#include <vector>
#include <optional>
#include <future>
#include <algorithm>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <CRSLib/include/utility.hpp>

#include "utility.hpp"

namespace arduino_bridge
{
	using DynamicSerialData = std::vector<u8>;
	template<size_t n>
	using StaticSerialData = std::array<u8, n>;

	namespace asio = boost::asio;

	class Serial
	{
		static constexpr unsigned int baud_rate_ = 115200;

		asio::io_context io_context_{};
		asio::executor_work_guard work_guard_{io_context_};

		// async_read, async_writeはそれぞれ同時に1つしか動かせない。
		asio::io_context::strand read_strand_{io_context_};
		asio::io_context::strand write_strand_{io_context_};

		// read, writeそれぞれに一つ。
		asio::thread_pool thread_pool_{2};

		asio::serial_port port_{io_context_};

	public:
		Serial(const std::string& port_filepath) noexcept
		{
			// set baudrate.
			port_->set_option(asio::serial_port_base::baud_rate(baud_rate_));
			// open.
			port_->open(port_filepath);

			asio::post(thread_pool_, [this]{io_context_.run()});
		}

		Serial(const Serial&) = delete;
		Serial(Serial&&) = default;

		~Serial()
		{
			port_->close();
		}

		void async_write(const DynamicSerialData& data, boost::system::error_code ec)
		{
			asio::post(write_strand_, [&data]{asio::async_write(port_, asio::buffer(data), print_error_code);});
		}

		void async_read(DynamicSerialData& data)
		{
			asio::post(read_strand_, [&data]{asio::async_read(port_, asio::buffer(data), print_error_code)});
		}

		template<size_t n>
		StaticSerialData<n> read(boost::system::error_code& ec) noexcept
		{
			StaticSerialData<n> ret{};
			asio::read(port_, asio::buffer(ret), ec);
			return ret;
		}

		DynamicSerialData read(const size_t data_size, boost::system::error_code& ec) noexcept
		{
			DynamicSerialData data(data_size, 0);
			asio::read(port_, asio::buffer(data), ec);
			return data;
		}
	};
}