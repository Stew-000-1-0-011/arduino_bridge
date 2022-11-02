#pragma once

#include <string>
#include <atomic>

#include <boost/asio.hpp>

#include <ros/ros.h>

#include "utility.hpp"
#include "pack_frame.hpp"

namespace arduino_bridge
{
	namespace asio = boost::asio;

	const auto print_error_code = [](const boost::system::error_code& ec, std::string msg = "")
	{
		ROS_ERROR_STREAM(msg + ec.message());
	};

	class Serial final
	{
		static constexpr unsigned int baud_rate = 9600;

		asio::io_context io{};
		asio::executor_work_guard<asio::io_context::executor_type> work_guard{io.get_executor()};

		asio::thread_pool thread_pool{2};
		std::atomic<bool> is_active{true};

	public:
		asio::serial_port port{io};

		asio::io_context::strand write_strand{io};
		asio::io_context::strand read_strand{io};

	public:
		Serial(const std::string& port_filepath) noexcept(false)
		{
			// open.
			port.open(port_filepath);

			// set baudrate.
			port.set_option(asio::serial_port_base::baud_rate(baud_rate));

			asio::post(thread_pool,
				[this]
				{
					io.run();
				}
			);
		}

		Serial(const Serial&) = delete;
		Serial& operator=(const Serial&) = delete;
		Serial(Serial&&) = delete;
		Serial& operator=(Serial&&) = delete;

		~Serial()
		{
			thread_pool.join();
			port.close();
		}

		template<class F>
		void async_receive(std::vector<u8>& buffer, F&& process_frame) noexcept
		{
			asio::post(read_strand, [this, &buffer, process_frame = std::move(process_frame)]() mutable
			{
				if(!is_active) return;

				asio::streambuf stream_buf{};
				asio::async_read_until(port, stream_buf, '\0',
					[this, &buffer, &stream_buf, process_frame = std::move(process_frame)](const boost::system::error_code& ec, const size_t) mutable
					{
						if(ec)
						{
							ROS_ERROR_STREAM("Arduino::async_receive: error occured. :" << ec.message());
							async_receive(buffer, process_frame);  // もう一度読む(次回はきっと最初から最後まで正しく読めるはず...)
						}
						else
						{
							buffer.resize(stream_buf.size());
							buffer_copy(boost::asio::buffer(buffer), stream_buf.data());
							process_frame(unpack_frame(buffer));
						}
					}
				);
			});
		}

		void async_send(const std::vector<u8>& buffer) noexcept
		{
			asio::post(write_strand, [this, &buffer]
			{
				if(!is_active) return;
				
				asio::async_write(port, asio::buffer(buffer),
					[this, &buffer](const boost::system::error_code& ec, const size_t)
					{
						if(ec)
						{
							ROS_ERROR_STREAM("Arduino::async_send: error occured. :" << ec.message());
							async_send(buffer);  // もう一度書く(次回はry)
						}
					}
				);
			});
		}

		void deactivate() noexcept
		{
			is_active = false;
		}
	};
}