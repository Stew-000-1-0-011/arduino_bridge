#pragma once

#include <cstring>
#include <array>
#include <optional>
#include <future>

#include <boost/utility/string_view.hpp>
#include <boost/asio.hpp>

#include <CRSLib/include/utility.hpp>

#include "utility.hpp"

namespace arduino_bridge
{
    struct TopicId
    {};

    class ArduinoSerial{
        static constexpr unsigned int baud_rate_ = 115200;

        asio::io_context io_context_{};
        asio::executor_work_guard work_guard_{io_context_};

        // async_read, async_writeはそれぞれ同時に1つしか動かせない。
        asio::io_context::strand read_strand_{io_context_};
        asio::io_context::strand write_strand_{io_context_};

        // read, writeそれぞれに一つ。
        asio::thread_pool thread_pool_{2};

        asio::serial_port port_{io_context_};
        std::vector<TopicId> topic_id_{};
        
    public:
        ArduinoSerial(const boost::string_view port_filepath) noexcept{
            // set baudrate.
            port_->set_option(asio::serial_port_base::baud_rate(baud_rate_));
            // open.
            port_->open(port_filepath);

            asio::post(thread_pool_, [this]{io_context_.run()});
        }

        ~ArduinoSerial(){
            port_->close();
        }

        // でーたをあるどぅいーのにやる。
        void async_afon(const Frame& tx_frame){
            asio::post(write_strand_, [&tx_frame]{asio::async_write(port_, asio::buffer(tx_frame));});
        }

        // でーたをあるどぅいーのよりうく。
        void async_asendan(Frame& rx_frame){
            asio::post(read_strand_, [&rx_frame]{asio::async_read(port_, asio::buffer(rx_frame))});
        }

    private:
        // 全ての通信の前に行うこと。それ以外の場合動作は保証されない
        bool handShake() noexcept{

            const Frame poo_poo_cushion =
             {
                CRSLib::to_underlying(TopicId::for_bridge),
                CRSLib::to_underlying(BridgeCommand::handshake),
                'H', 'e', 'l', 'l', 'o', 'C', 'R', 'S'
             };

            Frame farting_sound(10);
            boost::system::error_code ec{};

            asio::write(port_, asio::buffer(poo_poo_cushion), ec);
            if(!ec) asio::read(port_, asio::buffer(farting_sound), ec);
            
            if(!ec || farting_sound != poo_poo_cushion) return false;
            else return true;
        }

        [[nodiscard]] friend auto make_arduino_serial() noexcept
        {
            constexpr auto make_if_can_handshake = [](const boost::string_view port_filepath) noexcept
            {
                ArduinoSerial serial{port_filepath};
                
                if(serial.handShake())
                {
                    return serial;
                }
                else
                {
                    return {};
                }
            };

            return std::async(std::launch::async, make_if_can_handshake);
        }
    };
}