#pragma once

#include <cstring>
#include <array>
#include <optional>
#include <future>
#include <algorithm>

#include <boost/asio.hpp>

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
        // asio::io_context::strand read_strand_{io_context_};
        asio::io_context::strand write_strand_{io_context_};

        // read, writeそれぞれに一つ。
        asio::thread_pool thread_pool_{2};

        asio::serial_port port_{io_context_};

    public:
        ArduinoSerial(const std::string& port_filepath) noexcept{
            // set baudrate.
            port_->set_option(asio::serial_port_base::baud_rate(baud_rate_));
            // open.
            port_->open(port_filepath);

            asio::post(thread_pool_, [this]{io_context_.run()});
        }

        ~ArduinoSerial(){
            port_->close();
        }

        // でーたをあるどぅいーのにやる。すれっどせーふ。
        void async_afon(const std::vector<u8>& tx_frame){
            asio::post(write_strand_, [&tx_frame]{asio::async_write(port_, asio::buffer(tx_frame));});
        }

        // // でーたをあるどぅいーのよりうく。
        // void async_asendan(std::vector<u8>& rx_frame){
        //     asio::post(read_strand_, [&rx_frame]{asio::async_read(port_, asio::buffer(rx_frame))});
        // }

        u8 read_1byte() noexcept
        {
            std::array<u8, 1> arr;
            asio::read(port_, asio::buffer(arr), asio::transfer_exactly(1));
            return arr[0];
        }

        SerialData read_bytes(const size_t data_size) noexcept
        {
            SerialData data(data_size);
            asio::read(port_, asio::buffer(), asio::transfer_exactly(data_size));
            return data;
        }

    private:
        // 全ての通信の前に行うこと。それ以外の場合動作は保証されない
        bool handShake() noexcept{

            const std::vector<u8> poo_poo_cushion =
             {
                CRSLib::to_underlying(TopicId::bridge_command),
                CRSLib::to_underlying(BridgeCommand::handshake),
                'H', 'e', 'l', 'l', 'o', 'C', 'R', 'S'
             };

            std::vector<u8> farting_sound(10);
            boost::system::error_code ec{};

            asio::write(port_, asio::buffer(poo_poo_cushion), ec);
            if(!ec) asio::read(port_, asio::buffer(farting_sound), ec);
            
            if(!ec || farting_sound != poo_poo_cushion) return false;
            else return true;
        }

        [[nodiscard]] friend std::future<std::optional<ArduinoSerial>> make_arduino_serial(const std::string& port_filepath) noexcept;
    };

    [[nodiscard]] friend std::future<std::optional<ArduinoSerial>> make_arduino_serial(const std::string& port_filepath) noexcept
    {
        constexpr auto make_if_can_handshake = [port_filepath]() noexcept
        {
            ArduinoSerial serial{port_filepath};
            
            if(serial.handShake())
            {
                return std::move(serial);
            }
            else
            {
                return {};
            }
        };

        return std::async(std::launch::async, make_if_can_handshake);
    }
}