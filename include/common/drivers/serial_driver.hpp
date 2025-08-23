#pragma once
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <iostream>

class SerialDriver {
public:
    using ReceiveCallback = std::function<void(const uint8_t* data, std::size_t len)>;
    using ErrorCallback   = std::function<void(const boost::system::error_code& ec)>;

    struct SerialPortConfig {
        unsigned int baud_rate = 115200;
        unsigned int char_size = 8;
        boost::asio::serial_port_base::parity::type parity = boost::asio::serial_port_base::parity::none;
        boost::asio::serial_port_base::stop_bits::type stop_bits = boost::asio::serial_port_base::stop_bits::one;
        boost::asio::serial_port_base::flow_control::type flow_control = boost::asio::serial_port_base::flow_control::none;
    };

    explicit SerialDriver(std::size_t read_buf_size = 4096)
        : io_(),
          port_(io_),
          read_buf_(read_buf_size),
          running_(false)
    {}

    ~SerialDriver() {
        stop();
    }

    void init_port(const std::string& device, const SerialPortConfig& cfg) {
        device_ = device;
        config_ = cfg;
    }

    void set_receive_callback(ReceiveCallback cb) { receive_cb_ = std::move(cb); }
    void set_error_callback(ErrorCallback cb) { error_cb_ = std::move(cb); }

    bool start() {
        boost::system::error_code ec;
        port_.open(device_, ec);
        if(ec) {
            if(error_cb_) error_cb_(ec);
            return false;
        }

        port_.set_option(boost::asio::serial_port_base::baud_rate(config_.baud_rate), ec);
        port_.set_option(boost::asio::serial_port_base::character_size(config_.char_size), ec);
        port_.set_option(boost::asio::serial_port_base::parity(config_.parity), ec);
        port_.set_option(boost::asio::serial_port_base::stop_bits(config_.stop_bits), ec);
        port_.set_option(boost::asio::serial_port_base::flow_control(config_.flow_control), ec);

        running_ = true;
        read_thread_ = std::thread([this]() { read_loop(); });

        return true;
    }

    void stop() {
        if(!running_) return;
        running_ = false;

        boost::system::error_code ec;
        if(port_.is_open()) port_.cancel(ec);
        if(port_.is_open()) port_.close(ec);

        if(read_thread_.joinable()) read_thread_.join();
    }

    bool is_open() const { return port_.is_open(); }

    bool write(const std::vector<uint8_t>& data) {
        if(!port_.is_open()) return false;
        boost::system::error_code ec;
        size_t bytes_written = boost::asio::write(port_, boost::asio::buffer(data), ec);
        if(ec) {
            if(error_cb_) error_cb_(ec);
            return false;
        }
        return bytes_written == data.size();
    }

private:
    void read_loop() {
        while(running_ && port_.is_open()) {
            boost::system::error_code ec;
            size_t n = port_.read_some(boost::asio::buffer(read_buf_), ec);
            if(ec) {
                if(error_cb_) error_cb_(ec);
                break;
            }
            if(n > 0 && receive_cb_) {
                receive_cb_(read_buf_.data(), n);
            }
        }
    }

private:
    boost::asio::io_context io_;
    boost::asio::serial_port port_;
    std::vector<uint8_t> read_buf_;
    SerialPortConfig config_;
    std::string device_;

    std::atomic<bool> running_;
    std::thread read_thread_;

    ReceiveCallback receive_cb_;
    ErrorCallback error_cb_;
};
