#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include <iostream>
#include <string>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class Conductor
{

public:
    Conductor();
    Conductor(std::string serial_port_name);
    ~Conductor();        

private:
    boost::asio::io_service io;
    boost::asio::serial_port* esp_port;
    void serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    std::array<char, 256> serial_read_buffer;
    std::array<char, 256> serial_write_buffer;

    // boost::asio::deadline_timer* timer;
    std::chrono::high_resolution_clock::time_point prev_jv_time;
    boost::asio::high_resolution_timer* timer;
    const long control_loop_period_ms = 1000; // ms
    void timer_handler(const boost::system::error_code& error);

};

#endif // CONDUCTOR_H
