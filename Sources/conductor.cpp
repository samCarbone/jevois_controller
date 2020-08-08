#include "conductor.h"

Conductor::Conductor(std::serial serial_port_name)
{
    // Construct and open the serial port
    esp_port = new boost::asio::serial_port(io, serial_port_name);
    esp_port->set_option(asio::serial_port_base::baud_rate(115200));
    esp_port->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    esp_port->set_option(asio::serial_port_base::character_size(8));
    esp_port->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    esp_port->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), serial_read_handler);

    // Setup timer
    timer = new boost::asio::deadline_timer(io, boost::posix_time::millisec(control_loop_period_ms));
    timer->async_wait(timer_handler);

    // Run boost io
    io.run();

}

~Conductor::Conductor()
{

}

void Conductor::serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(!error)
    {
        // Serial data available
        // Echo data back
        serial_write_buffer = serial_read_buffer;
        esp_port->async_write_some(boost::asio::buffer(serial_write_buffer, bytes_transferred), serial_write_handler);

    }
}

void Conductor::serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    // This is called once the data is written to the serial port
    if(!error)
    {

    }
}

void Conductor::timer_handler(const boost::system::error_code& error)
{

    // Calculate a new expiry time relative to previous to prevent drift
    timer->expires_at(timer->expires_at() + boost::posix_time::millisec(control_loop_period_ms));
    // Restart the timer
    timer->async_wait(timer_handler);

    if(!error)
    {
        // Timer expired
    }
}