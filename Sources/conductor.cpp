#include "conductor.h"


Conductor::Conductor()
{
	Conductor("/dev/ttyS0");
}

Conductor::Conductor(std::string serial_port_name)
{
    // Construct and open the serial port
    esp_port = new boost::asio::serial_port(io, serial_port_name);
    esp_port->set_option(boost::asio::serial_port_base::baud_rate(115200));
    esp_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    esp_port->set_option(boost::asio::serial_port_base::character_size(8));
    esp_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    esp_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    // Setup timer
    // timer = new boost::asio::deadline_timer(io, boost::posix_time::millisec(control_loop_period_ms));
    timer = new boost::asio::high_resolution_timer(io, std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(control_loop_period_ms));
    timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));

    prev_jv_time = std::chrono::high_resolution_clock::now();

    // Run boost io
    io.run();

}

Conductor::~Conductor()
{
    esp_port->cancel();
    esp_port->close();
    timer->cancel();

    delete esp_port;
    delete timer;
}

void Conductor::serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(!error)
    {
        // Serial data available
        // Echo data back
        serial_write_buffer = serial_read_buffer;
        // esp_port->async_write_some(boost::asio::buffer(serial_write_buffer, bytes_transferred), serial_write_handler);
	esp_port->async_write_some( boost::asio::buffer(serial_write_buffer, bytes_transferred), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

    }

    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

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
    // timer->expires_at(timer->expires_at() + boost::posix_time::millisec(control_loop_period_ms));
    timer->expires_at(timer->expires_at() + std::chrono::milliseconds(control_loop_period_ms));
    // Restart the timer
    timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));

    if(!error)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::string write_str;
        std::ostringstream os;
        os << "Time diff: " << std::chrono::duration_cast<std::chrono::microseconds>(current_time - prev_jv_time).count() << " us\r\n";
        write_str = os.str();
        prev_jv_time = current_time;
        esp_port->async_write_some( boost::asio::buffer(write_str), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );


        // Timer expired
    }
}
