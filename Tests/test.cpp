
// Serial code adapted from
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/serial_port.hpp>

#include <string>

#include "CRC.h"

#include <cstdint>


enum msg_type {IDLE_W, SOF_A, LEN_LO, PAYLOAD, CRC_HI, CRC_LO, VALID};

boost::asio::serial_port* esp_port;
void serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
void serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
std::array<char, 256> serial_read_buffer;
std::array<char, 256> serial_write_buffer;

std::vector<char> serial_payload;
std::size_t payload_len = 0;

msg_type recv_state = IDLE_W;
std::size_t payload_idx = 0;
std::uint8_t cksum_hi = 0;

void send_payload(std::vector<char> &data);


int main(int argc, char*argv[])
{
    boost::asio::io_service io;
    
    // Setup serial port
    std::string serial_port_name = "/dev/ttyACM0";
    esp_port = new boost::asio::serial_port(io, serial_port_name);
    esp_port->set_option(boost::asio::serial_port_base::baud_rate(57600));
    esp_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    esp_port->set_option(boost::asio::serial_port_base::character_size(8));
    esp_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    esp_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    
    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&serial_read_handler, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    serial_payload.reserve(256);
    
    // Run boost io
    io.run();
    
    delete esp_port;
}


void serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(!error)
    {
        
        std::size_t bytes_available = bytes_transferred;
        std::size_t read_idx = 0;
        while(read_idx < bytes_transferred) {
        
            if(recv_state == IDLE_W) {
                if(serial_read_buffer.at(read_idx) == '#') {
                    recv_state = SOF_A;
                }
                read_idx++;
            }
            
            if(recv_state == SOF_A && read_idx < bytes_transferred) {
                if(serial_read_buffer.at(read_idx) == 'S') {
                    recv_state = LEN_LO;
                }
                else if(serial_read_buffer.at(read_idx) == '#') {
                    recv_state = SOF_A;
                }
                else {
                    recv_state = IDLE_W;
                }
                read_idx++;
            }
            
            if(recv_state == LEN_LO && read_idx < bytes_transferred) {
                payload_len = serial_read_buffer.at(read_idx);
                payload_idx = 0;
                serial_payload.clear();
                recv_state = PAYLOAD;
                read_idx++;
            }
            
            if(recv_state == PAYLOAD) {
                if(bytes_transferred < read_idx || payload_len < payload_idx) {
                    recv_state = IDLE_W; // Reset the state
                    break;
                }
                std::size_t read_len = (bytes_transferred - read_idx) < (payload_len - payload_idx) ? (bytes_transferred - read_idx) : (payload_len - payload_idx);
                if(read_len > 0) {
                    serial_payload.insert(serial_payload.end(), serial_read_buffer.begin()+read_idx, serial_read_buffer.begin()+read_idx+read_len);
                    payload_idx += read_len;
                }
                if(payload_idx >= payload_len) {
                    recv_state = CRC_HI;
                }
                read_idx += read_len;
            }
            
            if(recv_state == CRC_HI && read_idx < bytes_transferred) {
                cksum_hi = serial_read_buffer.at(read_idx);
                read_idx++;
                recv_state = CRC_LO;
            }
            
            if(recv_state == CRC_LO && read_idx < bytes_transferred) {
                std::uint8_t cksum_lo = serial_read_buffer.at(read_idx);
                read_idx++;
                
                // Now check the checksum
                std::uint16_t sum_recv = (static_cast<std::uint16_t>(cksum_hi) << 8) + static_cast<std::uint16_t>(cksum_lo);
                std::uint16_t sum_calc = CRC::Calculate(serial_payload.data(), serial_payload.size(), CRC::CRC_16_XMODEM());
                if(sum_calc == sum_recv) {
                    recv_state = VALID;
                }
                else {
                    recv_state = IDLE_W;
                }
                
            }
            
            if(recv_state == VALID) {
                // Now process the payload
                // ----- //
                // Print on the console
                std::cout << "Return-->";
                for(auto &a : serial_payload) {
                    std::cout << a;
                }
                std::cout << std::endl;
                
                // Echo back
                send_payload(serial_payload);
                
                // Reset the state
                recv_state = IDLE_W;
                
            }
        }
        
        
    }

    
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&serial_read_handler, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    
}

void serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    // This is called once the data is written to the serial port
    if(!error)
    {

    }
}


// Function to construct/send packet
void send_payload(std::vector<char> &data) {
    std::vector<char> packet;
    packet.reserve(data.size() + 5);
    packet.push_back('#');
    packet.push_back('S');
    std::uint8_t payload_size = data.size() <= 255 ? data.size() : 255;
    packet.push_back(static_cast<char>(payload_size));
    packet.insert(packet.end(), data.begin(), data.begin()+payload_size);
    
    // Calculate the checksum
    std::uint16_t sum_calc = CRC::Calculate(data.data(), payload_size, CRC::CRC_16_XMODEM());
    packet.push_back(static_cast<char>((sum_calc & 0xFF00) >> 8));
    packet.push_back(static_cast<char>(sum_calc & 0xFF));
    
    // Send
    esp_port->async_write_some( boost::asio::buffer(packet), boost::bind(&serial_write_handler, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

}

