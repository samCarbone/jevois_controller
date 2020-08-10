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




    // Channels
    const double MAX_CHANNEL_VALUE = 100;
    const double MIN_CHANNEL_VALUE = -100;
    const unsigned char MSP_CHANNEL_ID = 200;
    static int channelToTxRange(double value); //

    // Mode
    const int MODE_PC = 1;
    const int MODE_JV = 2;
    const int MODE_ERR = 0;
    bool controllerActive = false;
    void setControllerActive(const bool ctrlActv);

    // Comms
    const int PING_TIMEOUT = 100;
    const int SEND_CONTROL_PERIOD_MS = 50; // ms, time between sending control commansd
    int pingLoopTime = 0;
    bool sendChannels(const std::array<double, 16> &channels, const bool response=false); //
    bool sendEspMode(const int mode, const bool response=true); //
    void parsePacket(QByteArray &data); //
    void parseMode(QJsonObject mode_obj); //
    void parsePing(QJsonObject ping_obj); //
    void parseAltitude(QJsonObject alt_obj); //

    // Control system
    std::array<double, 4> controllerChannels = {0, 0, 0, 0};
//    AltitudeController* altController;
    AltitudeEstimator* altEstimator;
    // void updateControllerChannels(); // TODO: change

    // Files
    std::ofstream file_log;
    bool filesOpen = false;
    std::string header_log = "time_esp_ms,time_esp_prop,Delta_t_prop_ms,z_prop,z_dot_prop,chnThr,chnEle,chnAil,chnRud";
    std::string prefix_log = "alt_prop_ctrl_";
    std::string format = ".txt";
    std::string suffix = "temp";
    std::string fileDirectory = "";


};

#endif // CONDUCTOR_H
