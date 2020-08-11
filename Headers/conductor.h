#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <array>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <nlohmann/json.hpp>
#include "def.h"

using asio = boost::asio;
using json = nlohmann::json;

class Conductor
{

public:
    Conductor();
    Conductor(std::string serial_port_name);
    ~Conductor();        

private:
    asio::io_service io;
    asio::serial_port* esp_port;
    void serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    std::array<char, 256> serial_read_buffer;
    std::array<char, 256> serial_write_buffer;
    std::vector<char> serial_read_concat;

    bool find_first_set(const std::vector<char> &inVec, int &start, int &end)

    std::chrono::steady_clock::time_point prev_jv_time;
    asio::high_resolution_timer* timer;
    const long control_loop_period_ms = 1000; // ms
    void timer_handler(const boost::system::error_code& error);

    bool controllerActive = false;
    bool getControllerActive();
    void setControllerActive(const bool ctrlActv);

    // Files
    bool openFiles(); //
    void closeFiles(); //
    void setFileSuffix(std::string suffix_in); //
    std::string getFileSuffix(); //
    void setFileDirectory(std::string directory); //
    std::string getFileDirectory(); //

    void sendTimerDone();


    // Channels
    const double MAX_CHANNEL_VALUE = 100;
    const double MIN_CHANNEL_VALUE = -100;
    const unsigned char MSP_CHANNEL_ID = 200;
    void setJoyChannelValue(const int channel, double value);
    static int channelToTxRange(double value);
    double saturate(double channelValue);

    const int SEND_CONTROL_PERIOD_MS = 50; // ms, time between sending control commansd
    const int PROP_TIMEOUT = 100;

    // Comms
    bool sendChannels(const std::array<double, 16> &channels, const bool response=false); //
    void parse_packet(const std::vector<char> &inVec, const int start, const int end); //
    void parseMode(QJsonObject mode_obj); //
    void parsePing(QJsonObject ping_obj); //
    void parseAltitude(QJsonObject alt_obj); //

    // Control system
    std::array<double, 4> controllerChannels = {0, 0, 0, 0};
    AltitudeController* altController;
    AltitudeEstimator* altEstimator;


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
