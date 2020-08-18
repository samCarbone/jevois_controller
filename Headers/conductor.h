#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <array>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <nlohmann/json.hpp>

#include "def.h"
#include "altitudeestimator.h"
#include "altitudecontroller.h"

using json = nlohmann::json;

enum log_levels { ALL, DEBUG, INFO, WARN, ERROR, FATAL, OFF };

const int GLOBAL_LOG_LEVEL = DEBUG;

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
    std::vector<char> serial_read_concat;


    void sendTimerDone();

    const int SEND_CONTROL_PERIOD_MS = 500; // ms, time between sending control commansd
    const int PROP_TIMEOUT = 100;


    // Channels
    const double MAX_CHANNEL_VALUE = 100;
    const double MIN_CHANNEL_VALUE = -100;
    const unsigned char MSP_CHANNEL_ID = 200;
    std::array<double, 4> controller_channels = {0, 0, 0, 0};
    double get_controller_channel_value(const int channel);
    int value_to_tx_range(double value);
    double saturate(double channelValue);

    // Mode
    bool controller_activity = false;
    void set_controller_activity(const bool is_active);
    bool get_controller_activity();
    bool landing = false;
    void set_landing(bool is_landing);

    // Send Timer
    std::chrono::steady_clock::time_point start_time_jv;
    boost::asio::high_resolution_timer* send_timer;
    const long CONTROL_LOOP_PERIOD_MS = 500; // ms
    void timer_handler(const boost::system::error_code& error);
    int time_elapsed_ms();

    // Comms
    void send_channels(const std::array<double, 16> &channels, const bool response=false);
    bool find_first_json(const std::vector<char> &inVec, int &start, int &end);
    bool find_first_msp(const std::vector<char> &inVec, int &start, int &end);
    void parse_packet(const std::vector<char> &inVec, const int start, const int end);
    void parse_mode(const json &mode_obj);
    void parse_landing(const json &land_obj);
    void parse_altitude(const json &alt_obj);
    void send_mode(const bool active);
    void send_landing(const bool active);
    void pub_log_check(const std::string &in_str, int log_level, bool send);
    void send_log(const std::string &in_str, int log_level);

    // Control system
    AltitudeController* alt_controller;
    AltitudeEstimator* alt_estimator;

    // File Methods
    bool open_files(); //
    void close_files(); //
    void set_file_suffix(std::string suffix_in); //
    std::string get_file_suffix(); //
    void set_file_directory(std::string directory_in); //
    std::string get_file_directory(); //

    // Files - general properties
    bool files_open = false;
    std::string format = ".txt";
    std::string suffix = "jv";
    std::string file_directory = ".";
    // Logging messages
    std::ofstream file_log;
    std::string header_log = "********************* LOG *********************";
    std::string prefix_log = "log_";
    // control signals
    std::ofstream file_sig;
    std::string header_sig = "time_esp_ms,time_esp_prop,Delta_t_prop_ms,z_prop,z_dot_prop,chnThr,chnEle,chnAil,chnRud";
    std::string prefix_sig = "alt_prop_ctrl_";
    

};

#endif // CONDUCTOR_H
