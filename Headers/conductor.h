#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <nlohmann/json.hpp>
#include <CRC.h>

#include "def.h"
#include "my_enums.h"
#include "altitudeestimator.h"
#include "altitudecontroller.h"
#include "lateralestimator.h"
#include "pidcontroller.h"
#include "waypointselector.h"

using json = nlohmann::json;
using namespace boost::interprocess;

enum msg_type {IDLE_W, SOF_A, LEN_LO, PAYLOAD, CRC_HI, CRC_LO, VALID};

const int GLOBAL_LOG_LEVEL = LL_INFO;

class Conductor
{

public:
    Conductor();
    Conductor(std::string serial_port_name, unsigned int baud_rate);
    ~Conductor();        

private:

    // Boost serial
    boost::asio::io_service io;
    boost::asio::serial_port* esp_port;
    void serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    std::array<char, 256> serial_read_buffer;
    std::vector<char> serial_payload;
    std::size_t payload_len = 0;
    msg_type recv_state = IDLE_W;
    std::size_t payload_idx = 0;
    std::uint8_t cksum_hi = 0;
    void send_payload(const std::vector<char> &data);
    void send_payload(const std::string &data_str);

    // Boost timer
    void sendTimerDone();
    const long int PROP_LIMIT = 100;

    // Boost IPC
    named_semaphore * sem_filled;
    named_semaphore * sem_empty;
    shared_memory_object * segment;
    mapped_region * memregion;
    cam_ipc_data_t * cam_data;
    bool get_cam_data(std::array<double,3> &rotation, std::array<double,3> &translation, long int &proc_time);

    // Channels
    static constexpr double MAX_CHANNEL_VALUE = 100;
    static constexpr double MIN_CHANNEL_VALUE = -100;
    const double MAX_THROTTLE = 20;
    const unsigned char MSP_CHANNEL_ID = 200;
    const unsigned char MSP_ATTITUDE = 108;
    std::array<double, 4> controller_channels = {0, 0, 0, 0};
    // 0->ail, 1->ele, 2->thr, 3->rud, 4->arm
    double get_controller_channel_value(const int channel);
    int value_to_tx_range(double value);
    double saturate(double channelValue, const double MIN=MIN_CHANNEL_VALUE, const double MAX=MAX_CHANNEL_VALUE);

    // Mode
    bool controller_activity = false;
    void set_controller_activity(const bool is_active);
    bool get_controller_activity();
    bool landing = false;
    void set_landing(bool is_landing);
    bool sent_initial = false;

    // Send Timer
    std::chrono::steady_clock::time_point start_time_jv;
    boost::asio::high_resolution_timer* send_timer;
    const long CONTROL_LOOP_PERIOD_MS = 50; // ms
    void timer_handler(const boost::system::error_code& error);
    long int time_elapsed_ms();

    // Comms
    void send_channels(const std::array<double, 16> &channels, const bool response=false);
    // bool find_first_json(const std::vector<char> &inVec, int &start, int &end);
    // bool find_first_msp(const std::vector<char> &inVec, int &start, int &end);
    void parse_packet(const std::vector<char> &inVec);
    // void parse_mode(const json &mode_obj);
    // void parse_landing(const json &land_obj);
    // void parse_altitude(const json &alt_obj);
    void parse_altitude(const std::vector<unsigned char> &altData);
    void parse_attitude_msp(const std::vector<unsigned char> &attData);
    void send_mode(const bool active);
    void send_landing(const bool active);
    void pub_log_check(const std::string &in_str, int log_level, bool send);
    void send_log(const std::string &in_str, int log_level);

    static bool find_json_start(std::vector<char> &data, int search_start, int &start);
    static bool find_json_end(std::vector<char> &prev_data, std::vector<char> &data, int search_start, int &end);
    static bool find_json_overlap(std::vector<char> &prev_data, std::vector<char> &data, int &end_delta);

    bool found_start = false;
    size_t SRC_MAX_LEN = 512; // Maximum length of serial read concat

    // Control system
    AltitudeController* alt_controller;
    AltitudeEstimator* alt_estimator;
    LateralEstimator* lateral_estimator;
    PIDcontroller* psi_control; double P_psi = 20; double I_psi = 0; double D_psi = 0;
    PIDcontroller* x_control; double P_x = 20; double I_x = 0; double D_x = 25;
    PIDcontroller* y_control; double P_y = 20; double I_y = 0; double D_y = 25;
    const double MIN_CHANNEl_LAT = -30;
    const double MAX_CHANNEL_LAT = 30;
    WaypointSelector* waypoints;

    // Start propagating lateral position only once a minimum altitude has been reached
    bool start_alt_reached = false;
    const double MIN_START_ALT = 0.2; // 0.2m start altitude (altitude, i.e. -z)

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
    std::string header_sig = "time_esp_ms,time_esp_prop,Delta_t_prop_ms,time_pc_ms,z_prop,z_dot_prop,chnThr,chnEle,chnAil,chnRud";
    std::string prefix_sig = "alt_prop_ctrl_";
    

};

#endif // CONDUCTOR_H
