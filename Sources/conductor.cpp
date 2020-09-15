#include "conductor.h"

// **********************************************************
// Constructor/Destructor
// **********************************************************

Conductor::Conductor()
{
	Conductor("/dev/ttyS0", 57600);
}

Conductor::Conductor(std::string serial_port_name, unsigned int baud_rate)
{

    // Construct and open the serial port
    esp_port = new boost::asio::serial_port(io, serial_port_name);
    esp_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    esp_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    esp_port->set_option(boost::asio::serial_port_base::character_size(8));
    esp_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    esp_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    
    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    serial_payload.reserve(256);

    // Epoch time for receiving esp messages
    start_time_jv = std::chrono::steady_clock::now();

    // Timer to send control signals
    send_timer = new boost::asio::high_resolution_timer(io, std::chrono::milliseconds(CONTROL_LOOP_PERIOD_MS));
    send_timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));

    // Altitude controller
    alt_controller = new AltitudeController();
    alt_estimator = new AltitudeEstimator();
    lateral_estimator = new LateralEstimator();

    // Open files - allow for immediate recording
    #ifndef IS_HOST
    set_file_suffix("jv");
    set_file_directory("/jevois/scripts/logs");
    #endif
    #ifdef IS_HOST
    set_file_suffix("jv");
    set_file_directory("/home/samuel/Documents/host_logs");
    #endif

    open_files();

    pub_log_check("Started", LL_INFO, true);

    // Run boost io
    io.run();

}

Conductor::~Conductor()
{
    esp_port->cancel();
    esp_port->close();
    send_timer->cancel();

    close_files();

    delete esp_port;
    delete send_timer;
    delete alt_controller;
    delete alt_estimator;
    delete lateral_estimator;
}

// **********************************************************
//
// **********************************************************



void Conductor::serial_read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
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
                else {
                    std::cerr << serial_read_buffer.at(read_idx);
                    #ifdef IS_HOST
                    std::cout << serial_read_buffer.at(read_idx);
                    #endif
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
                    std::cerr << serial_read_buffer.at(read_idx);
                    #ifdef IS_HOST
                    std::cout << serial_read_buffer.at(read_idx);
                    #endif
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
                parse_packet(serial_payload);
                
                // Reset the state
                recv_state = IDLE_W;
                
            }
        }
    }
    else {
        pub_log_check("Serial read error", LL_ERROR, true);
    }

    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

}

// Function to construct/send packet
void Conductor::send_payload(const std::vector<char> &data) {
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
    esp_port->async_write_some( boost::asio::buffer(packet), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

}

void Conductor::send_payload(const std::string &data_str) {
    std::vector<char> data(data_str.begin(), data_str.end());
    send_payload(data);
}


// [{, }]
void Conductor::parse_packet(const std::vector<char> &inVec)
{

    if(inVec.size()<4) {
        pub_log_check("Packet len<4", LL_ERROR, true);
        return;
    }
    char src = inVec.at(0);
    char dst = inVec.at(1);
    char mid = inVec.at(2);
    char rsp = inVec.at(3);

    if(dst != DST_JV) {
        pub_log_check("Incorrect packet dest", LL_ERROR, true);
        return;
    }

    if(src == SRC_ESP) {

        if(mid == MID_ALT) {
            std::vector<unsigned char> alt_data(inVec.begin()+4, inVec.end());
            parse_altitude(alt_data);
        }
        else if(mid == MID_MSP) {
            std::vector<unsigned char> att_data(inVec.begin()+4, inVec.end());
            parse_attitude_msp(att_data);
        }
        
    }
    else if(src == SRC_PC) {
        pub_log_check("SRC PC", LL_DEBUG, true);

        if(mid == MID_MODE && inVec.size() == 5) {
            if(inVec.at(4) == JV_CTRL_ENA) {
                set_controller_activity(true);
            }
            else if(inVec.at(4) == JV_CTRL_DIS) {
                set_controller_activity(false);
            }

            if(rsp == RSP_TRUE) {
                send_mode(controller_activity);
            }
        }
        else if(mid == MID_LAND && inVec.size() == 5) {
            if(inVec.at(4) == JV_LAND_ENA) {
                set_landing(true);
            }
            else if(inVec.at(4) == JV_LAND_DIS) {
                set_landing(false);
            }

            if(rsp == RSP_TRUE) {
                send_landing(landing);
            }  
        }
        else if(mid == MID_QUIT && inVec.size() == 5) {
            if(inVec.at(4) == JV_QUIT_TRUE) {
                set_controller_activity(false);
                pub_log_check("Quit", LL_INFO, true); // Might not send
                send_timer->cancel();
                esp_port->cancel();
                esp_port->close();
            }
            else {
                pub_log_check("Not quit", LL_INFO, true); // Not necessary
            }
        }
    }
    else if(src == SRC_FC) {
        if(mid == MID_MSP) {

        }
    }

}

void Conductor::serial_write_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    // This is called once the data is written to the serial port
    if(!error)
    {

    }
    else {
        pub_log_check("Serial write error", LL_ERROR, true);
    }
}


void Conductor::parse_altitude(const std::vector<unsigned char> &altData)
{

    if(altData.size() != 22) {
        pub_log_check("Invalid alt packet", LL_ERROR, true);
        return;
    }

    mod_VL53L1_RangingMeasurementData_t rawData = {0, 0, 0, 0, 0, 0, 0, 0};
    rawData.TimeStamp = (std::uint32_t)altData.at(0) | (std::uint32_t)altData.at(1) << 8 | (std::uint32_t)altData.at(2) << 16 | (std::uint32_t)altData.at(3) << 24;
    rawData.StreamCount = (std::uint8_t)altData.at(4);
    rawData.SignalRateRtnMegaCps = (FixPoint1616_t)altData.at(5) | (FixPoint1616_t)altData.at(6) << 8 | (FixPoint1616_t)altData.at(7) << 16 | (FixPoint1616_t)altData.at(8) << 24;
    rawData.AmbientRateRtnMegaCps = (FixPoint1616_t)altData.at(9) | (FixPoint1616_t)altData.at(10) << 8 | (FixPoint1616_t)altData.at(11) << 16 | (FixPoint1616_t)altData.at(12) << 24;
    rawData.EffectiveSpadRtnCount = (std::uint16_t)altData.at(13) | (std::uint16_t)altData.at(14) << 8;
    rawData.SigmaMilliMeter = (FixPoint1616_t)altData.at(15) | (FixPoint1616_t)altData.at(16) << 8 | (FixPoint1616_t)altData.at(17) << 16 | (FixPoint1616_t)altData.at(18) << 24;
    rawData.RangeMilliMeter = (std::int16_t)altData.at(19) | (std::int16_t)altData.at(20) << 8;
    rawData.RangeStatus = (std::uint8_t)altData.at(21);

    RangingData_t convData;
    convData.timeEsp_ms = (long int) rawData.TimeStamp;
    convData.timePc_ms = (long int) time_elapsed_ms();
    convData.signal_rate = rawData.SignalRateRtnMegaCps/65336.0;
    convData.ambient_rate = rawData.AmbientRateRtnMegaCps/65336.0;
    convData.eff_spad_count = rawData.EffectiveSpadRtnCount/256.0;
    convData.sigma_mm = rawData.SigmaMilliMeter/65336.0;
    convData.range_mm = rawData.RangeMilliMeter;
    convData.status = rawData.RangeStatus;

    // #ifdef IS_HOST
    // std::cout << "Time esp: " << convData.timeEsp_ms
    //             << ", Time pc: " << convData.timePc_ms
    //             << ", sigma_mm: " << convData.sigma_mm
    //             << ", range_mm: " << convData.range_mm << std::endl;
    // #endif

    // Update state estimate
    alt_estimator->addRangeMeasurement(convData);
    AltState_t estimatedState = alt_estimator->getStateEstimate();

    // Update controller
    if(controller_activity) {
        alt_controller->addEstState(estimatedState);
    }

}

void Conductor::parse_attitude_msp(const std::vector<unsigned char> &attData)
{
    // Check the header and message length are valid
    bool valid_header = false;
    if(attData.size() >= 5) {
        valid_header = attData.at(0) == '$' && attData.at(1) == 'M' 
                    && attData.at(2) == '>' && attData.at(3) == '6' 
                    && attData.at(4) == MSP_ATTITUDE;
    }

    if(attData.size() != 12 || !valid_header) {
        pub_log_check("Invalid attitude msp packet", LL_ERROR, true);
        return;
    }

    unsigned char crc = attData.at(3) ^ attData.at(4);
    std::uint16_t roll, pitch, yaw;
    roll = (std::uint16_t)attData.at(5) | (std::uint16_t)attData.at(6) << 8; // [-1800 : 1800] 1/10 deg
    pitch = (std::uint16_t)attData.at(7) | (std::uint16_t)attData.at(8) << 8; // [-900 : 900] 1/10 deg
    yaw = (std::uint16_t)attData.at(9) | (std::uint16_t)attData.at(10) << 8; // [-180 : 180] deg
    long int time_ms = time_elapsed_ms();

    AltState_t alt_state = alt_estimator->getStateEstimate();
    if(!alt_state.valid) {
        pub_log_check("IMU value received before altitude", LL_WARN, true);
    }
    lateral_estimator->add_attitude(roll, pitch, yaw, alt_state.z, time_ms);

}


// **********************************************************
// Channels
// **********************************************************


double Conductor::get_controller_channel_value(const int channel) 
{
    controller_channels.at(channel);
}

int Conductor::value_to_tx_range(double value)
{
    if (value > MAX_CHANNEL_VALUE) {
        value = MAX_CHANNEL_VALUE;
        pub_log_check("channel value out of range (greater)", LL_WARN, true);
    }
    else if (value < MIN_CHANNEL_VALUE) {
        value = MIN_CHANNEL_VALUE;
        pub_log_check("channel value out of range (lower)", LL_WARN, true);
    }

    return round(value*6 + 1500); // Scaled from (-100, 100) to (900, 2100)
}

double Conductor::saturate(double channel_value, const double MIN, const double MAX)
{
    if (channel_value > MAX) {
        channel_value = MAX;
    }
    else if (channel_value < MIN) {
        channel_value = MIN;
    }
    return channel_value;
}


// **********************************************************
// Mode
// **********************************************************

void Conductor::set_controller_activity(const bool is_active)
{

    if(is_active != controller_activity) {

        landing = false;
        controller_activity = is_active;
        alt_controller->resetState();
        lateral_estimator->reset();

        if(is_active) {
            // Make a better way of setting the target
            AltTarget_t targetTemp; targetTemp.z = -0.5; targetTemp.z_dot = 0;
            alt_controller->setTarget(targetTemp);
            // Close/reopen files to reprint each of the headers
            close_files();
            open_files();
            pub_log_check("Controller started", LL_INFO, true);
        }
        else {
            pub_log_check("Controller stopped", LL_INFO, true);
        }

    }

}

bool Conductor::get_controller_activity()
{
    return controller_activity;
}

void Conductor::set_landing(bool is_landing)
{
    if(landing != is_landing) {
        landing = is_landing;
        if(landing) {
            pub_log_check("Landing started", LL_INFO, true);
        }
        else {
            pub_log_check("Landing stopped", LL_INFO, true);
        }
    }
}

void Conductor::send_mode(const bool active)
{

    std::vector<char> send_data = { SRC_JV, DST_PC, MID_MODE, RSP_FALSE, active ? JV_CTRL_ENA : JV_CTRL_DIS};
    send_payload(send_data);
}

void Conductor::send_landing(const bool active)
{

    std::vector<char> send_data = { SRC_JV, DST_PC, MID_LAND, RSP_FALSE, active ? JV_LAND_ENA : JV_LAND_DIS};
    send_payload(send_data);

}

// **********************************************************
// Send Timer
// **********************************************************

long int Conductor::time_elapsed_ms()
{
    return (long int)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_jv).count();
}

void Conductor::timer_handler(const boost::system::error_code& error)
{

    // Calculate a new expiry time relative to previous to prevent drift
    send_timer->expires_at(send_timer->expires_at() + std::chrono::milliseconds(CONTROL_LOOP_PERIOD_MS));
    // Restart the timer
    send_timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));


    if(!error)
    {

        pub_log_check("Send timer", LL_DEBUG, true);

        if(!sent_initial) {
            send_mode(controller_activity);
            send_landing(landing);
            sent_initial = true;
        }

        if(controller_activity && alt_controller->isValidState()) {

            if(landing) {
                AltTarget_t targetTemp;
                if(alt_controller->getTarget(targetTemp)) {
                    targetTemp.z += 0.1*((double)CONTROL_LOOP_PERIOD_MS)/1000.0;
                    targetTemp.z_dot = 0.1;
                    targetTemp.z = targetTemp.z > 0.0 ? targetTemp.z : 0.0;
                    alt_controller->setTarget(targetTemp);
                }
            }


            double chn_thr = -100;
            double chn_ele = 0;
            double chn_ail = 0;
            double chn_rud = 0;
            bool error = false;
            std::string error_str  = "";
            long int current_pc_time_ms = time_elapsed_ms();
            AltState_t prop_alt_state = alt_estimator->getPropagatedStateEstimate_safe(current_pc_time_ms, PROP_LIMIT, error, error_str);
            chn_thr = saturate(alt_controller->getControlTempState(prop_alt_state), MIN_CHANNEL_VALUE, MAX_THROTTLE);
            // TODO: find a way to not set the arm channel
            // 0->ail, 1->ele, 2->thr, 3->rud, 4->arm
            std::array<double, 16> mixed_channels = {chn_ail, chn_ele, chn_thr, chn_rud, 100, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            send_channels(mixed_channels, false);

            if(error) {
                pub_log_check(error_str, LL_ERROR, true);
            }

            #ifdef IS_HOST
            std::cout << "Prop-> Time esp: " << prop_alt_state.timeEsp_ms
                        << ", Time pc: " << prop_alt_state.timePc_ms
                        << std::fixed << std::setprecision(4) << ", z: " << prop_alt_state.z
                        << ", \tz_dot: " << prop_alt_state.z_dot;
            std::cout   << ",  \tchn_thr: " << chn_thr
                        << std::endl;
            #endif

            if(files_open) {
                // header
                // "time_esp_ms,time_esp_prop,Delta_t_prop_ms,time_pc_ms,z_prop,z_dot_prop,chnThr,chnEle,chnAil,chnRud"
                file_sig << alt_estimator->getCurrentTimeEsp_ms() << "," << prop_alt_state.timeEsp_ms << "," << prop_alt_state.timeEsp_ms-alt_estimator->getCurrentTimeEsp_ms() << ","
                         << current_pc_time_ms << prop_alt_state.z << "," << prop_alt_state.z_dot << "," << chn_thr << "," << chn_ele << "," << chn_ail << "," << chn_rud << std::endl;
            }

        }

    }

}



void Conductor::send_channels(const std::array<double, 16> &channels, const bool response)
{

    std::vector<char> msp_data;
    msp_data.reserve(6+2*channels.size());
    char message_len = static_cast<char>(2*channels.size()); // If channels.size() is greater than 63, then this will not be valid
                                        // Intentionally overflowing 
    char message_id = static_cast<char>(MSP_CHANNEL_ID); // Intentionally overflow
    
    // Header
    msp_data.push_back('$'); msp_data.push_back('M'); msp_data.push_back('<');
    msp_data.push_back(message_len);
    msp_data.push_back(message_id);

    // Channel values
    for(auto value : channels) {
        int rangedValue = value_to_tx_range(value);
        char lower_byte = static_cast<char>(rangedValue & 0xFF);
        char upper_byte = static_cast<char>((rangedValue & 0xFF00) >> 8);

        // Format as little-endian --> | least-significant byte | most-significant byte |
        msp_data.push_back(lower_byte);
        msp_data.push_back(upper_byte);
    }

    // Checksum
    char checksum = message_len ^ message_id;
    for(unsigned int i=5; i < msp_data.size(); i++) {
        checksum ^= msp_data.at(i);
    } 
    msp_data.push_back(checksum);

    // Insert msp data into a new vector. Not the most efficient way, but more portable
    std::vector<char> all_data = {SRC_JV, DST_FC, MID_MSP, response ? RSP_TRUE : RSP_FALSE};
    all_data.insert(all_data.end(), msp_data.begin(), msp_data.end());

    // Write to serial
    send_payload(all_data);

}



void Conductor::pub_log_check(const std::string &in_str, int log_level, bool send)
{
    if(log_level >= GLOBAL_LOG_LEVEL) {
        std::string lvl_str = "[]";
        if(log_level == LL_ALL) { lvl_str = "[ALL] "; }
            else if(log_level == LL_DEBUG) { lvl_str = "[DEBUG] "; }
            else if(log_level == LL_INFO) { lvl_str = "[INFO] "; }
            else if(log_level == LL_WARN) { lvl_str = "[WARN] "; }
            else if(log_level == LL_ERROR) { lvl_str = "[ERROR] "; }
            else if(log_level == LL_FATAL) { lvl_str = "[FATAL] "; }
            else if(log_level == LL_OFF) { lvl_str = "[OFF] "; }

        if(file_log.is_open())
            file_log << lvl_str << in_str << std::endl;
        if(send)
            send_log(in_str, log_level);
    }
}

void Conductor::send_log(const std::string &in_str, int log_level)
{
    
    json send_doc;
    send_doc["typ"] = "log";
    send_doc["lvl"] = log_level;
    send_doc["log"] = in_str;

    std::string s = send_doc.dump();
    std::vector<char> all_data = {SRC_JV, DST_PC, MID_JSON, RSP_FALSE};
    all_data.insert(all_data.end(), s.begin(), s.end());

    send_payload(all_data);

}


// **********************************************************
// Files
// **********************************************************


bool Conductor::open_files()
{

    // Returns true if all files are open/opened
    if(!file_log.is_open()) {
        std::string name_log = file_directory + "/" + prefix_log + suffix + format;
        file_log.open(name_log, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }

    if(!file_sig.is_open()) {
        std::string name_sig = file_directory + "/" + prefix_sig + suffix + format;
        file_sig.open(name_sig, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }
    
    if (file_log.is_open()) {
        file_log << std::endl << header_log << std::endl;
    }
    if (file_sig.is_open()) {
        file_sig << std::endl << header_sig << std::endl;
    }

    bool status = file_log.is_open();
    status &= file_sig.is_open();
    status &= alt_estimator->open_files();
    status &= alt_controller->open_files();
    status &= lateral_estimator->open_files();

    files_open = status;

    // send error message if required

    return status;
}

void Conductor::close_files()
{
    file_log.close();
    alt_estimator->close_files();
    alt_controller->close_files();
    lateral_estimator->close_files();
    files_open = false;
}

void Conductor::set_file_suffix(std::string suffix_in)
{
    suffix = suffix_in;
    alt_estimator->set_file_suffix(suffix);
    alt_controller->set_file_suffix(suffix);
    lateral_estimator->set_file_suffix(suffix);
}

std::string Conductor::get_file_suffix()
{
    return suffix;
}

void Conductor::set_file_directory(std::string directory_in)
{
    file_directory = directory_in;
    alt_estimator->set_file_directory(directory_in);
    alt_controller->set_file_directory(directory_in);
    lateral_estimator->set_file_directory(directory_in);
}

std::string Conductor::get_file_directory()
{
    return file_directory;
}
