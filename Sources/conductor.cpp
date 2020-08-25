#include "conductor.h"

// **********************************************************
// Constructor/Destructor
// **********************************************************

Conductor::Conductor()
{
	Conductor("/dev/ttyS0");
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

    // Open files - allow for immediate recording
    set_file_suffix("jv");
    set_file_directory("/jevois/scripts/logs");
    open_files();

    pub_log_check("Started", INFO, true);

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
                parse_packet(serial_payload);
                
                // Reset the state
                recv_state = IDLE_W;
                
            }
        }
    }
    else {
        pub_log_check("Serial read error", ERROR, true);
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
    esp_port->async_write_some( boost::asio::buffer(packet), boost::bind(&serial_write_handler, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

}

void Conductor::send_payload(const std::string &data_str) {
    std::vector<char> data(data_str.begin(), data_str.end());
    send_payload(data);
}

// [start, end] --> [", "]
bool Conductor::find_first_msp(const std::vector<char> &inVec, int &start, int &end)
{

    const std::array<char, 2> chrs_option_1 = {'{',','};
    const std::array<char, 7> chrs_2 = {'\"', 'm', 's','p','\"',':','\"'};
    bool found_start = false;
    const char chr_3 = '\"';
    const std::array<char, 2> chrs_option_4 = {',','}'};
    start = 0;
    end = 0;
    int expected_end = 0;
    const int min_msp_len = 6;

    // Find the start
    for(int i=0; i<(int)inVec.size()-(int)chrs_2.size()-2-min_msp_len; i++) {

        // Check for the first character
        // one OR the other
        for(auto &opt : chrs_option_1) {
            if(inVec.at(i) == opt) {
                found_start = true;
                break;
            }
        }
        if(!found_start)
            continue;

        // Check for the next sequence of characters
        for(int j=0; j<chrs_2.size(); j++) {

            if(inVec.at(i+1+j) != chrs_2.at(j)) {
                found_start = false;
                break;
            }

        }

        if(found_start) {
            start = i+chrs_2.size();
            // the msp message length is read and used to find the end
            unsigned char message_len = static_cast<unsigned char>(inVec.at(start+4));
            expected_end = start + min_msp_len + static_cast<int>(message_len) + 1;
            break;
        }

    }

    if(found_start && expected_end < (int)inVec.size()-1) {

        if(inVec.at(expected_end) != chr_3)
            return false;

        for(auto &opt: chrs_option_4) {
            if(inVec.at(expected_end+1) == opt) {
                end = expected_end;
                return true;
            }
        }

    }

    return false;

}

// [{, }]
void Conductor::parse_packet(const std::vector<char> &inVec)
{
    // Copy into a new vector --> not ideal if not necessary
    std::vector<char> cut_vec(inVec.begin(), inVec.end());

    /* Note: this does not support multiple msp messages
    within a single json packet. It will use the first msp message.
    */
    // MSP search ({,)"msp":"*"(,})
    // try {
    std::vector<char> msp_vec;
    bool found_msp = false;
    int start_msp = 0;
    int end_msp = 0;
    if(find_first_msp(cut_vec, start_msp, end_msp)) {
        // copy and erase between the " "
        msp_vec.insert(msp_vec.begin(), cut_vec.begin()+start_msp+1, cut_vec.begin()+end_msp);
        cut_vec.erase(cut_vec.begin()+start_msp+1, cut_vec.begin()+end_msp);

        found_msp = true;
    }
    else {
        // Safe any bytes outside acceptable range
        for (int i=0; i<cut_vec.size(); i++) {
            if(cut_vec.at(i) <= 31) {
                if(cut_vec.at(i) == '\r') {cut_vec.at(i) = 'r';}
                else if(cut_vec.at(i) == '\n') {cut_vec.at(i) = 'n';}
                else {cut_vec.at(i) = '#';}
            }
        }
    }

    // Catch all exceptions during parsing
    // We want to keep the program running and just ignore invalid json packets
    json j_doc;
    try {
        j_doc = json::parse(cut_vec);
    } catch(const std::exception& e) {
        pub_log_check("JSON parse error", ERROR, true);
        std::cerr << "[caught] " << e.what() << std::endl;   
        return; 
    }
        
    if(j_doc.is_null())
            return;

    if (j_doc["dst"] != "jv") {
        pub_log_check("Incorrect json dest", ERROR, true);
        return;
    }

    pub_log_check("JSON received", DEBUG, true);

    if(j_doc["snd"] == "esp") {

        if(j_doc["typ"] == "alt") {
            parse_altitude(j_doc["alt"]);
            pub_log_check("Altitude received", DEBUG, true);
        }
        
        else if(j_doc["typ"] == "msp") {
            // Need to search for the msp bytes
            // parse_msp(std::vector<char> msp_vec);
            pub_log_check("MSP received", DEBUG, true);
        }

    }

    else if (j_doc["snd"] == "pc") {

        if(j_doc["typ"] == "mode") {
            pub_log_check("Mode received", DEBUG, true);
            parse_mode(j_doc);
        }

        else if(j_doc["typ"] == "land") {
            pub_log_check("Landing received", DEBUG, true);
            parse_landing(j_doc);
        }

        else if(j_doc["typ"] == "quit") {
            set_controller_activity(false);
            pub_log_check("Quit", INFO, true); // Might not send
            send_timer->cancel();
            esp_port->cancel();
            esp_port->close();
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
        pub_log_check("Serial write error", ERROR, true);
    }
}



void Conductor::parse_altitude(const json &alt_obj)
{
    if(alt_obj.contains("sigrt") && alt_obj.contains("ambrt") && alt_obj.contains("sigma")
            && alt_obj.contains("spad") && alt_obj.contains("range") && alt_obj.contains("time")
            && alt_obj.contains("status") )
    {
        RangingData_t altData;
        altData.signal_rate = alt_obj["sigrt"].get<double>();
        altData.ambient_rate = alt_obj["ambrt"].get<double>();
        altData.sigma_mm = alt_obj["sigma"].get<double>();
        altData.eff_spad_count = alt_obj["spad"].get<double>()/256.0; // divide by 256 for real value
        altData.range_mm = alt_obj["range"].get<int>();
        altData.timeEsp_ms = alt_obj["time"].get<int>();
        altData.status = alt_obj["status"].get<int>();
        altData.timePc_ms = time_elapsed_ms();

        // Update state estimate
        alt_estimator->addRangeMeasurement(altData);
        AltState_t estimatedState = alt_estimator->getStateEstimate();

        // Update controller
        if(controller_activity) {
            alt_controller->addEstState(estimatedState);
        }

    }
    else 
    {
        pub_log_check("Invalid alt packet", ERROR, true);
    }

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
        pub_log_check("channel value out of range (greater)", WARN, true);
    }
    else if (value < MIN_CHANNEL_VALUE) {
        value = MIN_CHANNEL_VALUE;
        pub_log_check("channel value out of range (lower)", WARN, true);
    }

    return round(value*6 + 1500); // Scaled from (-100, 100) to (900, 2100)
}

double Conductor::saturate(double channel_value)
{
    if (channel_value > MAX_CHANNEL_VALUE) {
        channel_value = MAX_CHANNEL_VALUE;
    }
    else if (channel_value < MIN_CHANNEL_VALUE) {
        channel_value = MIN_CHANNEL_VALUE;
    }
    return channel_value;
}


// **********************************************************
// Mode
// **********************************************************

void Conductor::parse_mode(const json &mode_obj)
{
    if(mode_obj["mode"].get<int>() == 1) { // TODO: change to enum
        set_controller_activity(true);
        pub_log_check("Controller started", INFO, true);
    }
    else if(mode_obj["mode"].get<int>() == 2) { // TODO: change to enum
        set_controller_activity(false);
        pub_log_check("Controller stopped", INFO, true);
    }

    if(mode_obj["rsp"] == "true") {
        send_mode(controller_activity);
    }

}

void Conductor::parse_landing(const json &land_obj)
{
    if(land_obj["land"].get<int>() == 1) { // TODO: change to enum
        set_landing(true);
        pub_log_check("Landing started", INFO, true);
    }
    else if(land_obj["land"].get<int>() == 2) { // TODO: change to enum
        set_landing(false);
        pub_log_check("Landing stopped", INFO, true);
    }

    if(land_obj["rsp"] == "true") {
        send_landing(landing);
    }
}

void Conductor::set_controller_activity(const bool is_active)
{
    landing = false;
    controller_activity = is_active;
    alt_controller->resetState();

    if(is_active) {
        // Make a better way of setting the target
        AltTarget_t targetTemp; targetTemp.z = -0.5; targetTemp.z_dot = 0;
        alt_controller->setTarget(targetTemp);
    }

}

bool Conductor::get_controller_activity()
{
    return controller_activity;
}

void Conductor::set_landing(bool is_landing)
{
    landing = is_landing;
}

void Conductor::send_mode(const bool active)
{
    json send_doc;
    send_doc["snd"] = "jv";
    send_doc["dst"] = "pc";
    send_doc["typ"] = "mode";
    send_doc["mode"] = active ? 1 : 2; // TODO: change to enum

    // Echo the json packet
    std::string s = send_doc.dump();    
    send_payload(s);
}

void Conductor::send_landing(const bool active)
{
    json send_doc;
    send_doc["snd"] = "jv";
    send_doc["dst"] = "pc";
    send_doc["typ"] = "land";
    send_doc["land"] = active ? 1 : 2; // TODO: change to enum

    // Echo the json packet
    std::string s = send_doc.dump();
    send_payload(s);

}

// **********************************************************
// Send Timer
// **********************************************************

int Conductor::time_elapsed_ms()
{
    return (int)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_jv).count();
}

void Conductor::timer_handler(const boost::system::error_code& error)
{

    // Calculate a new expiry time relative to previous to prevent drift
    send_timer->expires_at(send_timer->expires_at() + std::chrono::milliseconds(CONTROL_LOOP_PERIOD_MS));
    // Restart the timer
    send_timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));


    if(!error)
    {

        pub_log_check("Send timer", DEBUG, true);

        if(!sent_initial) {
            send_mode(controller_activity);
            send_landing(landing);
            sent_initial = true;
        }

        if(controller_activity && alt_controller->isValidState()) {

            if(landing) {
                AltTarget_t targetTemp;
                if(alt_controller->getTarget(targetTemp)) {
                    targetTemp.z += 0.1*((double)CONTROL_LOOP_PERIOD_MS)/1000.0; targetTemp.z_dot = 0.1;
                    targetTemp.z = targetTemp.z > 0.0 ? targetTemp.z : 0.0;
                    alt_controller->setTarget(targetTemp);
                }
            }

            double chn_thr, chn_ele, chn_ail, chn_rud;
            AltState_t prop_alt_state = alt_estimator->getPropagatedStateEstimate_safe(time_elapsed_ms(), PROP_LIMIT);
            chn_thr = saturate(alt_controller->getControlTempState(prop_alt_state));
            // TODO: find a way to not set the arm channel
            std::array<double, 16> mixed_channels = {0, 0, chn_thr, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            send_channels(mixed_channels, false);

            if(files_open) {
                // header
                // "time_esp_ms,time_esp_prop,Delta_t_prop_ms,z_prop,z_dot_prop,chnThr,chnEle,chnAil,chnRud"
                file_sig << alt_estimator->getCurrentTimeEsp_ms() << "," << prop_alt_state.timeEsp_ms << "," << prop_alt_state.timeEsp_ms-alt_estimator->getCurrentTimeEsp_ms() << ","
                         << prop_alt_state.z << "," << prop_alt_state.z_dot << "," << chn_thr << "," << chn_ele << "," << chn_ail << "," << chn_rud << std::endl;
            }

        }

    }

}



void Conductor::send_channels(const std::array<double, 16> &channels, const bool response)
{

    // auto current_time = std::chrono::steady_clock::now();
    // std::string write_str;
    // std::ostringstream os;
    // os << "Time since epoch: " << (int)std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count() << " ms\r\n";
    // write_str = os.str();
    // prev_jv_time = current_time;

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
    for(int i=5; i < msp_data.size(); i++) {
        checksum ^= msp_data.at(i);
    } 
    msp_data.push_back(checksum);

    // Hard-code json
    std::string pre_string = "{\"snd\":\"pc\",\"dst\":\"fc\",\"typ\":\"msp\",\"rsp\":";
    pre_string += response ? "\"true\"" : "\"false\"";
    pre_string += ",\"ctrl\":\"true\",\"msp\":\"";
    std::string post_string = "\"}";

    std::vector<char> all_data(pre_string.begin(), pre_string.end());
    all_data.insert(all_data.end(), msp_data.begin(), msp_data.end());
    all_data.insert(all_data.end(), post_string.begin(), post_string.end());

    // Write to serial
    send_payload(all_data);

}



void Conductor::pub_log_check(const std::string &in_str, int log_level, bool send)
{
    if(log_level >= GLOBAL_LOG_LEVEL) {
        std::string lvl_str = "[]";
        if(log_level == ALL) { lvl_str = "[ALL] "; }
            else if(log_level == DEBUG) { lvl_str = "[DEBUG] "; }
            else if(log_level == INFO) { lvl_str = "[INFO] "; }
            else if(log_level == WARN) { lvl_str = "[WARN] "; }
            else if(log_level == ERROR) { lvl_str = "[ERROR] "; }
            else if(log_level == FATAL) { lvl_str = "[FATAL] "; }
            else if(log_level == OFF) { lvl_str = "[OFF] "; }

        if(file_log.is_open())
            file_log << lvl_str << in_str << std::endl;
        if(send)
            send_log(in_str, log_level);
    }
}

void Conductor::send_log(const std::string &in_str, int log_level)
{
    
    json send_doc;
    send_doc["snd"] = "jv";
    send_doc["dst"] = "pc";
    send_doc["typ"] = "log";
    send_doc["lvl"] = log_level;
    send_doc["log"] = in_str;

    // Echo the json packet
    std::string s = send_doc.dump();
    send_payload(s);

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

    files_open = status;

    // send error message if required

    return status;
}

void Conductor::close_files()
{
    file_log.close();
    alt_estimator->close_files();
    alt_controller->close_files();
    files_open = false;
}

void Conductor::set_file_suffix(std::string suffix_in)
{
    suffix = suffix_in;
    alt_estimator->set_file_suffix(suffix);
    alt_controller->set_file_suffix(suffix);
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
}

std::string Conductor::get_file_directory()
{
    return file_directory;
}
