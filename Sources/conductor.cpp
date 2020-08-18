#include "conductor.h"

// **********************************************************
// Constructor/Destructor
// **********************************************************

Conductor::Conductor()
{
	Conductor("/dev/ttyS0");
}

Conductor::Conductor(std::string serial_port_name)
{

    // Construct and open the serial port
    esp_port = new boost::asio::serial_port(io, serial_port_name);
    esp_port->set_option(boost::asio::serial_port_base::baud_rate(57600));
    esp_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    esp_port->set_option(boost::asio::serial_port_base::character_size(8));
    esp_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    esp_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    
    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    serial_read_concat.reserve(256);

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
    set_file_directory("./logs");
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
        // Serial data available
        // Echo data back
        // serial_write_buffer = serial_read_buffer;
	    // esp_port->async_write_some( boost::asio::buffer(serial_write_buffer, bytes_transferred), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );
        serial_read_concat.insert(serial_read_concat.end(), serial_read_buffer.begin(), serial_read_buffer.begin()+bytes_transferred);

        int start, end;
        while(find_first_json(serial_read_concat, start, end)) {
            // Include the first character of the delimiter (i.e. the '}' of "}\r\n")
            end++;

            // Parse it
            parse_packet(serial_read_concat, start, end);
            
            // Now erase this json packet from serial_read_concat
            // Erase elements prior to this json packet
            serial_read_concat.erase(serial_read_concat.begin(), serial_read_concat.begin()+end+2); // removes the \r\n as well
        }


    }
    else {
        pub_log_check("Serial read error", ERROR, true);
    }

    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

}

bool Conductor::find_first_json(const std::vector<char> &inVec, int &start, int &end)
{

    bool found_start = false;
    start = 0;
    end = 0;
    std::array<char, 3> DELIM = {'}', '\r', '\n'};
    // Search for the start
    for(int i=0; i<inVec.size()-DELIM.size(); i++) {
        if(inVec.at(i) == '{') {
            start = i;
            found_start = true;
            break;
        }
    }

    if(found_start) {
        // Search for the end
        for(int i=start+1; i<inVec.size()-DELIM.size()+1; i++) {
            bool is_end = true;
            for(int j=0; j<DELIM.size()&&is_end; j++) {
                is_end &= inVec.at(i+j) == DELIM.at(j);
            }
            if(is_end) {
                end = i;
                return true;
            }
        }
    }
    

    return false;    

}

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
    for(int i=0; i<inVec.size()-chrs_2.size()-2-min_msp_len; i++) {

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

    if(found_start) {

        // Find the end. Note: this will find a closing quotation mark of
        // another element if it is missing for the msp message.
        // this means that the end position is >= real msp end
        for(int i=expected_end; i<inVec.size()-1; i++) {
            bool found_end = false;

            if(inVec.at(i) != chr_3)
                continue;

            for(auto &opt: chrs_option_4) {
                if(inVec.at(i+1) == opt) {
                    found_end = true;
                    break;
                }
            }
            if(found_end) {
                end = i;
                return true;
            }
        }

    }

    return false;

}

void Conductor::parse_packet(const std::vector<char> &inVec, const int start, const int end)
{
    // Could copy into a new vector for convenience
    std::vector<char> cut_vec(inVec.begin()+start, inVec.begin()+end);
    for (int i=0; i<cut_vec.size(); i++) {
        if(cut_vec.at(i) <= 31) {
            cut_vec.at(i) = 'x';
        }
    }

    /* Note: this does not support multiple msp messages
    within a single json packet. It will use the first msp message.
    */
    // MSP search ({,)"msp":"*"(,})
    std::vector<char> msp_vec;
    bool found_msp = false;
    int start_msp = 0;
    int end_msp = 0;
    if(find_first_msp(cut_vec, start_msp, end_msp)) {
            msp_vec.insert(msp_vec.begin(), cut_vec.begin()+start_msp+1, cut_vec.begin()+end_msp);
            cut_vec.erase(cut_vec.begin()+start_msp+1, cut_vec.begin()+end_msp);
            found_msp = true;
    }

    // Echo the cut vector
    // esp_port->async_write_some( boost::asio::buffer(cut_vec), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

    // Catch all exceptions
    // We want to keep the program running and just ignore invalid json packets
    try {
        json j_doc = json::parse(cut_vec);
        if(j_doc.is_null())
            return;

        // Echo the json packet
        // std::string s = j_doc.dump();
        // s += '\r\n';
        // esp_port->async_write_some( boost::asio::buffer(s), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

        if (j_doc["dst"] != "jv") {
            pub_log_check("Incorrect json dest", ERROR, true);
            return;
        }

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

        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;    
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
    if(mode_obj["mode"] == "start") {
        set_controller_activity(true);
        pub_log_check("Controller started", INFO, true);
    }
    else if(mode_obj["mode"] == "stop") {
        set_controller_activity(false);
        pub_log_check("Controller stopped", INFO, true);
    }
    else if(mode_obj["mode"] == "quit") {
        set_controller_activity(false);
        pub_log_check("Quit", INFO, true); // Might not send
        send_timer->cancel();
        esp_port->cancel();
        esp_port->close();
    }

    if(mode_obj["rsp"] == "true") {
        send_mode(controller_activity);
    }

}

void Conductor::parse_landing(const json &land_obj)
{
    if(land_obj["land"] == "true") {
        set_landing(true);
        pub_log_check("Landing started", INFO, true);
    }
    else if(land_obj["land"] == "false") {
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
    send_doc["mode"] = active ? "true" : "false";

    // Echo the json packet
    std::string s = send_doc.dump();
    s += "\r\n";
    esp_port->async_write_some( boost::asio::buffer(s), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

}

void Conductor::send_landing(const bool active)
{
    json send_doc;
    send_doc["snd"] = "jv";
    send_doc["dst"] = "pc";
    send_doc["land"] = active ? "true" : "false";

    // Echo the json packet
    std::string s = send_doc.dump();
    s += "\r\n";
    esp_port->async_write_some( boost::asio::buffer(s), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

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

    pub_log_check("Send timer done", INFO, true);

    if(!error)
    {

        if(controller_activity && alt_controller->isValidState()) {

            if(landing) {
                AltTarget_t targetTemp;
                if(alt_controller->getTarget(targetTemp)) {
                    targetTemp.z += 0.1*((double)CONTROL_LOOP_PERIOD_MS)/1000.0; targetTemp.z_dot = 0.1;
                    alt_controller->setTarget(targetTemp);
                }
            }

            double chn_thr, chn_ele, chn_ail, chn_rud;
            AltState_t prop_alt_state = alt_estimator->getPropagatedStateEstimate_safe(time_elapsed_ms(), PROP_TIMEOUT);
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
    std::string post_string = "\"}\r\n";

    std::vector<char> all_data(pre_string.begin(), pre_string.end());
    all_data.insert(all_data.end(), msp_data.begin(), msp_data.end());
    all_data.insert(all_data.end(), post_string.begin(), post_string.end());

    // Write to serial
    esp_port->async_write_some( boost::asio::buffer(all_data), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

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
    s += "\r\n";
    esp_port->async_write_some( boost::asio::buffer(s), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );

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
        std::string name_sig = file_directory + "/" + prefix_log + suffix + format;
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
    alt_estimator->set_file_directory(directory_in);
    alt_controller->set_file_directory(directory_in);
}

std::string Conductor::get_file_directory()
{
    return file_directory;
}
