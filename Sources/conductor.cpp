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

    prev_jv_time = std::chrono::steady_clock::now();

    serial_read_concat.reserve(256);

    // Setup serial read handler
    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    // Setup timer
    timer = new boost::asio::high_resolution_timer(io, std::chrono::milliseconds(control_loop_period_ms));
    timer->async_wait(boost::bind(&Conductor::timer_handler, this, boost::asio::placeholders::error));

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
        // serial_write_buffer = serial_read_buffer;
	    // esp_port->async_write_some( boost::asio::buffer(serial_write_buffer, bytes_transferred), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );
        serial_read_concat.insert(serial_read_concat.end(), serial_read_buffer.begin(), serial_read_buffer.begin()+bytes_transferred);

        int start, end;
        while(find_first_set(serial_read_concat, start, end)) {
            // Include the first character of the delimiter (i.e. the '}' of "}\n")
            end++;

            // Parse it
            parse_packet(serial_read_concat, start, end);
            
            // Now erase this json packet from serial_read_concat
            serial_read_concat.erase(serial_read_concat.begin()+start, serial_read_concat.begin()+end+1); // removes the \n as well
        }


    }

    esp_port->async_read_some(boost::asio::buffer(serial_read_buffer), boost::bind(&Conductor::serial_read_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

}

bool Conductor::find_first_set(const std::vector<char> &inVec, int &start, int &end)
{

    bool found_start = false;

    // Search for the start
    for(int i=0; i<inVec.size(); i++) {
        if(inVec.at(i) == '{') {
            start = i;
            found_start = true;
            break;
        }
    }

    if(found_start) {
        // Search for the end
    	std::array<char, 2> DELIM = {'}', '\n'};
        for(int i=start; i<inVec.size()-DELIM.size()+1; i++) {
            bool is_end = true;
            for(int j=0; j<DELIM.size(); j++) {
                is_end &= serial_read_concat.at(i+j) == DELIM.at(j);
            }
            if(is_end) {
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
    const std::vector<char> cut_vec(inVec.begin()+start, inVec.begin()+end);
    for (char element : cut_vec) {
        if(element < 0 || element > 127) {
            element = 'x';
        }
    }

    // Echo the cut vector
    esp_port->async_write_some( boost::asio::buffer(cut_vec), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );


    // Catch all exceptions
    // We want to keep the program running
    try {
        json j_doc = json::parse(cut_vec);
        if(j_doc.is_null())
            return;

        if (j_doc["dst"] != "jv") {return;}

        if(j_doc["snd"] == "esp") {

            if(j_doc["typ"] == "alt") {
                // parseAltitude(j_doc["alt"]);
            }
            
            else if(j_doc["typ"] == "msp") {
                // Need to search for the msp bytes
            }

        }

        else if (j_doc["snd"] == "pc") {

            if(j_doc["typ"] == "mode") {
                // parseMode(j_doc);
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
        auto current_time = std::chrono::steady_clock::now();
        std::string write_str;
        std::ostringstream os;
        os << "Time diff: " << std::chrono::duration_cast<std::chrono::microseconds>(current_time - prev_jv_time).count() << " us\r\n";
        write_str = os.str();
        prev_jv_time = current_time;
        esp_port->async_write_some( boost::asio::buffer(write_str), boost::bind(&Conductor::serial_write_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred) );


        // Timer expired
    }

}

// void Conductor::parseAltitude(const json &alt_obj)
// {
//     if(alt_obj.contains("sigrt") && alt_obj.contains("ambrt") && alt_obj.contains("sigma")
//             && alt_obj.contains("spad") && alt_obj.contains("range") && alt_obj.contains("time")
//             && alt_obj.contains("status") )
//     {
//         RangingData_t altData;
//         altData.signal_rate = alt_obj["sigrt"].get<double>();
//         altData.ambient_rate = alt_obj["ambrt"].get<double>();
//         altData.sigma_mm = alt_obj["sigma"].get<double>();
//         altData.eff_spad_count = alt_obj["spad"].get<double>()/256; // divide by 256 for real value
//         altData.range_mm = alt_obj["range"].get<int>();
//         altData.timeEsp_ms = alt_obj["time"].get<int>();
//         altData.status = alt_obj["status"].get<int>();
// 	auto now = std::chrono::steady_clock::now();
//         altData.timePc_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

//         // Update state estimate
//         altEstimator->addRangeMeasurement(altData);
//         AltState_t estimatedState = altEstimator->getStateEstimate();

//         // Update controller
//         if(controllerActive) {
//             altController->addEstState(estimatedState);
//         }

//     }

// }

// void Conductor::parseMode(const json &mode_obj)
// {
//     if(mode_obj["mode"] == "start") {}
//     else if(mode_obj["mode"] == "stop") {}
//     else if(mode_obj["mode"] == "quit") {}

// }
