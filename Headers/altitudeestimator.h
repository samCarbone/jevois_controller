#ifndef ALTITUDEESTIMATOR_H
#define ALTITUDEESTIMATOR_H

#include <fstream>
#include <../eigen/Eigen/Dense>
#include <string>
#include <iostream>
#include <math.h>
#include "def.h"

class AltitudeEstimator
{
public:
    // Methods
    AltitudeEstimator();
    ~AltitudeEstimator();
    void resetStateEstimate();
    void addRangeMeasurement(RangingData_t rangeData);
    AltState_t getStateEstimate();
    AltState_t getPropagatedStateEstimate(long int newTimePc_ms);
    AltState_t getPropagatedStateEstimate_safe(long int newTimePc_ms, long int limitDelta_ms, bool &error, std::string &error_str);
    long int getCurrentTimePc_ms();
    long int getCurrentTimeEsp_ms();

    // Attributes

    // File save methods
    bool open_files();
    void close_files();
    void set_file_suffix(std::string suffix_in);
    std::string get_file_suffix();
    void set_file_directory(std::string directory_in);
    std::string get_file_directory();

private:
    // Methods

    // Attributes
    long int currentTimePc_ms;
    long int currentTimeEsp_ms;
    Eigen::Matrix<double, 2, 1> x; // [z (mm), z_dot (mm/ms)]
    Eigen::Matrix<double, 2, 2> P;
    double sigma_v = 100; // mm/s, process uncertainty in velocity
    bool hasPreviousMeasurement = false;

    // File save attributes
    std::ofstream file_alt_est;
    bool files_open = false;
    const std::string header_alt_est = "time_esp_ms,time_pc_ms,z,z_dot,P_11,P_12,P_21,P_22,Delta_t_ms,sigma_v,range_mm,sigma_mm,signal_rate,ambient_rate,eff_spad_count,status";
    const std::string prefix_alt_est = "alt_est_";
//    std::string header_alt_meas = "time_esp_ms,time_recv_ms,range_mm,sigma_mm,signal_rate,ambient_rate,eff_spad_count,status";
//    std::string prefix_alt_meas = "alt_meas_";
    const std::string format = ".txt";
    std::string suffix = "jv";
    std::string file_directory = ".";


};

#endif // ALTITUDEESTIMATOR_H
