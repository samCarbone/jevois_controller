#ifndef ALTITUDEESTIMATOR_H
#define ALTITUDEESTIMATOR_H

#include <fstream>
#include <eigen/Eigen/Dense>
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
    AltState_t getPropagatedStateEstimate(int newTimePc_ms);
    AltState_t getPropagatedStateEstimate_safe(int newTimePc_ms, int limitDelta_ms);
    int getCurrentTimePc_ms();
    int getCurrentTimeEsp_ms();

    // Attributes

    // File save methods
    bool openFiles();
    void closeFiles();
    void setSuffix(std::string suffix_in);
    std::string getSuffix();
    void setFileDirectory(std::string directory);
    std::string getFileDirectory();

private:
    // Methods

    // Attributes
    int currentTimePc_ms;
    int currentTimeEsp_ms;
    Eigen::Matrix<double, 2, 1> x; // [z (mm), z_dot (mm/ms)]
    Eigen::Matrix<double, 2, 2> P;
    double sigma_v = 100; // mm/s, process uncertainty in velocity
    bool hasPreviousMeasurement = false;

    // File save attributes
    std::ofstream file_alt_est;
    bool filesOpen = false;
    const std::string header_alt_est = "time_esp_ms,time_pc_ms,z,z_dot,P_11,P_12,P_21,P_22,Delta_t_ms,sigma_v,range_mm,sigma_mm,signal_rate,ambient_rate,eff_spad_count,status";
    const std::string prefix_alt_est = "alt_est_";
//    std::string header_alt_meas = "time_esp_ms,time_recv_ms,range_mm,sigma_mm,signal_rate,ambient_rate,eff_spad_count,status";
//    std::string prefix_alt_meas = "alt_meas_";
    const std::string format = ".txt";
    std::string suffix = "temp";
    std::string fileDirectory = "";


};

#endif // ALTITUDEESTIMATOR_H
