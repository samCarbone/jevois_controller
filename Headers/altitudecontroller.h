#ifndef ALTITUDECONTROLLER_H
#define ALTITUDECONTROLLER_H

#include <fstream>
#include <../eigen/Eigen/Dense>
#include <string>
#include <iostream>
#include <math.h>
#include "def.h"

class AltitudeController
{
public:
    // Methods
    AltitudeController();
    ~AltitudeController();
    void resetState(); //
//    void resetIntegral(); //
    void setTarget(const AltTarget_t target); //
    bool getTarget(AltTarget_t &target_out);
    void addEstState(const AltState_t newState); //
    double getControl(); //
    double getControlTempState(const AltState_t tempState); //
    bool isValidState();

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
    bool validStateSet = false;
    Eigen::Matrix<double, 3, 1> Dx; // [Delta z (m), Delta z_dot (mm/ms), Delta z_int (m.s)]
    Eigen::Matrix<double, 2, 1> x_target; // [z (m), z_dot (mm/ms)]
    long int currentTimeEsp_ms;
    long int currentTimePc_ms;
    bool targetIsSet = false;
    Eigen::Matrix<double, 1, 3> B; // [P D I]
    double thrSS = -3.5;
    double P = -7.178;
    double I = -2.5;
    double D = -12.5;

    // File save attributes
    std::ofstream file_alt_ctrl;
    bool files_open = false;
    const std::string header_alt_ctrl = "time_esp_ms,time_pc_ms,Delta_z,Delta_z_dot,Delta_z_int,u,P_z,D_z_dot,I_z_int,thrSS,P,D,I";
    const std::string prefix_alt_ctrl = "alt_ctrl_";
    const std::string format = ".txt";
    std::string suffix = "jv";
    std::string file_directory = ".";

};

#endif // ALTITUDECONTROLLER_H
