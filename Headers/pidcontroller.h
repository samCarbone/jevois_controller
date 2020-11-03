#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <fstream>
#include <../eigen/Eigen/Dense>
#include <string>
#include <iostream>
#include <math.h>
#include "def.h"

class PIDcontroller
{
public:
    // Methods
    PIDcontroller();
    PIDcontroller(const double P, const double I, const double D);
    ~PIDcontroller();
    void resetState();
    void setTarget(const double yr, const double yr_dot);
    bool getTarget(double &yr, double &yr_dot);
    void setGains(const double P, const double I, const double D);
    void getGains(double &P, double &I, double &D);
    void addMeas(const double y, const double y_dot, long int timePc_ms);
    double getControl();
    bool isValidState();

    // File save methods
    bool open_files();
    void close_files();
    void set_file_suffix(std::string suffix_in);
    std::string get_file_suffix();
    void set_file_prefix(std::string prefix_in);
    std::string get_file_prefix();
    void set_file_directory(std::string directory_in);
    std::string get_file_directory();

private:
    // Methods

    // Attributes
    bool validStateSet = false;
    Eigen::Matrix<double, 3, 1> yError; // [error (m), error_dot (mm/ms), error_int (m.s)]
    Eigen::Matrix<double, 2, 1> yTarget; // [yr (m), yr_dot (mm/ms)]
    long int currentTimePc_ms;
    bool targetIsSet = false;
    Eigen::Matrix<double, 1, 3> B; // [P D I]

    // File save attributes
    std::ofstream file_ctrl;
    bool files_open = false;
    const std::string header_ctrl = "time_pc_ms,error,error_dot,error_int,u,P_e,D_e_dot,I_e_int,P,D,I";
    const std::string format = ".txt";
    std::string suffix = "jv";
    std::string prefix = "pidctrl_";
    std::string file_directory = ".";

};

#endif // PIDCONTROLLER_H
