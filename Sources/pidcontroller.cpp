#include "pidcontroller.h"

PIDcontroller::PIDcontroller()
{
    yError << 0, 0, 0;
    yTarget << 0, 0;
    resetState();
    B(0,0) = 0; B(0,1) = 0; B(0,2) = 0; // [P D I]
}

PIDcontroller::PIDcontroller(const double P, const double I, const double D)
{
    yError << 0, 0, 0;
    yTarget << 0, 0;
    resetState();
    B(0,0) = P; B(0,1) = D; B(0,2) = I; // [P D I]
}

PIDcontroller::~PIDcontroller()
{
    close_files();
}

void PIDcontroller::resetState()
{
    yError(0,0) = 0; yError(1,0) = 0; yError(2,0) = 0;
    validStateSet = false;
}

void PIDcontroller::setTarget(const double yr, const double yr_dot)
{
    yTarget(0,0) = yr;
    yTarget(1,0) = yr_dot;
    targetIsSet = true;
}

bool PIDcontroller::getTarget(double &yr, double &yr_dot)
{
    if(targetIsSet) {
        yr = yTarget(0,0);
        yr_dot = yTarget(1,0);
        return true;
    }
    return false;
}

void PIDcontroller::setGains(const double P, const double I, const double D)
{
    B(0,0) = P; B(0,1) = D; B(0,2) = I; // [P D I]
}

void PIDcontroller::getGains(double &P, double &I, double &D)
{
    P = B(0,0);
    D = B(0,1);
    I = B(0,2);
}

void PIDcontroller::addMeas(const double y, const double y_dot, long int timePc_ms)
{
    if(targetIsSet) {
        double yError_new = yTarget(0,0) - y;
        if(validStateSet) {
            long int Delta_t_ms = timePc_ms - currentTimePc_ms;

            // Id Delta_t is negative, instead set Delta_t to 0
            if(Delta_t_ms < 0) {
                Delta_t_ms = 0;
            }

            // Setting integral
            yError(2,0) = (yError_new + yError(0,0))/2 * Delta_t_ms/1000.0 + yError(2,0);
        }
        else {
            // If this is the first measurement, set integral to zero
            yError(2,0) = 0;
            validStateSet = true;
        }
        yError(0,0) = yError_new;
        yError(1,0) = yTarget(1,0) - y_dot;
        currentTimePc_ms = timePc_ms;
    }
}

double PIDcontroller::getControl()
{
    if(validStateSet) {
        double u = (B*yError)(0,0);

        if(files_open) {
            // header
            // "time_pc_ms,error,error_dot,error_int,u,P_e,D_e_dot,I_e_int,P,D,I"
            file_ctrl << currentTimePc_ms << "," << yError(0,0) << "," << yError(1,0) << "," << yError(2,0) << ","
                << u << "," << B(0,0)*yError(0,0) << "," << B(0,1)*yError(1,0) << "," << B(0,2)*yError(2,0) << ","
                << B(0,0) << "," << B(0,1) << "," << B(0,2) << std::endl;
        }

        return u;
    }

    return 0;
}


bool PIDcontroller::isValidState()
{
    return validStateSet;
}

// **********************************************************************
// File Methods
// **********************************************************************

bool PIDcontroller::open_files() {
    if(!file_ctrl.is_open()) {
        std::string name_alt_est = file_directory + "/" + prefix + suffix + format;
        file_ctrl.open(name_alt_est, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }
    
    if(file_ctrl.is_open()) {
        file_ctrl << std::endl << header_ctrl << std::endl;
        files_open = true;
        return true;
    }
    else {
        return false;
    }
}

void PIDcontroller::close_files() {
    file_ctrl.close();
    files_open = false;
}

void PIDcontroller::set_file_suffix(std::string suffix_in) {
    suffix = suffix_in;
}

std::string PIDcontroller::get_file_suffix() {
    return suffix;
}

void PIDcontroller::set_file_prefix(std::string prefix_in) {
    prefix = prefix_in;
}

std::string PIDcontroller::get_file_prefix() {
    return prefix;
}

void PIDcontroller::set_file_directory(std::string directory_in) {
    file_directory = directory_in;
}

std::string PIDcontroller::get_file_directory()
{
    return file_directory;
}

