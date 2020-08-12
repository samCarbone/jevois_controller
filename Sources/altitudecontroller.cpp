#include "altitudecontroller.h"

AltitudeController::AltitudeController()
{
    resetState();
    B(0,0) = -10.178; B(0,1) = -4.36569; B(0,2) = -1.76181; // [P D I]

}

AltitudeController::~AltitudeController()
{
    close_files();
}


void AltitudeController::resetState()
{
    Dx.fill(0);
    validStateSet = false;
}

//void AltitudeController::resetIntegral()
//{
//    Dx(2,0) = 0;
//}

void AltitudeController::setTarget(const AltTarget_t target)
{
    x_target(0,0) = target.z;
    x_target(1,0) = target.z_dot;
    targetIsSet = true;
}

bool AltitudeController::getTarget(AltTarget_t &target_out)
{
    if(targetIsSet) {
        target_out.z = x_target(0,0);
        target_out.z_dot = x_target(1,0);
        return true;
    }
    return false;
}

void AltitudeController::addEstState(const AltState_t newState)
{
    if(targetIsSet && newState.valid) {
        double Dx0_new = x_target(0,0) - newState.z;
        if(validStateSet) {
            Dx(2,0) = (Dx0_new + Dx(0,0))/2 * (newState.timeEsp_ms - currentTimeEsp_ms)/1000.0 + Dx(2,0);
        }
        else {
            validStateSet = true;
        }
        Dx(0,0) = Dx0_new;
        Dx(1,0) = x_target(1,0) - newState.z_dot;
        currentTimeEsp_ms = newState.timeEsp_ms;
        currentTimePc_ms = newState.timePc_ms;
    }
}

double AltitudeController::getControl()
{
    if(validStateSet) {
        double u = (B*Dx)(0,0) + thrSS;
        return u;

        if(files_open) {
            // header
            // "time_esp_ms,time_pc_ms,Delta_z,Delta_z_dot,Delta_z_int,u,P_z,D_z_dot,I_z_int,thrSS,P,D,I"
            file_alt_ctrl << currentTimeEsp_ms << "," << currentTimePc_ms << "," << Dx(0,0) << "," << Dx(1,0) << "," << Dx(2,0) << ","
                          << u << "," << B(0,0)*Dx(0,0) << "," << B(0,1)*Dx(1,0) << "," << B(0,2)*Dx(2,0) << "," << thrSS << ","
                          << B(0,0) << "," << B(0,1) << "," << B(0,2) << std::endl;
        }

    }

    return -100;
}

double AltitudeController::getControlTempState(const AltState_t tempState)
{
    if(validStateSet && tempState.valid) {
        Eigen::Matrix<double, 3, 1> Dx_temp;
        Dx_temp(0,0) = x_target(0,0) - tempState.z;
        Dx_temp(1,0) = x_target(1,0) - tempState.z_dot;
        Dx_temp(2,0) = (Dx_temp(0,0) + Dx(0,0))/2 * (tempState.timeEsp_ms - currentTimeEsp_ms)/1000.0 + Dx(2,0);

        double u = (B*Dx_temp)(0,0) + thrSS;

        if(files_open) {
            // header
            // "time_esp_ms,time_pc_ms,Delta_z,Delta_z_dot,Delta_z_int,u,P_z,D_z_dot,I_z_int,thrSS,P,D,I"
            file_alt_ctrl << tempState.timeEsp_ms << "," << tempState.timePc_ms << "," << Dx_temp(0,0) << "," << Dx_temp(1,0) << "," << Dx_temp(2,0) << ","
                          << u << "," << B(0,0)*Dx_temp(0,0) << "," << B(0,1)*Dx_temp(1,0) << "," << B(0,2)*Dx_temp(2,0) << "," << thrSS << ","
                          << B(0,0) << "," << B(0,1) << "," << B(0,2) << std::endl;
        }

        return u;
    }
    return -100;
}


bool AltitudeController::isValidState()
{
    return validStateSet;
}

// **********************************************************************
// File Methods
// **********************************************************************

bool AltitudeController::open_files() {
    if(!file_alt_ctrl.is_open()) {
        std::string name_alt_est = file_directory + "/" + prefix_alt_ctrl + suffix + format;
        file_alt_ctrl.open(name_alt_est, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }
    
    if(file_alt_ctrl.is_open()) {
        file_alt_ctrl << std::endl << header_alt_ctrl << std::endl;
        files_open = true;
        return true;
    }
    else {
        return false;
    }
}

void AltitudeController::close_files() {
    file_alt_ctrl.close();
    files_open = false;
}

void AltitudeController::set_file_suffix(std::string suffix_in) {
    suffix = suffix_in;
}

std::string AltitudeController::get_file_suffix() {
    return suffix;
}

void AltitudeController::set_file_directory(std::string directory_in) {
    file_directory = directory_in;
}

std::string AltitudeController::get_file_directory()
{
    return file_directory;
}

