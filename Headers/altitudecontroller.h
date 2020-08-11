#ifndef ALTITUDECONTROLLER_H
#define ALTITUDECONTROLLER_H

#include <fstream>
#include <eigen/Eigen/Dense>
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
    void addEstState(const AltState_t newState); //
    double getControl(); //
    double getControlTempState(const AltState_t tempState); //

    // Attributes

    // File save methods
    bool openFiles();
    void closeFiles();
    void setFileSuffix(std::string suffix_in);
    std::string getFileSuffix();
    void setFileDirectory(std::string directory);
    std::string getFileDirectory();

private:
    // Methods

    // Attributes
    bool validStateSet = false;
    Eigen::Matrix<double, 3, 1> Dx; // [Delta z (m), Delta z_dot (mm/ms), Delta z_int (m.s)]
    Eigen::Matrix<double, 2, 1> x_target; // [z (m), z_dot (mm/ms)]
    int currentTimeEsp_ms;
    int currentTimePc_ms;
    bool targetIsSet = false;
    Eigen::Matrix<double, 1, 3> B; // [P D I]
    double thrSS = -47.8935;

    // File save attributes
    std::ofstream file_alt_ctrl;
    bool filesOpen = false;
    const std::string header_alt_ctrl = "time_esp_ms,time_pc_ms,Delta_z,Delta_z_dot,Delta_z_int,u,P_z,D_z_dot,I_z_int,thrSS,P,D,I";
    const std::string prefix_alt_ctrl = "alt_ctrl_";
    const std::string format = ".txt";
    std::string suffix = "temp";
    std::string fileDirectory = "";

};

#endif // ALTITUDECONTROLLER_H
