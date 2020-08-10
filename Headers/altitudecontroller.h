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
    void resetState();
    void resetIntegral();
    void addEstState();
    void addTempEstState();
    void setTarget();
    double getControl();
//    CtrlState_t getControlState();
//    CtrlState_t getControlTempState();

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
    bool validState = false;
    Eigen::Matrix<double, 3, 1> x; // [z (mm), z_dot (mm/ms), z_int (mm.s)]

    // File save attributes
    std::ofstream file_alt_ctrl;
    bool filesOpen = false;
    const std::string header_alt_ctrl = "time_esp_ms,time_pc_ms,z,z_dot,P_11,P_12,P_21,P_22,Delta_t_s,range_mm,sigma_mm,sigma_v";
    const std::string prefix_alt_ctrl = "alt_ctrl_";
    const std::string format = ".txt";
    std::string suffix = "temp";
    std::string fileDirectory = "";

};

#endif // ALTITUDECONTROLLER_H
