#ifndef WAYPOINTSELECTOR_H
#define WAYPOINTSELECTOR_H

#include <fstream>
#include <../eigen/Eigen/Dense>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include "def.h"

class WaypointSelector
{
public:
    // Methods
    WaypointSelector();
    ~WaypointSelector();
    void updatePosition(const double x, const double y, const double z, long int time_ms); // Update the platform's position
    unsigned int getWPID();
    void getWPLoc(double &x_wp, double &y_wp, double &z_wp);
    void getWPVel(double &vx_wp, double &vy_wp, double &vz_wp);
    void getWPPsi(double &psi_wp);
    void reset();

    // File save methods
    bool open_files();
    void close_files();
    void set_file_suffix(std::string suffix_in);
    std::string get_file_suffix();
    void set_file_directory(std::string directory_in);
    std::string get_file_directory();

private:

    // Attributes
    unsigned int curr_seq_idx = 0; // Current sequence index
    unsigned int curr_wp_id; // Current waypoint id

    // Waypoint locations
    const std::vector<double> wp_x_e = {0,3,4,4,4,2,1,1,1}; // m
    const std::vector<double> wp_y_e = {0,0,0,0,-1,-1,-1,-1,0}; // m
    const std::vector<double> wp_z_e = {-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5}; // m

    // Wapoint velocity targets
    const std::vector<double> wp_vx_e = {0,0.2,0,0,0,-0.2,0,0,0}; // m/s
    const std::vector<double> wp_vy_e = {0,0,0,0,0,0,0,0,0}; // m/s
    const std::vector<double> wp_vz_e = {0,0,0,0,0,0,0,0,0}; // m/s

    // Wapoint orientation targets
    const std::vector<double> wp_psi = {0,0,0,-2.356,-3.142,-3.142,-3.142,0.7854,0}; // rad

    // Waypoint loiter times
    const std::vector<long int> wp_loiter = {15000,500,3000,3000,5000,500,3000,3000,5000}; // ms

    // Waypoint sequence
    const std::vector<int> wp_seq = {0,1,2,3,4,5,6,7,8,0}; // allows arbitrary sequences of the waypoints

    // Trigger distance
    const double trigger_dist = 0.5; // m
    bool curr_triggered = false; // If the current waypoint has been triggered
    long int triggerTime_ms = 0; // ms, time that the current waypoint was triggered

    // File save attributes
    std::ofstream file_wp;
    bool files_open = false;
    const std::string header_wp = "time_pc_ms,curr_triggered,x_pl,y_pl,z_pl,x_wp,y_wp,z_wp,wpID";
    const std::string format = ".txt";
    std::string suffix = "jv";
    std::string prefix = "wp_";
    std::string file_directory = ".";

};

#endif // WAYPOINTSELECTOR_H
