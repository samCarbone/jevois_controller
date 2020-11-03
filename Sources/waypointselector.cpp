#include "waypointselector.h"

WaypointSelector::WaypointSelector()
{
    curr_wp_id = wp_seq.at(curr_seq_idx);
}

WaypointSelector::~WaypointSelector()
{
    close_files();
}

void WaypointSelector::updatePosition(const double x, const double y, const double z, long int time_ms)
{
    // Calculate position norm
    Eigen::Vector3d curr_pos(x,y,z);
    Eigen::Vector3d wp_loc(wp_x_e.at(curr_wp_id),wp_y_e.at(curr_wp_id),wp_z_e.at(curr_wp_id));
    Eigen::Vector3d r_error;
    r_error = wp_loc - curr_pos;
    double r_error_norm = r_error.norm();

    // Check if it i within the trigger distance
    if(r_error_norm <= trigger_dist) {

        // If it hasn't been triggered -- set the first triggered time
        if(!curr_triggered) {
            curr_triggered = true;
            triggerTime_ms = time_ms;
        }

        // Check if the current time - triggered time is >= loiter time
        if(time_ms - triggerTime_ms >= wp_loiter.at(curr_wp_id)) {

            // Move onto the next sequence ID and update curr waypoint id
            // If it is at the end of the sequence, then remain at this waypoint
            curr_seq_idx = curr_seq_idx + 1 == wp_seq.size() ? curr_seq_idx : curr_seq_idx + 1;
            curr_wp_id = wp_seq.at(curr_seq_idx);
            curr_triggered = false;

        }

    }

    // Save to file
    if(files_open) {
        // header
        // "time_pc_ms,curr_triggered,x_pl,y_pl,z_pl,x_wp,y_wp,z_wp,wpID"
        file_wp << time_ms << "," << curr_triggered << "," << x << "," << y << "," << z << ","
            << wp_x_e.at(curr_wp_id) << "," << wp_y_e.at(curr_wp_id) << "," << wp_z_e.at(curr_wp_id) << "," 
            << curr_wp_id << std::endl;
    }

}

int WaypointSelector::getWPID()
{
    return wp_seq.at(curr_seq_idx);
}

void WaypointSelector::getWPLoc(double &x_wp, double &y_wp, double &z_wp)
{
    x_wp = wp_x_e.at(curr_wp_id);
    y_wp = wp_y_e.at(curr_wp_id);
    z_wp = wp_z_e.at(curr_wp_id);
}

void WaypointSelector::getWPVel(double &vx_wp, double &vy_wp, double &vz_wp)
{
    vx_wp = wp_vx_e.at(curr_wp_id);
    vy_wp = wp_vy_e.at(curr_wp_id);
    vz_wp = wp_vz_e.at(curr_wp_id);
}

void WaypointSelector::getWPPsi(double &psi_wp)
{
    psi_wp = wp_psi.at(curr_wp_id);
}


void WaypointSelector::reset()
{
    curr_seq_idx = 0;
    curr_wp_id = wp_seq.at(curr_seq_idx);
    curr_triggered = false;
}

// **********************************************************************
// File Methods
// **********************************************************************

bool WaypointSelector::open_files() {
    if(!file_wp.is_open()) {
        std::string name_alt_est = file_directory + "/" + prefix + suffix + format;
        file_wp.open(name_alt_est, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }
    
    if(file_wp.is_open()) {
        file_wp << std::endl << header_wp << std::endl;
        files_open = true;
        return true;
    }
    else {
        return false;
    }
}

void WaypointSelector::close_files() {
    file_wp.close();
    files_open = false;
}

void WaypointSelector::set_file_suffix(std::string suffix_in) {
    suffix = suffix_in;
}

std::string WaypointSelector::get_file_suffix() {
    return suffix;
}

void WaypointSelector::set_file_directory(std::string directory_in) {
    file_directory = directory_in;
}

std::string WaypointSelector::get_file_directory()
{
    return file_directory;
}

