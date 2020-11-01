#ifndef LATERALESTIMATOR_H
#define LATERALESTIMATOR_H

#include <../eigen/Eigen/Dense>
#include <../eigen/Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <random>
#include <numeric>
#include <algorithm>
#include <vector>
#include <deque>
#include <fstream>
#include "def.h"

class LateralEstimator
{
public:
    LateralEstimator();
    
    ~LateralEstimator();
    
    void add_attitude(const std::int16_t roll, const std::int16_t pitch, const std::int16_t yaw, const double z, const long int time_ms);

    void add_gate_obs(const Eigen::Vector3d &r_cam2gate, const Eigen::Vector3d &orient_c, const long int time_cap_ms);

    void get_position(long int time_ms, double &x, double &vx, double &y, double &vy, bool &valid, bool &warn_time, const long int prop_time_limit_ms=1000);

    void get_heading(int &yaw, bool &valid);

    void reset();

    void prior_ransac(const Eigen::VectorXd &Dt,
                    const Eigen::VectorXd &Dx,
                    const unsigned int iter, /* Number of iterations */
                    const double sigma_thresh,  /* Threshold for squared error for a point to be included */
                    unsigned int select_n, /* Number of points used to create the model in each iteration */
                    const Eigen::Matrix<double, 2, 2> &P, /* Penalty matrix */
                    double &x_off, /* Output offset (y-intercept) */
                    double &v_off, /* Output gradient offset */
                    bool &valid, /* Whether a valid model was found */
                    const unsigned int min_num_matched=0 /* Minimum number of points which must be matched for the model to be valid */
                    );

    // File save methods
    bool open_files();
    void close_files();
    void set_file_suffix(std::string suffix_in);
    std::string get_file_suffix();
    void set_file_directory(std::string directory_in);
    std::string get_file_directory();
                    
private:

    // Drag
    double Cdx = 0.5;
    double Cdy = 0.5;

    // True once there is a measurement
    bool has_measurement = false;

    // Deques
    std::deque<double> x_raw;
    std::deque<double> vx_raw;
    std::deque<double> y_raw;
    std::deque<double> vy_raw;
    std::deque<double> z_raw;
    std::deque<int> roll_d; // roll [-1800 : 1800] 1/10 deg
    std::deque<int> pitch_d; // pitch [-900 : 900] 1/10 deg
    std::deque<int> yaw_d; // yaw [-180 : 180] deg
    std::deque<long int> t_ms_raw;

    double x_off = 0;
    double vx_off = 0;
    double y_off = 0;
    double vy_off = 0;
    long int t_ms_off = 0;

    // yaw_true = yaw_raw + yaw_off
    double psi_off = 0;
    double psi_off_inst = 0; // Instantaneous psi offset

    /*one-gate*/
    // const std::vector<double> gates_x_e = {1}; // m
    // const std::vector<double> gates_y_e = {0}; // m
    // const std::vector<double> gates_z_e = {-0.5}; // m

    // const std::vector<double> gates_orient_x = {1};
    // const std::vector<double> gates_orient_y = {0};
    // const std::vector<double> gates_orient_z = {0};

    /*two-gates*/
    const std::vector<double> gates_x_e = {1.5,1.5}; // m
    const std::vector<double> gates_y_e = {0,-1.5}; // m
    const std::vector<double> gates_z_e = {-0.5,-0.5}; // m

    const std::vector<double> gates_orient_x = {1,-1};
    const std::vector<double> gates_orient_y = {0,0};
    const std::vector<double> gates_orient_z = {0,0};

    const double POS_ERROR_MAX_LIMIT = 2; // m
    const double PSI_ERROR_MAX_LIMIT = M_PI/4; // rad
    bool no_obs = true; // if true, ignore position error limit. This is in case there is a large xy before takeoff
                        // Once a correction has been calculated, this becomes false

    std::deque<long int> queue_t_ms;
    std::deque<double> queue_x_meas;
    std::deque<double> queue_x_raw;
    std::deque<double> queue_y_meas;
    std::deque<double> queue_y_raw;
    std::deque<double> queue_psi_error;

    static constexpr long int SLIDING_WINDOW_MS = 5000;
    static constexpr int MIN_SAMPLES_IN_WINDOW = 10; // Minimum number of samples in the window to perform regression

    static constexpr long int LIMIT_CAM_TIME_DIFF_MS = 50; // ms, limit between mis-match of camera and IMU times
    
    static void DCM_Cbe(const double phi, const double theta, const double psi, Eigen::Matrix<double,3,3> &DCM);

    void calc_offsets(double &_x_off, double &_vx_off, double &_y_off, double &_vy_off, double &_psi_off, double &_psi_dot_off, long int &_t_ms_off, bool &valid);

    static void calc_vec_z_rotation(const Eigen::Vector3d &a, /* Observed gate orientation vector in Earth frame */
                                        const Eigen::Vector3d &b, /* True gate orientation vector in Earth frame */
                                        double &psi /* True gate yaw - observed gate yaw */);

    // a mutex to control writes

    // a validity flag

    // Random number generator
    std::mt19937 * gen;

    // File save attributes
    std::ofstream file_lateral_raw_imu;
    std::ofstream file_lateral_cam;

    bool files_open = false;
    const std::string header_lateral_raw_imu = "time_ms,roll_dec,pitch_dec,yaw,psi_off,psi_corr,z,x_raw,y_raw,vx_raw,vy_raw,x_off_c,y_off_c,vx_off_c,vy_off_c";
    //
    const std::string header_lateral_cam = "time_cap_ms,r_cam2gate_cx,r_cam2gate_cy,r_cam2gate_cz,orient_gate_cx,orient_gate_cy,orient_gate_cz,t_match_ms,roll_dec_m,pitch_dec_m,x_raw_m,y_raw_m,z_raw_m,vx_raw_m,vy_raw_m,psi_raw_m,i_gate_match,min_pos_error,assoc_psi_error,r_origin2body_ex,r_origin2body_ey,r_origin2body_ez,t_ms_off,x_off,y_off,vx_off,vy_off,psi_off,psi_off_inst,n_queue,valid";
    //
    const std::string prefix_lateral_raw_imu = "lateral_raw_imu_";
    const std::string prefix_lateral_cam = "lateral_cam_";
    const std::string format = ".txt";
    std::string suffix = "jv";
    std::string file_directory = ".";
    
};

#endif // LATERALESTIMATOR_H