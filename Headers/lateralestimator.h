#ifndef LATERALESTIMATOR_H
#define LATERALESTIMATOR_H

#include <../eigen/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <random>
#include <numeric>
#include <algorithm>
#include <vector>
#include <deque>
#include "def.h"

class LateralEstimator
{
public:
    LateralEstimator();
    
    ~LateralEstimator();
    
    void add_attitude(std::uint16_t roll, std::uint16_t pitch, std::uint16_t yaw, double z, long int time_ms);

    void add_gate_obs(Eigen::Vector3d &r_cam2gate, Eigen::Vector3d &orient_c, long int time_cap_ms);

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
                    
private:
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

    const std::vector<double> gates_x_e = {1}; // m
    const std::vector<double> gates_y_e = {0}; // m
    const std::vector<double> gates_z_e = {-0.5}; // m

    const std::vector<double> gates_orient_x = {1};
    const std::vector<double> gates_orient_y = {0};
    const std::vector<double> gates_orient_z = {0};

    std::deque<long int> queue_t_ms;
    std::deque<double> queue_x_meas;
    std::deque<double> queue_x_raw;
    std::deque<double> queue_y_meas;
    std::deque<double> queue_y_raw;

    static constexpr long int SLIDING_WINDOW_MS = 5000;
    static constexpr int MIN_SAMPLES_IN_WINDOW = 10; // Minimum number of samples in the window to perform regression

    static constexpr long int LIMIT_CAM_TIME_DIFF_MS = 50; // ms, limit between mis-match of camera and IMU times
    
    static void DCM_Cbe(const double phi, const double theta, const double psi, Eigen::Matrix<double,3,3> &DCM);

    void calc_correction(double &_x_off, double &_vx_off, double &_y_off, double &_vy_off, long int &_t_ms_off, bool &valid);

    // a mutex to control writes

    // a validity flag

    // 
    std::mt19937 * gen;

    

};

#endif // LATERALESTIMATOR_H