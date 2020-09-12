#ifndef LATERALESTIMATOR_H
#define LATERALESTIMATOR_H

#include <../eigen/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <cstdint>
#include <random>
#include <numeric>
#include <algorithm>
#include <vector>
#include "def.h"

class LateralEstimator
{
public:
    LateralEstimator();
    ~LateralEstimator();
    // 
    void add_attitude(std::uint16_t roll, std::uint16_t pitch, std::uint16_t yaw);

    void add_gate();

    void reset();

    void position();

    void heading();

    void is_valid();

// private:
    //

    double x; // Or a vector
    double y;
    
    // a mutex to control writes

    // a validity flag

    // 

    static void prior_ransac(const Eigen::VectorXd &Dt,
                    const Eigen::VectorXd &Dx,
                    const unsigned int iter, /* Number of iterations */
                    const double sigma_thresh,  /* Threshold for squared error for a point to be included */
                    const unsigned int select_n, /* Number of points used to create the model in each iteration */
                    Eigen::Matrix<double, 2, 2> &P, /* Penalty matrix */
                    double &x_off, double &v_off, /* Output offsets */
                    bool &valid, /* Whether a valid model was found */
                    const unsigned int min_num_matched=0, /* Minimum number of points which must be matched for the model to be valid */
                    );
    


};

#endif // LATERALESTIMATOR_H