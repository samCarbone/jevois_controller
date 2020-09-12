// Test the prior ransac algorithm in the lateral estimator code

#include "matplotlibcpp.h"
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>
#include <../eigen/Eigen/Dense>
#include "lateralestimator.h"
#include <cmath>

namespace plt = matplotlibcpp;

int main()
{
    // Linear model parameters (y = mx + c)
    double m = 1.5;
    double c = 10;

    unsigned int n = 100;

    // truth values
    std::vector<double> x(n);
    for(int i=0; i<x.size(); i++) {
        x.at(i) = i/10.0;
    }
    std::vector<double> y(n);
    for(int i=0; i<y.size(); i++) {
        y.at(i) = m*x.at(i) + c;
    }

    // Simulate measurements
    // Make a random selection from the x vector
    num_samples = 20;
    num_samples = num_samples <= n ? num_samples : n;
    std::vector<double> x_meas(n);
    x_meas = x;
    std::random_device rd; std::mt19937 g(rd());
    std::shuffle(x_meas.begin(), x_meas.end(), g);
    x_meas.erase(x_meas.begin()+num_samples, x_meas.end());
    
    std::vector<double> y_meas(num_samples);
    for(int i=0; i<y_meas.size(); i++) {
        y_meas.at(i) = m*x_meas.at(i)+c;
    }

    // Create some outliers
    unsigned int num_outliers = 3;
    num_outliers = num_outliers <= y_meas.size() ? num_outliers : y_meas.size();
    unsigned int outlier_mag = 15;
    std::vector<size_t> outlier_indices(y_meas.size());
    std::iota(outlier_indices.begin(), outlier_indices.end(), 0);
    std::random_device rd; std::mt19937 gen(rd());
    std::shuffle(outlier_indices.begin(), outlier_indices.end(), g);
    outlier_indices.erase(outlier_indices.begin(), outlier_indices.begin()+num_outliers);
    std::uniform_int_distribution< u32 > distribute_sign( 1, 2 );
    std::uniform_real_distribution< u32 > distribute_real( 0,  outlier_mag);
    for(int i=0; i<outlier_indices.size(); i++) {
        y_meas.at(outlier_indices.at(i)) += distribute_sign(gen)*distribute_real(gen);
    }

    // Convert to Eigen vectors
    Eigen::VectorXd x_meas_e = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_meas.data(), x_meas.size());
    Eigen::VectorXd y_meas_e = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(y_meas.data(), y_meas.size());

    // Fit with a LS model
    Eigen::VectorXd b(2)
    Eigen::VectorXd coeffs(2);
    b = x_meas_e.transpose()*y_meas_e;
    coeffs = (x_meas_e.transpose()*x_meas_e).colPivHouseholderQr().solve(b);

    std::vector<double> y_ls(x.size());
    for(int i=0; i<y_ls.size(); i++) {
        y_ls.at(i) = coeffs(1)*x.at(i) + coeffs(0);
    }

    // Fit with ransac
    unsigned int iter = 100;
    unsigned double sigma_thresh = 9;
    unsigned double sample_prop = 0.8;
    unsigned int select_n = static_cast<unsigned int>(round(n*sample_prop));
    Eigen::Matrix<double, 2,2> P;
    P   << 0 << 0 
        << 0 << 0;
    double m_est = 0;
    double c_est = 0;
    bool valid = false;
    LateralEstimator estimator();
    estimator.prior_ransac(x_meas_e, y_meas_e, 100, select_n, P, c_est, m_est, valid, 0);
    std::vector<double> y_ran(x.size());
    for(int i=0; i<y_ran.size(); i++) {
        y_ran.at(i) = m_est*x.at(i) + c_est;
    }

    // Plot
    plt::figure_size(640, 640);
    plt::named_plot("Truth", x, y); // Plot the truth model
    plt::named_plot("Meas", x_meas, y_meas, 'ro'); // Plot measurements
    plt::named_plot("LS", x, y_ls); // Plot LS model
    plt::named_plot("RANSAC", x, y_ran); // Plot ransac model
    plt::legend();
    plt::show();

}
