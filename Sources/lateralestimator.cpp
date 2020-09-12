#include "lateralestimator.h"

void LateralEstimator::prior_ransac(const Eigen::VectorXd &Dt,
                const Eigen::VectorXd &Dx,
                const unsigned int iter, /* Number of iterations */
                const double sigma_thresh,  /* Threshold for squared error for a point to be included */
                const unsigned int select_n, /* Number of points used to create the model in each iteration */
                Eigen::Matrix<double, 2, 2> &P, /* Penalty matrix */
                double &x_off, double &v_off, /* Output offsets */
                bool &valid, /* Whether a valid model was found */
                const unsigned int min_num_matched=0, /* Minimum number of points which must be matched for the model to be valid */
                ) 
{

    // The minimum summed error for the iterations
    double epsilon_min = 0;
    bool error_set = false;

    // Declare matrices/vectors
    size_t max_len = Dx.size() <= Dt.size() ? Dx.size() : Dt.size();
    if(select_n > max_len) { select_n = max_len; }
    std::vector<size_t> indices(max_len);
    std::iota(indices.begin(), indices.end(), 0); 
    Eigen::VectorXd Dx_sample(select_n);
    Eigen::VectorXd Dt_sample(select_n);
    Eigen::MatrixXd::Ones(select_n, 2) X;
    Eigen::VectorXd b(2)
    Eigen::VectorXd coeffs(2);
    std::random_device rd; std::mt19937 g(rd());

    for(unsigned int it=0; it<iter; it++) {

        // Clear matrices/vectors if required
        // NOTE: not necessary to reset indices.
        
        /* 
        Select n random elements from the input data
        */

        // Array of randomly shuffled indices
        std::shuffle(indices.begin(), indices.end(), g);

        // Shuffle the input vectors and cut to size select_n
        for(size_t j=0; j<select_n; j++) {
            Dx_sample(j) = Dx(indices.at(j));
            Dt_sample(j) = Dt(indices.at(j));
        }

        /*
        Apply LS to the samples
        */
        // Populate matrices   
        X.col(1) = Dt_sample;
        const Eigen::VectorXd &Y = Dx_sample;

        // Perform LS
        // TODO: change to a templated function
        // TODO: investigate linalg options
        b = X.transpose()*Y;
        coeffs = (X.transpose()*X + P).colPivHouseholderQr().solve(b);

        // Sum of squared errors for all of the samples (for this iteration)
        double epsilon_it = 0;
        unsigned int num_matched = 0;
        
        for(size_t j=0; j<Dt; j++) {
            // Calculate the 2-norm of the error between the estimate and
            // actual Dx this sample
            double epsilon_j = math.pow(coeffs(1)* D_t(j) + coeffs(0) - D_x(j),2);
            // Add to the total error for this iteration
            if(epsilon_j > math.pow(sigma_th,2) { epsilon_it += sigma_th; }
            else { 
                epsilon_it += epsilon_j; 
                num_matched += 1;
            }
        }

        // Check if this has the lowest sum of squared errors compared to the other iterations
        if((!error_set || epsilon_it < epsilon_min) && num_matched >= min_num_matched) {
            epsilon_min = epsilon_it;
            error_set = true;
            valid = true;
            x_off = coeffs(0);
            v_off = coeffs(1);
        }

    }   

}