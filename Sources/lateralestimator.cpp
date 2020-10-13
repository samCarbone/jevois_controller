#include "lateralestimator.h"

LateralEstimator::LateralEstimator()
{
    std::random_device rd; 
    gen = new std::mt19937(rd());
    x_raw.assign(500,0);
    vx_raw.assign(500,0);
    y_raw.assign(500,0);
    vy_raw.assign(500,0);
    z_raw.assign(500,0);
    roll_d.assign(500,0);
    pitch_d.assign(500,0);
    yaw_d.assign(500,0);
    t_ms_raw.assign(500,0);
}

LateralEstimator::~LateralEstimator() 
{
    close_files();
    delete gen;
}

void LateralEstimator::prior_ransac(const Eigen::VectorXd &Dt,
                const Eigen::VectorXd &Dx,
                const unsigned int iter, /* Number of iterations */
                const double sigma_thresh,  /* Threshold for error magnitude for a point to be included */
                unsigned int select_n, /* Number of points used to create the model in each iteration */
                const Eigen::Matrix<double, 2, 2> &P, /* Penalty matrix */
                double &x_off, double &v_off, /* Output offsets */
                bool &valid, /* Whether a valid model was found */
                const unsigned int min_num_matched /* Minimum number of points which must be matched for the model to be valid */
                ) 
{

    // The minimum summed error for the iterations
    double epsilon_min = 0;
    bool error_set = false;
    valid = false;
    x_off = 0;
    v_off = 0;

    // Declare matrices/vectors
    size_t max_len = Dx.size() <= Dt.size() ? Dx.size() : Dt.size();
    if(select_n > max_len) { select_n = max_len; }
    std::vector<size_t> indices(max_len);
    std::iota(indices.begin(), indices.end(), 0); 
    Eigen::VectorXd Dx_sample(select_n);
    Eigen::VectorXd Dt_sample(select_n);
    Eigen::MatrixXd X(select_n, 2);
    X.setOnes(select_n, 2);
    Eigen::VectorXd b(2);
    Eigen::VectorXd coeffs(2);
    

    for(unsigned int it=0; it<iter; it++) {

        // Clear matrices/vectors if required
        // NOTE: not necessary to reset indices.
        
        /* 
        Select n random elements from the input data
        */

        // Array of randomly shuffled indices
        std::shuffle(indices.begin(), indices.end(), *gen);

        // Shuffle the input vectors and cut to size select_n
        for(unsigned int j=0; j<select_n; j++) {
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
        
        for(unsigned int j=0; j<Dt.size(); j++) {
            // Calculate the 2-norm of the error between the estimate and
            // actual Dx this sample
            double epsilon_j = std::pow(coeffs(1)* Dt(j) + coeffs(0) - Dx(j),2);
            // Add to the total error for this iteration
            if(epsilon_j > std::pow(sigma_thresh,2)) { epsilon_it += sigma_thresh; }
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

void LateralEstimator::add_attitude(const std::int16_t roll, const std::int16_t pitch, const std::int16_t yaw, const double z, const long int time_ms)
{
    // roll [-1800 : 1800] 1/10 deg
    // pitch [-900 : 900] 1/10 deg
    // yaw [-180 : 180] deg
    double psi = yaw/180.0 * M_PI;
    double theta = pitch/1800.0 * M_PI;
    double phi = roll/1800.0 * M_PI;
    Eigen::Matrix<double,3,3> Ceb;
    DCM_Cbe(phi, theta, psi, Ceb);
    Ceb.transposeInPlace(); // Body to Earth frame transformation
    Eigen::Vector3d Ftu_b; // Thrust unit vector in body frame
    Eigen::Vector3d Ftu_e; // Thrust unit vector in Earth frame
    Eigen::Vector3d Ft_e; // Thrust vector (m/s^2) in Earth frame
    Ftu_b << 0, 0, -1;
    Ftu_e = Ceb*Ftu_b;
    Ft_e = -9.81/Ftu_e(2) * Ftu_e;

    // Propagate the state (without correction)

    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 0, 1,
        0, -0.5;
    B << 0, 1;

    Eigen::Vector2d x_prev; x_prev << x_raw.back(), vx_raw.back();
    Eigen::Vector2d y_prev; y_prev << y_raw.back(), vy_raw.back();
    Eigen::Vector2d x_i;
    Eigen::Vector2d y_i;
    double T_s = (time_ms - t_ms_raw.back())/1000.0; // s

    // Set T_s to 0 if this is the first measurement
    if(!has_measurement) {
        T_s = 0;
        has_measurement = true; // Set measurement received flag
    }

    x_i = (Eigen::Matrix2d::Identity() + A*T_s)*x_prev + T_s*Ft_e(0)*B;
    y_i = (Eigen::Matrix2d::Identity() + A*T_s)*y_prev + T_s*Ft_e(1)*B;

    // Update deques
    x_raw.erase(x_raw.begin());
    x_raw.push_back(x_i(0));
    vx_raw.erase(vx_raw.begin());
    vx_raw.push_back(x_i(1));
    y_raw.erase(y_raw.begin());
    y_raw.push_back(y_i(0));
    vy_raw.erase(vy_raw.begin());
    vy_raw.push_back(y_i(1));
    z_raw.erase(z_raw.begin());
    z_raw.push_back(z);
    roll_d.erase(roll_d.begin());
    roll_d.push_back(roll);
    pitch_d.erase(pitch_d.begin());
    pitch_d.push_back(pitch);
    yaw_d.erase(yaw_d.begin());
    yaw_d.push_back(yaw);
    t_ms_raw.erase(t_ms_raw.begin());
    t_ms_raw.push_back(time_ms);

    // File writes
    if(files_open) {
        // header
        // time_ms,roll_dec,pitch_dec,yaw,z,x_raw,y_raw,vx_raw,vy_raw,x_off_c,y_off_c,vx_off_c,vy_off_c
        file_lateral_raw_imu << time_ms << "," << roll << "," << pitch << "," << yaw << "," << z << "," 
                            << x_raw.back() << "," << y_raw.back() << "," << vx_raw.back() << "," << vy_raw.back() << ","
                            << x_off << "," << y_off << "," << vx_off << "," << vy_off
                            << std::endl;
    }

}

void LateralEstimator::add_gate_obs(const Eigen::Vector3d &r_cam2gate_c, const Eigen::Vector3d &orient_gate_c, const long int time_cap_ms)
{

    // SAFE TO RUN IN PARALLEL

    /*
    Class variables which are read:
    has_measurement
    t_ms_raw
    yaw_d
    pitch_d
    roll_d
    *_raw
    queue_t_ms
    */

    /*
    Class variables which are written to:
    queue
    _off
    */

   // Format:
   // r_first2second_frame: from first to second in frame
   // orient_thing_frame: orientation of thing in frame


    // r_to_gate: vector to from the body to the gate in the camera frame
    // orient_centre: vector out from the centre of the gate (gate orientation) in the camera frame

    // Only process once there is at least one imu measurement
    if(!has_measurement) {
        return;
    }

    // Rotate vectors to the body frame
    Eigen::Matrix3d Cbc; // Camera to body frame transformation
    Cbc << 0, 0, 1,
           1, 0, 0,
           0, 1, 0;

    // Search through the time deque for a match
    long int min_error = std::abs(time_cap_ms-t_ms_raw.back());
    int i_match = t_ms_raw.size()-1;
    for(int i=t_ms_raw.size()-2; i>=0; i--) {
        long int curr_error = std::abs(time_cap_ms-t_ms_raw.at(i));
        if(curr_error < min_error) {
            i_match = i;
            min_error = curr_error;
        }
        else {
            break;
        }
    }
    if(min_error > LIMIT_CAM_TIME_DIFF_MS) {
        std::cerr << time_cap_ms << " [error] Lateral estimator: LIMIT_CAM_TIME_DIFF_MS exceeded. Min error: " 
                << min_error << "ms" << std::endl;
        return;
    }

    // Convert attitude to Euler angles
    double psi_raw = yaw_d.at(i_match)/180.0 * M_PI;
    double theta = pitch_d.at(i_match)/1800.0 * M_PI;
    double phi = roll_d.at(i_match)/1800.0 * M_PI;

    // Apply psi correction
    double psi_corr = psi_raw + psi_off;

    Eigen::Matrix3d Ceb; // Body to Earth frame
    DCM_Cbe(phi, theta, psi_corr, Ceb);
    
    // Estimate drone position at the matched time
    double x_est_match = x_raw.at(i_match) + x_off + vx_off*(t_ms_raw.at(i_match)-t_ms_off)/1000.0;
    double y_est_match = y_raw.at(i_match) + y_off + vy_off*(t_ms_raw.at(i_match)-t_ms_off)/1000.0;

    // Estimate gate location and orientation in the Earth frame
    Eigen::Vector3d r_origin2body_e_est;
    r_origin2body_e_est << x_est_match, y_est_match, z_raw.at(i_match);
    Eigen::Vector3d r_body2gate_e;
    r_body2gate_e = Ceb*Cbc*r_cam2gate_c; // Observed vector to gate from the current IMU state
    // Eigen::Vector3d r_origin2gate_e;
    // r_origin2gate_e = r_origin2body_e + r_body2gate_e;
    Eigen::Vector3d orient_gate_e_obs; // Observed gate orientation from the current IMU state
    orient_gate_e_obs = Ceb*Cbc*orient_gate_c;

    // Figure out which gate this is
    double min_pos_error = 0;
    double assoc_psi_error = 0; // Yaw error associated with the minimum position error
    int i_gate_match = 0;
    bool gate_assigned = false;
    for(unsigned int i=0; i<gates_x_e.size(); i++) {

        // Given the orientation of the gate, calculate the yaw rotation necessary from this observation
        Eigen::Vector3d orient_gate_e_true(gates_orient_x.at(i), gates_orient_y.at(i), gates_orient_z.at(i));

        // yaw error = true gate yaw - observed gate yaw
        double psi_error = 0; // rads
        calc_vec_z_rotation(orient_gate_e_obs, orient_gate_e_true, psi_error);
        Eigen::Matrix3d dcm_z;
        DCM_Cbe(0, 0, -psi_error, dcm_z);

        // Calculate the platform's position working back from the gate position
        Eigen::Vector3d r_gate_e_true(gates_x_e.at(i), gates_y_e.at(i), gates_z_e.at(i));
        Eigen::Vector3d r_origin2body_e_back;
        r_origin2body_e_back = r_gate_e_true - dcm_z *r_body2gate_e;

        // Calculate error between the platform's expected position and the back-calculated position
        Eigen::Vector3d r_error;
        r_error = r_origin2body_e_back - r_origin2body_e_est;
        double r_error_norm = r_error.norm();

        if((i==0 || r_error_norm < min_pos_error) && r_error_norm < POS_ERROR_MAX_LIMIT && std::abs(psi_error) < PSI_ERROR_MAX_LIMIT) {
            min_pos_error = r_error_norm;
            assoc_psi_error = psi_error;
            i_gate_match = i;
            gate_assigned = true;
        }

    }

    // If no gate was assigned, then record this error
    if(!gate_assigned) {
        std::cerr << time_cap_ms << " [error] Lateral estimator: No gate assigned" << std::endl;
        return;
    }

    // Re-calculate the platform's position working back from the position of the matched gate
    Eigen::Vector3d r_origin2gatematch_e(gates_x_e.at(i_gate_match), gates_y_e.at(i_gate_match), gates_z_e.at(i_gate_match));
    Eigen::Vector3d r_origin2body_e;
    Eigen::Matrix3d dcm_z;
    DCM_Cbe(0, 0, -assoc_psi_error, dcm_z);
    r_origin2body_e = r_origin2gatematch_e - dcm_z *r_body2gate_e;

    // // Figure out which gate this is
    // double min_pos_error = 0;
    // double assoc_orient_error = 0; // orient error norm associated with the gate for min pos error
    // int i_gate_match = 0;
    // for(unsigned int i=0; i<gates_x_e.size(); i++) {

    //     // Get position and orientation errors of the observed vs known gate
    //     // locations and orientations
    //     Eigen::Vector3d r_error;
    //     r_error << gates_x_e.at(i) - r_origin2gate_e(0),
    //             gates_y_e.at(i) - r_origin2gate_e(1),
    //             gates_z_e.at(i) - r_origin2gate_e(2);
    //     Eigen::Vector3d orient_error;
    //     orient_error << gates_orient_x.at(i) - orient_gate_e(0),
    //                     gates_orient_y.at(i) - orient_gate_e(1),
    //                     gates_orient_z.at(i) - orient_gate_e(2);
        
    //     double r_error_norm = r_error.norm();
    //     double orient_error_norm = orient_error.norm();
    //     if(i==0 || r_error_norm < min_pos_error) {
    //         min_pos_error = r_error_norm;
    //         assoc_orient_error = orient_error_norm;
    //         i_gate_match = i;
    //     }
    // }

    // If the position error is greater than the maximum allowable, disregard this observation
    // if(min_pos_error > POS_ERROR_LIMIT_MAX) {
    //     std::cerr << time_cap_ms << " [error] Lateral estimator: POS_ERROR_LIMIT_MAX exceeded. Min error: " 
    //             << min_pos_error << "m" << std::endl;
    //     return;
    // }

    // Back-calculate the body position based upon the gate observation and known position
    // Eigen::Vector3d r_origin2gatematchtruth_e;
    // r_origin2gatematchtruth_e << gates_x_e.at(i_gate_match), gates_y_e.at(i_gate_match), gates_z_e.at(i_gate_match);

    // Observed body position in the Earth frame
    // Using gate truth position and IMU orientation
    // Eigen::Vector3d r_origin2bodyIMU_e; 
    // r_origin2bodyIMU_e = r_origin2gatematchtruth_e - r_body2gate_e;

    // Remove elements in the queue outside of the window
    for(unsigned int i=0; i<queue_t_ms.size(); i++) {
        if(time_cap_ms - queue_t_ms.at(i) > SLIDING_WINDOW_MS) {
            queue_t_ms.erase(queue_t_ms.begin());
            queue_x_meas.erase(queue_x_meas.begin());
            queue_x_raw.erase(queue_x_raw.begin());
            queue_y_meas.erase(queue_y_meas.begin());
            queue_y_raw.erase(queue_y_raw.begin());
            queue_psi_error.erase(queue_psi_error.begin());
        }
        else {
            break;
        }
    }

    // Add to the queue
    queue_t_ms.push_back(time_cap_ms); // Using the capture time rather than IMU time
    queue_x_meas.push_back(r_origin2body_e(0));
    queue_y_meas.push_back(r_origin2body_e(1));
    queue_x_raw.push_back(x_raw.at(i_match));
    queue_y_raw.push_back(y_raw.at(i_match));

    // as the assoc_yaw_error will include the previous yaw correction, add the previous correction as well
    queue_psi_error.push_back(assoc_psi_error + psi_off);

    // Copy previous offsets for file save later
    double x_off_prev = x_off;
    double y_off_prev = y_off;
    double vx_off_prev = vx_off;
    double vy_off_prev = vy_off;
    double psi_off_prev = psi_off;
    double t_ms_off_prev = t_ms_off;

    bool valid = false; 
    // Check sufficient number of elements in the queue
    if(queue_t_ms.size() >= MIN_SAMPLES_IN_WINDOW) {

        // Process the queue
        double x_off_temp = 0; double vx_off_temp = 0; double y_off_temp = 0; double vy_off_temp = 0;
        double psi_off_temp = 0; double psi_dot_off_temp = 0;
        long int t_ms_off_temp = 0;
        calc_offsets(x_off_temp, vx_off_temp, y_off_temp, vy_off_temp, psi_off_temp, psi_dot_off_temp, t_ms_off_temp, valid);
        if(valid) {
            x_off = x_off_temp;
            vx_off = vx_off_temp;
            y_off = y_off_temp;
            vy_off = vy_off_temp;
            psi_off = psi_off_temp;
            // Note: ignoring yaw rate offset
            t_ms_off = t_ms_off_temp;
        }

    }

    /* Quantities saved to file
    time_cap_ms : time (pc) at which frame was captured (ms)
    gate_obs_x: x of gate in camera frame (m)
    gate_obs_y: y of gate in camera frame (m)
    gate_obs_z: z of gate in camera frame (m)
    gate_orient_x: x component of gate orientation vector in camera frame (unit)
    gate_orient_y: y component of gate orientation vector in camera frame (unit)
    gate_orient_z: z component of gate orientation vector in camera frame (unit)
    t_match_ms: IMU capture time (PC) which is closest to the camera capture time (ms)
    roll_dec_m: IMU roll for matched time (*1/10deg)
    pitch_dec_m: IMU pitch for matched time (*1/10deg)
    yaw_m: IMU yaw for matched time (deg)
    x_raw_m: raw (uncorrected) x for the matched time (m)
    y_raw_m: raw (uncorrected) y for the matched time (m)
    z_raw_m: z for the matched time (m)
    vx_raw_m: raw (uncorrected) velocity x for the matched time (m/s)
    vy_raw_m: raw (uncorrected) velocity y for the matched time (m/s)
    t_ms_off_p: previous - time of the offset (ms)
    x_off_p: previous x offset (m)
    y_off_p: previous y offset (m)
    vx_off_p: previous vx offset (m/s)
    vy_off_p: previous vy offset (m/s)
    t_ms_off: time of the offset (ms)
    x_off: x offset (m)
    y_off: y offset (m) 
    vx_off: vx offset (m/s)
    vy_off: vy offset (m/s)
    n_queue: number of matched frames in the queue
    valid: whether a correction was calculated for this frame/current queue is valid (1 true / 0 false)
    */


    // File writes
    if(files_open) {
        // header - long, not listed here
        //
        file_lateral_cam << time_cap_ms << "," << r_cam2gate_c(0) << "," << r_cam2gate_c(1) << "," << orient_gate_c(2) << "," << orient_gate_c(0) << "," << orient_gate_c(1) << "," << orient_gate_c(2) << ","
                        << t_ms_raw.at(i_match) << "," << roll_d.at(i_match) << "," << pitch_d.at(i_match) << "," << yaw_d.at(i_match) << "," << x_raw.at(i_match) << "," << y_raw.at(i_match) << "," << z_raw.at(i_match) << ","
                        << vx_raw.at(i_match) << "," << vy_raw.at(i_match) << ","
                        << t_ms_off_prev << "," << x_off_prev << "," << y_off_prev << "," << vx_off_prev << "," << vy_off_prev << ","
                        << t_ms_off << "," << x_off << "," << y_off << "," << vx_off << "," << vy_off << "," << queue_t_ms.size() << "," << (int)valid << std::endl;
    }

}

void LateralEstimator::calc_vec_z_rotation(const Eigen::Vector3d &a, /* Observed gate orientation vector in Earth frame */
                                        const Eigen::Vector3d &b, /* True gate orientation vector in Earth frame */
                                        double &psi /* True gate yaw - observed gate yaw, rad */) 
{
    
    // Angle to rotate a towards b about the z-axis

    // Project the vectors onto the XY-plane
    // In this case, can just take the x and y components
    Eigen::Vector3d a_proj(a(0), a(1), 0);
    Eigen::Vector3d b_proj(b(0), b(1), 0);
    
    // Calculate the yaw error between the true and observed gate orientations
    Eigen::Vector3d crossp_n = a_proj.cross(b_proj);
    double dotp = a_proj.dot(b_proj);

    // opposite and adjacent. Note: no need to divide by vector magnitudes as it will divide out
    // If the normal vector is in the opposite direction to the positive z-axis, need to negate the angle
    double opp = crossp_n(2) > 0 ? crossp_n.norm() : -crossp_n.norm();
    double adj = dotp;
    psi = std::atan2(opp, adj); // between -pi and pi

}


void LateralEstimator::calc_offsets(double &_x_off, double &_vx_off, double &_y_off, double &_vy_off, double &_psi_off, double &_psi_dot_off, long int &_t_ms_off, bool &valid)
{

    // SAFE TO RUN IN PARALLEL (BUT NOT IN PARALLEL TO ADD_GATE)

    /*
    Class variables which are read:
    has_measurement
    queue
    */

    /*
    Class variables which are written to:
    */

    // Do not attempt to calculate correction if there aren't sufficient elements in the queue.
    if(queue_t_ms.size() < 3 || !has_measurement) {
        valid = false;
        return;
    }

    Eigen::VectorXd Dt(queue_t_ms.size());
    Eigen::VectorXd Dx(queue_t_ms.size());
    Eigen::VectorXd Dy(queue_t_ms.size());
    Eigen::VectorXd Dpsi(queue_t_ms.size());

    for(unsigned int i=0; i<queue_t_ms.size(); i++) {
        Dt(i) = (queue_t_ms.at(i) - queue_t_ms.back())/1000.0;
        Dx(i) = queue_x_meas.at(i) - queue_x_raw.at(i);
        Dy(i) = queue_y_meas.at(i) - queue_y_raw.at(i);
        Dpsi(i) = queue_psi_error.at(i);
    }

    unsigned iter = 20;
    double sigma_thresh = 1;
    double sigma_thresh_yaw = 0.0873; // Approx 5 deg
    double sample_prop = 0.8;
    unsigned int select_n = static_cast<unsigned int>(round(queue_t_ms.size()*sample_prop));
    Eigen::Matrix2d P; P << 0, 0, 0, 0.3;
    bool valid_x = false;
    bool valid_y = false;
    bool valid_yaw = false;
    try {
        prior_ransac(Dt, Dx, iter, sigma_thresh, select_n, P, _x_off, _vx_off, valid_x, 0);
        prior_ransac(Dt, Dy, iter, sigma_thresh, select_n, P, _y_off, _vy_off, valid_y, 0);
        prior_ransac(Dt, Dpsi, iter, sigma_thresh, select_n, P, _psi_off, _psi_dot_off, valid_yaw, 0);
    } catch(const std::exception& e) {
        std::cerr << "Ransac thrown exception" << e.what() << std::endl;
        valid_x = false;
        valid_y = false;
        valid_yaw = false;
    }
    valid = valid_x && valid_y && valid_yaw;
    _t_ms_off = queue_t_ms.back();

}


void LateralEstimator::DCM_Cbe(const double phi, const double theta, const double psi, Eigen::Matrix<double,3,3> &DCM)
{
    /* 
    Calculate the DCM of earth to body frame. 
    Rotations applied in the order psi -> theta -> phi.
    */
    DCM(0,0) = std::cos(psi)*std::cos(theta); DCM(0,1) = std::sin(psi)*std::cos(theta); DCM(0,2) = -std::sin(theta);
    DCM(1,0) = std::cos(psi)*std::sin(theta)*std::sin(phi)-std::sin(psi)*std::cos(phi); DCM(1,1) = std::sin(psi)*std::sin(theta)*std::sin(phi)+std::cos(psi)*std::cos(phi); DCM(1,2) = std::cos(theta)*std::sin(phi);
    DCM(2,0) = std::cos(psi)*std::sin(theta)*std::cos(phi)+std::sin(psi)*std::sin(phi); DCM(2,1) = std::sin(psi)*std::sin(theta)*std::cos(phi)-std::cos(psi)*std::sin(phi); DCM(2,2) = std::cos(theta)*std::cos(phi);
}

void LateralEstimator::get_position(long int time_ms, double &x, double &vx, double &y, double &vy, bool &valid, bool &warn_time, const long int prop_time_limit_ms)
{
    // Apply the correction to give a position estimate
    double DeltaT_off = (time_ms - t_ms_off)/1000.0; // Time difference to when the offset was calculated in SECONDS
    double DeltaT_raw = (time_ms - t_ms_raw.back())/1000.0; // Time difference to the last IMU measurement in SECONDS

    // Warn if the propagation time limit was exceeded
    warn_time = false;
    if((time_ms - t_ms_raw.back()) > prop_time_limit_ms) {
        std::cerr << "[warn] LateralEstimator, propagation time exceeds limit. DeltaT_raw: " << DeltaT_raw << "s" << std::endl;
        warn_time = true;
        // Set the propagation times to their limits and continue
        double diff = DeltaT_raw - prop_time_limit_ms/1000.0;
        DeltaT_raw = prop_time_limit_ms/1000.0;
        DeltaT_off -= diff;
    }

    // Propagate the raw measurements
    // However treat it as constant velocity
    double x_raw_prop = x_raw.back() + vx_raw.back()*DeltaT_raw;
    double vx_raw_prop = vx_raw.back();
    double y_raw_prop = y_raw.back() + vy_raw.back()*DeltaT_raw;
    double vy_raw_prop = vy_raw.back();

    // Apply the offsets
    x = x_raw_prop + x_off + vx_off*DeltaT_off;
    vx = vx_raw_prop + vx_off;

    y = y_raw_prop + y_off + vy_off*DeltaT_off;
    vy = vy_raw_prop + vy_off;

    // Check validity
    valid = true;
    if(DeltaT_off < 0 || DeltaT_raw < 0) {
        std::cerr << "[error] LateralEstimator, negative time delta. DeltaT_off: " << DeltaT_off << "s, DeltaT_raw: " << DeltaT_raw << "s" << std::endl;
        valid = false;
        warn_time = true;
    }

    if(!has_measurement) {
        valid = false; // If there has not been an imu measurement
    }
    
}

void LateralEstimator::reset()
{
    // Reset everything back to the condition immediately after construction
    has_measurement = false;

    x_raw.clear(); x_raw.assign(500,0);
    vx_raw.clear(); vx_raw.assign(500,0);
    y_raw.clear(); y_raw.assign(500,0);
    vy_raw.clear(); vy_raw.assign(500,0);
    z_raw.clear(); z_raw.assign(500,0);
    roll_d.clear(); roll_d.assign(500,0);
    pitch_d.clear(); pitch_d.assign(500,0);
    yaw_d.clear(); yaw_d.assign(500,0);
    t_ms_raw.clear(); t_ms_raw.assign(500,0);

    x_off = 0;
    vx_off = 0;
    y_off = 0;
    vy_off = 0;
    t_ms_off = 0;
    psi_off = 0;

    queue_t_ms.clear();
    queue_x_meas.clear();
    queue_x_raw.clear();
    queue_y_meas.clear();
    queue_y_raw.clear();
    queue_psi_error.clear();
}

void LateralEstimator::get_heading(int &_yaw_d, bool &valid)
{
    // Note: yaw in (whole) degrees

    if(!has_measurement) {
        valid = false;
        return;
    }
    _yaw_d = yaw_d.back(); 
}


// **********************************************************************
// File Methods
// **********************************************************************

bool LateralEstimator::open_files() {
    if(!file_lateral_raw_imu.is_open()) {
        std::string name_lateral_raw_imu = file_directory + "/" + prefix_lateral_raw_imu + suffix + format;
        file_lateral_raw_imu.open(name_lateral_raw_imu, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }

    if(!file_lateral_cam.is_open()) {
        std::string name_lateral_cam = file_directory + "/" + prefix_lateral_cam + suffix + format;
        file_lateral_cam.open(name_lateral_cam, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    }
    
    files_open = true;
    if(file_lateral_raw_imu.is_open()) {
        file_lateral_raw_imu << std::endl << header_lateral_raw_imu << std::endl;
    } else {
        files_open = false;
    }
    if(file_lateral_cam.is_open()) {
        file_lateral_cam << std::endl << header_lateral_cam << std::endl;
    } else {
        files_open = false;
    }

    return files_open;
}

void LateralEstimator::close_files() {
    file_lateral_raw_imu.close();
    file_lateral_cam.close();
    files_open = false;
}

void LateralEstimator::set_file_suffix(std::string suffix_in) {
    suffix = suffix_in;
}

std::string LateralEstimator::get_file_suffix() {
    return suffix;
}

void LateralEstimator::set_file_directory(std::string directory_in) {
    file_directory = directory_in;
}

std::string LateralEstimator::get_file_directory()
{
    return file_directory;
}