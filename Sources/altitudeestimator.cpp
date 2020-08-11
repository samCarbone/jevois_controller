#include "altitudeestimator.h"

AltitudeEstimator::AltitudeEstimator()
{
    resetStateEstimate();
}

AltitudeEstimator::~AltitudeEstimator()
{
    closeFiles();
}

void AltitudeEstimator::resetStateEstimate()
{
    // TODO: move x_0 and P_0 into header file
    x.fill(0);
    P << 1, 1,
         1, 1;
    hasPreviousMeasurement = false;
}

void AltitudeEstimator::addRangeMeasurement(RangingData_t rangeData)
{
    if(!hasPreviousMeasurement) {
        /*
         * Here the state initialisation is overwritten and it is set directly to the
         * ranging sensor measurement.
         * Alternative: only set the times, and make the estimator continue
         * with the initialised states
        */
        x(0,0) = -rangeData.range_mm/1000.0;
        x(1,0) = 0;
        currentTimePc_ms = rangeData.timeEsp_ms;
        currentTimePc_ms = rangeData.timePc_ms;
        hasPreviousMeasurement = true;
        return;
    }

    // Time delta from previous measurement
    double Delta_t = (rangeData.timeEsp_ms - currentTimeEsp_ms)/1000.0;
    currentTimeEsp_ms = rangeData.timeEsp_ms;
    currentTimePc_ms = rangeData.timePc_ms;

    // State transition
    Eigen::Matrix<double, 2, 2> F;
    F << 1, Delta_t,
         0, 1;

    // Process covariance
    Eigen::Matrix<double, 2, 2> Q;
    Q << pow(Delta_t, 2)*pow(sigma_v, 2), Delta_t*pow(sigma_v, 2),
         Delta_t*pow(sigma_v, 2), pow(sigma_v, 2);

    // Measurement variance
    Eigen::Matrix<double, 1, 1> R;
    R << pow(rangeData.sigma_mm, 2);

    // Measurement
    Eigen::Matrix<double, 1, 1> z;
    z << rangeData.range_mm;

    // Measurement map matrix
    Eigen::Matrix<double, 1, 2> H;
    H << -1000, 0;

    // Propagate with constant velocity assumption
    x = F*x;
    try {
        P = F*P*F.inverse() + Q;
    } catch(const std::exception& e) {
        std::cout << e.what();
    }

    // Measurement update
    // Kalman gain
    Eigen::Matrix<double, 2, 1> K;
    try {
        K = P*H.transpose()*((H*P*H.transpose() + R).inverse());
    } catch(const std::exception& e) {
        std::cout << e.what();
    }

    // State update
    x = x + K*(z - H*x);

    // State covariance update
    P = (Eigen::MatrixXd::Identity(2,2) - K*H)*P*((Eigen::MatrixXd::Identity(2,2) - K*H).transpose()) + K*R*K.transpose();

    // File writes
    if(filesOpen) {
        // header
        // time_esp_ms,time_pc_ms,z,z_dot,P_11,P_12,P_21,P_22,Delta_t_ms,sigma_v,range_mm,sigma_mm,signal_rate,ambient_rate,eff_spad_count,status
        file_alt_est << currentTimeEsp_ms << "," << currentTimePc_ms << "," << x(0,0) << "," << x(1,0) << ","
                     << P(0,0) << "," << P(0,1) << "," << P(1,0) << "," << P(1,1) << "," << Delta_t << "," << sigma_v << ","
                     << rangeData.range_mm << "," << rangeData.sigma_mm << "," << rangeData.signal_rate << "," << rangeData.ambient_rate << ","
                     << rangeData.eff_spad_count << "," << rangeData.status << std::endl;
    }
}

AltState_t AltitudeEstimator::getStateEstimate()
{
    AltState_t stateResult;
    stateResult.z = x(0,0);
    stateResult.z_dot = x(1,0);
    stateResult.timeEsp_ms = currentTimeEsp_ms;
    stateResult.timePc_ms = currentTimePc_ms;
    stateResult.P = P;
    stateResult.valid = hasPreviousMeasurement;
    return stateResult;
}

AltState_t AltitudeEstimator::getPropagatedStateEstimate(int newTimePc_ms)
{
    // Time difference based on PC time
    double Delta_t = (newTimePc_ms - currentTimePc_ms)/1000.0;

    // State transition
    Eigen::Matrix<double, 2, 2> F;
    F << 1, Delta_t,
         0, 1;

    // Process covariance
    Eigen::Matrix<double, 2, 2> Q;
    Q << pow(Delta_t, 2)*pow(sigma_v, 2), Delta_t*pow(sigma_v, 2),
         Delta_t*pow(sigma_v, 2), pow(sigma_v, 2);

    // Propagate with constant velocity assumption
    Eigen::Matrix<double, 2, 1> x_prop;
    Eigen::Matrix<double, 2, 2> P_prop;
    x_prop = F*x;
    P_prop = F*P*F.inverse() + Q;

    AltState_t stateResult;
    stateResult.z = x_prop(0,0);
    stateResult.z_dot = x_prop(1,0);
    stateResult.timeEsp_ms = currentTimeEsp_ms + newTimePc_ms - currentTimePc_ms;
    stateResult.timePc_ms = newTimePc_ms;
    stateResult.P = P_prop;
    stateResult.valid = hasPreviousMeasurement;

    return stateResult;
}

AltState_t AltitudeEstimator::getPropagatedStateEstimate_safe(int newTimePc_ms, int limitDelta_ms)
{
    int Delta_t = newTimePc_ms - currentTimePc_ms;
    if(Delta_t > limitDelta_ms) {
        std::cout << "[warn] Long forward propagation time: " << Delta_t << "ms" << std::endl;
        newTimePc_ms = currentTimePc_ms + limitDelta_ms;
    }

    return getPropagatedStateEstimate(newTimePc_ms);
}

int AltitudeEstimator::getCurrentTimePc_ms()
{
    return currentTimePc_ms;
}

int AltitudeEstimator::getCurrentTimeEsp_ms()
{
    return currentTimeEsp_ms;
}



// **********************************************************************
// File Methods
// **********************************************************************

bool AltitudeEstimator::openFiles() {
    if (file_alt_est.is_open())
        return false;
    std::string name_alt_est = fileDirectory + "/" + prefix_alt_est + suffix + format;
    file_alt_est.open(name_alt_est, std::ios::out | std::ios::app); // Append the file contents to prevent overwrite
    if (file_alt_est.is_open()) {
        file_alt_est << std::endl << header_alt_est << std::endl;
        filesOpen = true;
        return true;
    }
    else {
        return false;
    }
}

void AltitudeEstimator::closeFiles() {
    file_alt_est.close();
    filesOpen = false;
}

void AltitudeEstimator::setFileSuffix(std::string suffix_in) {
    suffix = suffix_in;
}

std::string AltitudeEstimator::getFileSuffix() {
    return suffix;
}

void AltitudeEstimator::setFileDirectory(std::string directory) {
    fileDirectory = directory;
}

std::string AltitudeEstimator::getFileDirectory()
{
    return fileDirectory;
}
