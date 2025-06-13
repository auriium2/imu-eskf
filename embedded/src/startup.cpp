#include "ArduinoEigen.h"
#include "startup.h"
#include <vector>
#include <cmath>

namespace eskf {

// Constructor initializes all state variables
IMUCalibrator::IMUCalibrator(float gravity) 
    : g(gravity), num_samples(0) {
    reset();
}

// Reset all calibration state
void IMUCalibrator::reset() {
    num_samples = 0;
    
    // Reset accelerometer state
    acc_mean.setZero();
    acc_bias.setZero();
    M2_acc.setZero();
    
    // Reset gyroscope state
    gyr_mean.setZero();
    gyr_bias.setZero();
    M2_gyr.setZero();
    
    // Reset attitude state
    quat_sum.setZero();
    att_bias.setZero();
    M2_att.setZero();
    
    // Reset velocity state
    vel_mean.setZero();
    M2_vel.setZero();
    
    // Reset position state
    cumulative_vel.setZero();
    cumulative_pos.setZero();
    pos_sum.setZero();
    pos_sq_sum.setZero();
    
    // Reset gravity state
    g_bias.setZero();
    M2_g.setZero();
    
    // Reset quaternion mean
    q_mean = Eigen::Quaternionf::Identity();
    q_mean_conj = Eigen::Quaternionf::Identity();
    
    // Reset covariance matrices
    cov_acc.setZero();
    cov_gyr.setZero();
    cov_att.setZero();
    cov_vel.setZero();
    cov_pos.setZero();
    cov_g.setZero();
    
    // Reset variance vectors
    var_vel.setZero();
    var_pos.setZero();
    
    // Finalized state flag
    is_finalized = false;
}

// Add a single IMU sample for calibration
void IMUCalibrator::addSample(
    const Eigen::Vector3f& acc, 
    const Eigen::Vector3f& gyr, 
    const Eigen::Quaternionf& quat, 
    const Eigen::Matrix3f& rot, 
    const Eigen::Vector3f& dV, 
    float dT) 
{
    // Increment sample counter
    num_samples++;
    
    // Phase 1: Update means
    // Update accelerometer mean
    acc_mean += (acc - acc_mean) / num_samples;
    
    // Update gyroscope mean
    gyr_mean += (gyr - gyr_mean) / num_samples;
    
    // Accumulate quaternion components for later averaging
    quat_sum[0] += quat.x();
    quat_sum[1] += quat.y();
    quat_sum[2] += quat.z();
    quat_sum[3] += quat.w();
    
    // Update quaternion mean after each sample
    q_mean = Eigen::Quaternionf(
        quat_sum[3] / num_samples,
        quat_sum[0] / num_samples, 
        quat_sum[1] / num_samples, 
        quat_sum[2] / num_samples
    );
    q_mean.normalize();
    q_mean_conj = q_mean.conjugate();
    
    // Phase 2: Update covariances
    
    // Accelerometer bias calculation
    Eigen::Vector3f g_body = rot.transpose() * Eigen::Vector3f(0, 0, g);
    Eigen::Vector3f ba_sample = acc - g_body;
    
    // Update accelerometer bias covariance using Welford's online algorithm
    Eigen::Vector3f delta_acc = ba_sample - acc_bias;
    M2_acc += delta_acc * delta_acc.transpose();
    
    // Update gyroscope bias covariance using Welford's online algorithm
    Eigen::Vector3f delta_gyr = gyr - gyr_bias;
    M2_gyr += delta_gyr * delta_gyr.transpose();
    
    // Attitude error calculation
    Eigen::Quaternionf dq = quat * q_mean_conj;
    dq.normalize();
    Eigen::Vector3f att_err = 2 * dq.vec();
    
    // Update attitude bias mean using rolling average
    att_bias += (att_err - att_bias) / num_samples;
    
    // Update attitude bias covariance using Welford's online algorithm
    Eigen::Vector3f delta_att = att_err - att_bias;
    M2_att += delta_att * delta_att.transpose();
    
    // Velocity calculation
    Eigen::Vector3f dv_free_b = dV - g_body * dT;
    Eigen::Vector3f dv_free_w = rot * dv_free_b;
    
    // Update velocity mean using rolling average
    vel_mean += (dv_free_w - vel_mean) / num_samples;
    
    // Update velocity covariance using Welford's online algorithm
    Eigen::Vector3f delta_vel = dv_free_w - vel_mean;
    M2_vel += delta_vel * delta_vel.transpose();
    
    // Position calculations
    // Update cumulative velocity and position
    cumulative_vel += dv_free_w;
    cumulative_pos += cumulative_vel * dT;
    
    // Update position statistics
    pos_sum += cumulative_pos;
    pos_sq_sum += cumulative_pos.cwiseProduct(cumulative_pos);
    
    // Gravity calculation
    Eigen::Vector3f a_corr = acc - acc_bias;
    Eigen::Vector3f g_world_i = rot * a_corr;
    
    // Update gravity covariance using Welford's online algorithm
    Eigen::Vector3f delta_g = g_world_i - g_bias;
    M2_g += delta_g * delta_g.transpose();
    
    // Set biases based on current means
    gyr_bias = gyr_mean;  // Gyro bias is just the mean
    
    // Mark as not finalized after adding a new sample
    is_finalized = false;
}

// Finalize the calibration - compute final covariances
void IMUCalibrator::finalize() {
    if (num_samples <= 1) {
        // Not enough samples for meaningful statistics
        return;
    }
    
    // Compute final covariance matrices
    cov_acc = M2_acc / (num_samples - 1);
    cov_gyr = M2_gyr / (num_samples - 1);
    cov_att = M2_att / (num_samples - 1);
    
    // Calculate velocity variance/covariance
    for (int j = 0; j < 3; j++) {
        var_vel[j] = M2_vel(j, j) / (num_samples - 1);
    }
    cov_vel = var_vel.asDiagonal();
    
    // Calculate position variance/covariance
    Eigen::Vector3f pos_mean = pos_sum / num_samples;
    var_pos = (pos_sq_sum - pos_sum.cwiseProduct(pos_mean) * num_samples) / (num_samples - 1);
    cov_pos = var_pos.asDiagonal();
    
    // Calculate gravity covariance
    cov_g = M2_g / (num_samples - 1);
    
    // Mark calibration as finalized
    is_finalized = true;
}

// Get current accelerometer bias
Eigen::Vector3f IMUCalibrator::getAccBias() {
    if (!is_finalized) finalize();
    return acc_bias;
}

// Get current accelerometer bias covariance
Eigen::Matrix3f IMUCalibrator::getAccCovariance() {
    if (!is_finalized) finalize();
    return cov_acc;
}

// Get current gyroscope bias
Eigen::Vector3f IMUCalibrator::getGyrBias() {
    if (!is_finalized) finalize();
    return gyr_bias;
}

// Get current gyroscope bias covariance
Eigen::Matrix3f IMUCalibrator::getGyrCovariance() {
    if (!is_finalized) finalize();
    return cov_gyr;
}

// Get current attitude bias
Eigen::Vector3f IMUCalibrator::getAttBias() {
    if (!is_finalized) finalize();
    return att_bias;
}

// Get current attitude bias covariance
Eigen::Matrix3f IMUCalibrator::getAttCovariance() {
    if (!is_finalized) finalize();
    return cov_att;
}

// Get current velocity bias
Eigen::Vector3f IMUCalibrator::getVelBias() {
    if (!is_finalized) finalize();
    return vel_mean;
}

// Get current velocity bias covariance
Eigen::Matrix3f IMUCalibrator::getVelCovariance() {
    if (!is_finalized) finalize();
    return cov_vel;
}

// Get current position covariance
Eigen::Matrix3f IMUCalibrator::getPosCovariance() {
    if (!is_finalized) finalize();
    return cov_pos;
}

// Get current gravity bias
Eigen::Vector3f IMUCalibrator::getGravityBias() {
    if (!is_finalized) finalize();
    return g_bias;
}

// Get current gravity bias covariance
Eigen::Matrix3f IMUCalibrator::getGravityCovariance() {
    if (!is_finalized) finalize();
    return cov_g;
}

// Get the number of samples processed
size_t IMUCalibrator::getSampleCount() {
    return num_samples;
}

// Legacy static function maintained for compatibility
void performInitialCalibration(
    const std::vector<Eigen::Vector3f>& acc_stand,
    const std::vector<Eigen::Vector3f>& gyr_stand,
    const std::vector<Eigen::Quaternionf>& quat_stand,
    const std::vector<Eigen::Matrix3f>& rot_stand,
    const std::vector<Eigen::Vector3f>& dV_stand,
    const std::vector<float>& dT,
    float g)
{
    // Create calibrator instance
    IMUCalibrator calibrator(g);
    
    // Process all samples
    size_t numSamples = acc_stand.size();
    for (size_t i = 0; i < numSamples; i++) {
        calibrator.addSample(
            acc_stand[i],
            gyr_stand[i],
            quat_stand[i],
            rot_stand[i],
            dV_stand[i],
            dT[i]
        );
    }
    
    // Finalize calibration
    calibrator.finalize();
    
    // Values can be accessed via getter methods
    // but are not returned from this function
}

} // namespace eskf