#pragma once

#include "ArduinoEigen.h"
#include <vector>

namespace eskf {

/**
 * IMUCalibrator - A class for performing live IMU calibration
 * 
 * This class provides online calibration of IMU sensors using incremental
 * processing of samples. It can be used for initial calibration at startup
 * or for continuous re-calibration during operation.
 */
class IMUCalibrator {
public:
    /**
     * Constructor 
     * 
     * @param gravity The gravity constant (e.g., 9.80665 m/s^2)
     */
    IMUCalibrator(float gravity);
    
    /**
     * Reset the calibrator state
     */
    void reset();
    
    /**
     * Add a single IMU sample for calibration
     * 
     * @param acc Accelerometer measurement (body frame)
     * @param gyr Gyroscope measurement (body frame)
     * @param quat Orientation quaternion (body to world)
     * @param rot Rotation matrix (body to world)
     * @param dV Delta velocity measurement
     * @param dT Time interval for this sample
     */
    void addSample(
        const Eigen::Vector3f& acc, 
        const Eigen::Vector3f& gyr, 
        const Eigen::Quaternionf& quat, 
        const Eigen::Matrix3f& rot, 
        const Eigen::Vector3f& dV, 
        float dT);
    
    /**
     * Finalize the calibration calculations
     * Computes final covariance matrices and sets the finalized flag
     */
    void finalize();
    
    /**
     * Get current accelerometer bias
     * @return Accelerometer bias vector
     */
    Eigen::Vector3f getAccBias();
    
    /**
     * Get current accelerometer bias covariance
     * @return Accelerometer bias covariance matrix
     */
    Eigen::Matrix3f getAccCovariance();
    
    /**
     * Get current gyroscope bias
     * @return Gyroscope bias vector
     */
    Eigen::Vector3f getGyrBias();
    
    /**
     * Get current gyroscope bias covariance
     * @return Gyroscope bias covariance matrix
     */
    Eigen::Matrix3f getGyrCovariance();
    
    /**
     * Get current attitude bias
     * @return Attitude bias vector
     */
    Eigen::Vector3f getAttBias();
    
    /**
     * Get current attitude bias covariance
     * @return Attitude bias covariance matrix
     */
    Eigen::Matrix3f getAttCovariance();
    
    /**
     * Get current velocity bias
     * @return Velocity bias vector
     */
    Eigen::Vector3f getVelBias();
    
    /**
     * Get current velocity bias covariance
     * @return Velocity bias covariance matrix
     */
    Eigen::Matrix3f getVelCovariance();
    
    /**
     * Get current position covariance
     * @return Position covariance matrix
     */
    Eigen::Matrix3f getPosCovariance();
    
    /**
     * Get current gravity bias
     * @return Gravity bias vector
     */
    Eigen::Vector3f getGravityBias();
    
    /**
     * Get current gravity bias covariance
     * @return Gravity bias covariance matrix
     */
    Eigen::Matrix3f getGravityCovariance();
    
    /**
     * Get the number of samples processed
     * @return Number of samples
     */
    size_t getSampleCount();
    
private:
    // Constants
    float g;  // Gravity constant
    
    // Sample counter
    size_t num_samples;
    
    // Flag to track if results are finalized
    bool is_finalized;
    
    // Accelerometer state
    Eigen::Vector3f acc_mean;
    Eigen::Vector3f acc_bias;
    Eigen::Matrix3f M2_acc;
    Eigen::Matrix3f cov_acc;
    
    // Gyroscope state
    Eigen::Vector3f gyr_mean;
    Eigen::Vector3f gyr_bias;
    Eigen::Matrix3f M2_gyr;
    Eigen::Matrix3f cov_gyr;
    
    // Attitude state
    Eigen::Vector4f quat_sum;
    Eigen::Quaternionf q_mean;
    Eigen::Quaternionf q_mean_conj;
    Eigen::Vector3f att_bias;
    Eigen::Matrix3f M2_att;
    Eigen::Matrix3f cov_att;
    
    // Velocity state
    Eigen::Vector3f vel_mean;
    Eigen::Vector3f var_vel;
    Eigen::Matrix3f M2_vel;
    Eigen::Matrix3f cov_vel;
    
    // Position state
    Eigen::Vector3f cumulative_vel;
    Eigen::Vector3f cumulative_pos;
    Eigen::Vector3f pos_sum;
    Eigen::Vector3f pos_sq_sum;
    Eigen::Vector3f var_pos;
    Eigen::Matrix3f cov_pos;
    
    // Gravity state
    Eigen::Vector3f g_bias;
    Eigen::Matrix3f M2_g;
    Eigen::Matrix3f cov_g;
};

/**
 * Legacy function for batch processing calibration
 * 
 * @param acc_stand Accelerometer measurements during standing
 * @param gyr_stand Gyroscope measurements during standing
 * @param quat_stand Quaternion measurements during standing
 * @param rot_stand Rotation matrix measurements during standing
 * @param dV_stand Delta velocity measurements during standing
 * @param dT Time intervals
 * @param g Gravity constant
 */
void performInitialCalibration(
    const std::vector<Eigen::Vector3f>& acc_stand,
    const std::vector<Eigen::Vector3f>& gyr_stand,
    const std::vector<Eigen::Quaternionf>& quat_stand,
    const std::vector<Eigen::Matrix3f>& rot_stand,
    const std::vector<Eigen::Vector3f>& dV_stand,
    const std::vector<float>& dT,
    float g);

} // namespace eskf