#include "ArduinoEigenDense.h"

using Vec3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix<float,3,3>;
using Quat = Eigen::Quaternionf;
using Mat18 = Eigen::Matrix<float,18,18>;
using StartupMeasurement = Eigen::Matrix<float,3+3+3+3,1>;
using ZVUPMeasurement = Eigen::Matrix<float,3,1>;
using ProcessNoiseSized = Eigen::Matrix<float,12,1>;



struct Boggle {

};

/**
 * Some shorthand used in this file:
 *
 * CT: continous time
 * DT: Discrete Time
 * _b is usually bias
 */
namespace eskf {
    class ESKF {


    private:
        //CT noise, filled with signa_an, sigma_wn, sigma_aw, sigma_ww
        ProcessNoiseSized sigmas = ProcessNoiseSized::Zero();




    public: //Were this not cpp, i would not make everything public.
        //Giant blob of preallocated memory so we don't spam stack alloc/dealloc

        //Nominal State
        Vec3 p = Vec3::Zero(); //position in the world frame
        Vec3 v = Vec3::Zero(); //velocity in the world frame
        Quat q = Quat::Identity(); //body to world quaternion
        Vec3 a_b = Vec3::Zero(); //acceleration bias in the body frame
        Vec3 w_b = Vec3::Zero(); //heading bias in the body frame
        Vec3 g = Vec3(0,0,-9.80665f); //gravity bias in the world frame


        //Integrator Matrices
        Vec3 a_tilde = Vec3::Zero();
        Mat3 R_b2w; //Overwritten with quaternion
        Mat3 S; //Overwritten with acceleration skew
        Vec3 a_w = Vec3::Zero();


        //Predict Matrices
        Mat18 Fx = Mat18::Identity(); //DT linearized process matrix
        Mat18 Q = Mat18::Zero(); //DT noise prefab matrix, formed from Fi Qi Fi' for efficiency
        Mat3 Omega; //Overwritten with orientation skew

        //Error Covariance
        Mat18 P = Mat18::Identity()*1e-6f;

        ESKF(const float sigma_an, const float sigma_wn, const float sigma_aw, const float sigma_ww) {
            sigmas.segment<3>(0).setConstant(sigma_an * sigma_an);
            sigmas.segment<3>(3).setConstant(sigma_wn * sigma_wn);
            sigmas.segment<3>(6).setConstant(sigma_aw * sigma_aw);
            sigmas.segment<3>(9).setConstant(sigma_ww * sigma_ww);
        };


        /**
         * Runs the integration loop at 1khz
         */
        void integrate(const Vec3& a_m, const Vec3& w_m, float dt);

        /**
         * Every 100hz, call predict after integrate to do a kalman prediction
         */
        void predict(const Vec3& a_m, const Vec3& w_m, float dt);

        /**
         * During the startup period, call this after predict to learn the biases from the filter
         */
        void correct_startup(const StartupMeasurement& y);

        /**
         * During the running period, call this after predict to fix position drift
         */
        void correct_zvup(const ZVUPMeasurement& y);


        // forward / lateral velocity in heading frame
        void getHorizontalVel(float& v_fw, float& v_lat) const;
    };
}
