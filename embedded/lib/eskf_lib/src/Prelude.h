#pragma once
#ifdef ARDUINO
    #include "ArduinoEigen.h"
    #include <cmath>
#else
    #include <Eigen/Dense>
    #include "math.h"
#endif

using Vec3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix<float,3,3>;
using Quat = Eigen::Quaternionf;
using Mat18 = Eigen::Matrix<float,18,18>;
using X0 = Eigen::Matrix<float,19,1>;
using StartupMeasurement = Eigen::Matrix<float,3+3+3+3,1>;
using ZVUPMeasurement = Eigen::Matrix<float,3,1>;
using ProcessNoiseSized = Eigen::Matrix<float,12,1>;

struct ProcessNoise {
    float sigma_an;  // accelerometer noise standard deviation
    float sigma_wn;  // gyroscope noise standard deviation
    float sigma_aw;  // accelerometer bias walk standard deviation
    float sigma_ww;  // gyroscope bias walk standard deviation
    
    // Default constructor
    ProcessNoise() :
        sigma_an(0.0f),
        sigma_wn(0.0f),
        sigma_aw(0.0f),
        sigma_ww(0.0f) {}
    
    // Constructor with parameters
    ProcessNoise(float an, float wn, float aw, float ww) :
        sigma_an(an),
        sigma_wn(wn),
        sigma_aw(aw),
        sigma_ww(ww) {}
        
    // Convert to ProcessNoiseSized for internal calculations
    ProcessNoiseSized toVector() const {
        ProcessNoiseSized result;
        result.segment<3>(0).setConstant(sigma_an * sigma_an);
        result.segment<3>(3).setConstant(sigma_wn * sigma_wn);
        result.segment<3>(6).setConstant(sigma_aw * sigma_aw);
        result.segment<3>(9).setConstant(sigma_ww * sigma_ww);
        return result;
    }
};

// Nominal state structure for the ESKF
struct NominalState {
    Vec3 p;     // position in the world frame
    Vec3 v;     // velocity in the world frame
    Quat q;     // body to world quaternion
    Vec3 a_b;   // acceleration bias in the body frame
    Vec3 w_b;   // heading bias in the body frame
    Vec3 g;     // learned gravity vector in the world frame

    // Default constructor initializes to zero/identity
    NominalState() : 
        p(Vec3::Zero()),
        v(Vec3::Zero()),
        q(Quat::Identity()),
        a_b(Vec3::Zero()),
        w_b(Vec3::Zero()),
        g(Vec3(0,0,-9.80665f)) {}
};
