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
