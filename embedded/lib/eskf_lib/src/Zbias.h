#pragma once

#include "Prelude.h"
#include <cmath>


namespace eskf {
    class ZDebiaser {

        size_t num_samples = 0;

        public:
            Vec3 acc_sum = Vec3::Zero();
            //Eigen::Vector3f mag_mean = Eigen::Vector3f::Zero();

            void writeSample(const Vec3& a_m,const float dT) {
                num_samples++;
                acc_sum += a_m;
            }

            Quat euler2quat(float roll, float pitch, float yaw) {
                Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

                return yawAngle * pitchAngle * rollAngle;
            }

            Quat finalize() {
                Vec3 acc_mean = acc_sum / num_samples;

                float heading = 0;
                float roll = atan2f(-acc_mean.y(), -acc_mean.z());
                float pitch = atan2f(acc_mean.x(), sqrtf(acc_mean.y() * acc_mean.y() + acc_mean.z() * acc_mean.z()));

                return euler2quat(roll, pitch, heading);
            }

    };
}
