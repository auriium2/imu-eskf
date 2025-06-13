#include "Prelude.h"


namespace eskf {
    class Debiaser {
        public:
            Eigen::Vector3f acc_mean = Eigen::Vector3f::Zero();
            Eigen::Vector3f gyr_mean = Eigen::Vector3f::Zero();

            Debiaser(Vec3 g_fixed) {
                this->g_fixed = g_fixed;
            }

            void writeSample(const Vec3& a_m, const Vec3& w_m, const Quat& q_m, const Mat3& rot, const Vec3& dV, float dT);

            void finalize(Mat18& p0_out, Vec3& a_b_out, Vec3& w_b_out, Vec3& grav_out);


        private:
            Vec3 g_fixed;
            size_t num_samples = 0;

            Eigen::Vector3f ba_sample_mean = Eigen::Vector3f::Zero();


            Eigen::Matrix3f M2_acc = Eigen::Matrix3f::Zero();
            Eigen::Matrix3f M2_gyr = Eigen::Matrix3f::Zero();

            Eigen::Vector4f quat_sum = Eigen::Vector4f::Zero(); //meaningless, don't try to read this
            Eigen::Quaternionf q_mean = Eigen::Quaternionf::Identity();
            Eigen::Quaternionf q_mean_conj = Eigen::Quaternionf::Identity();
            Eigen::Vector3f q_bias = Eigen::Vector3f::Zero();
            Eigen::Matrix3f M2_q = Eigen::Matrix3f::Zero();

            Eigen::Vector3f vel_mean = Eigen::Vector3f::Zero();
            Eigen::Matrix3f M2_vel = Eigen::Matrix3f::Zero();

            // Position state
            Eigen::Vector3f cumulative_vel = Eigen::Vector3f::Zero();
            Eigen::Vector3f cumulative_pos = Eigen::Vector3f::Zero();
            Eigen::Vector3f pos_sum = Eigen::Vector3f::Zero();
            Eigen::Vector3f pos_sq_sum = Eigen::Vector3f::Zero();

            // Gravity state
            Eigen::Vector3f g_world_i_mean = Eigen::Vector3f::Zero();
            Eigen::Matrix3f M2_g = Eigen::Matrix3f::Zero();


    };
}
