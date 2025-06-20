#include "Debias.h"
#include "Prelude.h"
//#include "usb/SerialUSB.h"


namespace eskf {

    /**
     *
     */
    void Debiaser::writeSample(const Vec3& a_m, const Vec3& w_m, const Quat& q_m, const Mat3& rot, const Vec3& dV, float dT) {
        num_samples++;

        // Phase 1: Update true means
        acc_mean += (a_m - acc_mean) / num_samples;
        gyr_mean += (w_m - gyr_mean) / num_samples;

        quat_sum[0] += q_m.w();
        quat_sum[1] += q_m.x();
        quat_sum[2] += q_m.y();
        quat_sum[3] += q_m.z();

        q_mean = Eigen::Quaternionf(
            quat_sum[0] / num_samples,
            quat_sum[1] / num_samples,
            quat_sum[2] / num_samples,
            quat_sum[3] / num_samples
        );
        q_mean.normalize();
        q_mean_conj = q_mean.conjugate();

        // Phase 2: Update covariances

        //R^T * -g
        // r goes from body to world
        // so r^t goes from world to body
        // so

        Vec3 g_body = rot.transpose() * Vec3(0, 0, -9.80665);;
        Vec3 ba_sample = a_m + g_body;
        ba_sample_mean += (ba_sample - ba_sample_mean) / num_samples;





        Vec3 delta_acc = ba_sample - acc_mean;
        M2_acc += delta_acc * delta_acc.transpose();

        //gyro
        Vec3 delta_gyr = w_m - gyr_mean;
        M2_gyr += delta_gyr * delta_gyr.transpose();

        //q
        Quat dq = q_m * q_mean_conj;
        dq.normalize();
        Vec3 q_err = 2 * dq.vec();

        q_bias += (q_err - q_bias) / num_samples; //We don't actually care about this

        Vec3 local_delta_q = q_err - q_bias;
        M2_q += local_delta_q * local_delta_q.transpose();

        // Velocity calculation
        Vec3 dv_free_b = dV - g_body * dT;
        Vec3 dv_free_w = rot * dv_free_b;

        vel_mean += (dv_free_w - vel_mean) / num_samples;

        Vec3 delta_vel = dv_free_w - vel_mean;
        M2_vel += delta_vel * delta_vel.transpose();

        // Position calculations
        // Update cumulative velocity and position
        cumulative_vel += dv_free_w;
        cumulative_pos += cumulative_vel * dT;

        pos_sum += cumulative_pos;
        pos_sq_sum += cumulative_pos.cwiseProduct(cumulative_pos);

        // Gravity calculation
        Vec3 a_corr = a_m - ba_sample; //acc mean is acting as bias here
        Vec3 g_world_i = rot * -a_corr; //negative, since the accelerometer is measuring the free accel, at rest the free accel should be directly opposite to the world acceleration

        // Update gravity covariance using Welford's online algorithm
        Vec3 delta_g = g_world_i - g_world_i_mean;
        M2_g += delta_g * delta_g.transpose();

        g_world_i_mean += (g_world_i - g_world_i_mean) / num_samples;

    }

    // Finalize the calibration - compute final covariances
    void Debiaser::finalize(Mat18& p0_out, Vec3& a_b_out, Vec3& w_b_out, Vec3& grav_out) {



        // if (num_samples <= 1) {
        //     return; //TODO: this needs to be better
        // }

        Mat3 P_acc = M2_acc / (num_samples - 1);
        Mat3 P_w = M2_gyr / (num_samples - 1);
        Mat3 P_q = M2_q / (num_samples - 1);


        //velocity P
        Eigen::Vector3f var_vel = Eigen::Vector3f::Zero();
        for (int j = 0; j < 3; j++) {
            var_vel[j] = M2_vel(j, j) / (num_samples - 1);
        }



        Mat3 P_vel = var_vel.asDiagonal();

        Vec3 pos_mean = pos_sum / num_samples;
        Vec3 var_pos = (pos_sq_sum - pos_sum.cwiseProduct(pos_mean) * num_samples) / (num_samples - 1);
        Mat3 P_pos = var_pos.asDiagonal();

        Mat3 P_g = M2_g / (num_samples - 1);

        // //build initial conditions

        a_b_out = ba_sample_mean;
        w_b_out = gyr_mean;
        grav_out = g_fixed; //TODO change


        // //build covariance final

        p0_out.block<3,3>(0,0) = P_pos;
        p0_out.block<3,3>(3,3) = P_vel;
        p0_out.block<3,3>(6,6) = P_q;
        p0_out.block<3,3>(9,9) = P_acc;
        p0_out.block<3,3>(12,12) = P_w;
        p0_out.block<3,3>(15,15) = P_g;

    }

}
