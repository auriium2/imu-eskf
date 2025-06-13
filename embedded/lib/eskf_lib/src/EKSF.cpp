#include "ESKF.h"

namespace eskf {

    /**
     * Inline write to S
     */
    static inline void skew(Eigen::Matrix3f& S, const Eigen::Vector3f& v) {
        S <<
        0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    }



    /**
     * a_m is the measured acceleration in the local frame
     * so a_b must be the
     */
    void ESKF::integrate(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m, float dt) {
        a_tilde = a_m - a_b ;
        skew(S, a_tilde); //S = skew(tilde)
        R_b2w = q.toRotationMatrix(); //We know q is
        a_w = (R_b2w * a_tilde) + g; //gravity is already negative, so the real acceleration felt is a_tilde_world (free) + g

        //euler 12
        p += v * dt + 0.5 * a_w * dt * dt;
        v += a_w * dt;
        q = (q * qExp((w_m - w_b)*dt)).normalized();
    }

    void ESKF::predict(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m, float dt) {
        //Position Prediction
        Fx = Fx.Identity();
        Fx.block<3,3>(0 , 3).diagonal().setConstant(dt);

        //Velocity Prediction
        Fx.block<3,3>(3,6).noalias() = -R_b2w * S * dt;
        Fx.block<3,3>(3,9).noalias()  = -R_b2w * dt;

        //Heading Prediction
        skew(Omega, w_m - w_b); //Omega = skew(omegam-omegab)
        Fx.block<3,3>(6,6)  = Mat3::Identity() - Omega*dt;
        Fx.block<3,3>(6,12).diagonal().setConstant(-dt);


        Eigen::Matrix<float, 12, 1> dt_mask;

        dt_mask.segment<3>(0).setConstant(dt * dt);  // for sigma_an
        dt_mask.segment<3>(3).setConstant(dt * dt);  // for sigma_wn
        dt_mask.segment<3>(6).setConstant(dt);       // for sigma_aw
        dt_mask.segment<3>(9).setConstant(dt);       // for sigma_ww

        //Prefab Q = Fi * Qi * Fi^T (see julia script precompute_q.jl)
        auto pn = sigmas.array() * dt_mask.array();
        Q.block<12,12>(3,3).diagonal() = pn;

        P = Fx * P * Fx.transpose() + Q;
    }

}
