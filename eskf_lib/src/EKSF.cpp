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

    //TODO i need to optimize this thing lol
    static inline Eigen::Quaternionf qExp(const Eigen::Vector3f& theta) {
        float ang = theta.norm();
        if(ang < 1e-8f) return Eigen::Quaternionf::Identity();
        Eigen::Vector3f axis = theta / ang;
        float s = std::sin(0.5f*ang);
        return Eigen::Quaternionf(std::cos(0.5f*ang), axis.x()*s, axis.y()*s, axis.z()*s);
    }

    void ESKF::integrate(const Vec3& a_m, const Vec3& w_m, float dt) {
        a_tilde = a_m - a_b;
        skew(S, a_tilde); //S = skew(tilde)
        R_b2w = q.toRotationMatrix();
        Vec3 a_w = R_b2w * a_tilde + g;

        //euler 12
        p += v * dt; //+ 0.5 * a_w * dt * dt;
        v += a_w * dt;
        q = (q * qExp((w_m - w_b)*dt)).normalized();
    }

    void ESKF::predict(const Vec3& a_m, const Vec3& w_m, float dt) {
        //Position Prediction
        Fx.block<3,3>(0 , 3).diagonal().setConstant(dt);

        //Velocity Prediction
        Fx.block<3,3>(3,6) = -R_b2w * S * dt;
        Fx.block<3,3>(3,9)  = -R_b2w * dt;

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
