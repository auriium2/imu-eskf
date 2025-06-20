#pragma once

#include "Prelude.h"

namespace eskf {

class ESKF {
public:
    //--------------- parameter bundle --------------------
    struct Params {
        // process-noise std-devs
        float sigma_an = 0.05;   ///< m/s² white accelerometer noise
        float sigma_wn = 0.002;  ///< rad/s white gyro noise
        float sigma_aw = 0.0005; ///< m/s²/√s accelerometer random walk
        float sigma_ww = 0.0001; ///< rad/s/√s gyro random walk
        // initial covariance diagonal
        float init_cov = 1e-6;
    };

    ESKF(const Params& p)
        : params_{p}
    {
        P_.setIdentity();
        P_ *= params_.init_cov;
        g_ << 0.0, 0.0, -9.81f; //TODO bad
    }

    //--------------- prediction (IMU) --------------------
    void predict(const Eigen::Vector3f& a_m,
                 const Eigen::Vector3f& w_m,
                 float dt)
    {
        using namespace Eigen;

        // 1) nominal propagation ---------------------------------
        Vector3f a_tilde = a_m - a_b_;
        Matrix3f S = skew(a_tilde);

        Matrix3f R = q_.toRotationMatrix();               // body→nav
        Vector3f a_world = R * a_tilde + g_;

        p_ += v_ * dt;
        v_ += a_world * dt;
 
        Quaternionf dq = qExp((w_m - w_b_) * dt);         // exp(½ ωdt)
        q_ = (q_ * dq).normalized();

        // 2) build F_x and Q_i ----------------------------------
        Matrix<float,18,18> Fx = Matrix<float,18,18>::Identity();
        Fx.block<3,3>(0,3)  = Matrix3f::Identity()*dt;
        Fx.block<3,3>(3,6)  = -R * S * dt;
        Fx.block<3,3>(3,9)  = -R * dt;
        Fx.block<3,3>(3,15) = Matrix3f::Identity()*dt;

        Matrix3f Omega = skew(w_m - w_b_);
        Fx.block<3,3>(6,6)  = Matrix3f::Identity() - Omega*dt;
        Fx.block<3,3>(6,12) = -Matrix3f::Identity()*dt;

        // process-noise covariance
        Matrix<float,12,12> Qi = Matrix<float,12,12>::Zero();
        Qi.block<3,3>(0,0)  = std::pow(params_.sigma_an,2)*dt*dt * Matrix3f::Identity();
        Qi.block<3,3>(3,3)  = std::pow(params_.sigma_wn,2)*dt*dt * Matrix3f::Identity();
        Qi.block<3,3>(6,6)  = std::pow(params_.sigma_aw,2)*dt     * Matrix3f::Identity();
        Qi.block<3,3>(9,9)  = std::pow(params_.sigma_ww,2)*dt     * Matrix3f::Identity();

        Matrix<float,18,12> Fi = Matrix<float,18,12>::Zero();
        Fi.block<3,3>(3,0)  = Matrix3f::Identity();
        Fi.block<3,3>(6,3)  = Matrix3f::Identity();
        Fi.block<3,3>(9,6)  = Matrix3f::Identity();
        Fi.block<3,3>(12,9) = Matrix3f::Identity();

        // 3) covariance propagation -----------------------------
        P_ = Fx*P_*Fx.transpose() + Fi*Qi*Fi.transpose();
    }

    //--------------- correction (generic) -----------------
    template<typename HFunc, int N>
    void correct(const Eigen::Matrix<float,N,1>& y,
                 HFunc&&        hFun,  ///< function: const ESKF& -> Eigen::VectorXd
                 const Eigen::Matrix<float,N,18>& Hx,
                 const Eigen::Matrix<float,N,N>& Rvv)
    {
        using namespace Eigen;
        // 1) Build error-Jacobian H = Hx * X_delta ----------------
        Matrix<float,18,18> X = Matrix<float,18,18>::Identity();

        // quaternion block ∂(q ⊗ δq)/∂δθ   (w, x, y, z)
        const float w = q_.w();
        const float x = q_.x();
        const float yq = q_.y();
        const float z = q_.z();

        Matrix<float,4,3> Qdq;
        Qdq << -x, -yq, -z,
                w, -z,  yq,
                z,  w, -x,
               -yq,  x,  w;


        X.block<4,3>(6,6) = Qdq;

        Matrix<float,N,18> H = Hx * X;   // H: (m×18)

        // 2) Kalman update -------------------------
        Matrix<float,N,N> S = H * P_ * H.transpose() + Rvv;
        Matrix<float,18,N> K = P_ * H.transpose() * S.inverse();

        auto innovation = y - hFun(*this);
        Matrix<float,18,1> delta = K * innovation;

        // Joseph covariance update
        Matrix<float,18,18> I_KH = Matrix<float,18,18>::Identity() - K*H;
        P_ = I_KH*P_*I_KH.transpose() + K*Rvv*K.transpose();

        // 3) Inject error state --------------------
        // p_ += delta.segment<3>(0);      // Optional: see Solà eqn (282)
        v_ += delta.segment<3>(3);

        Quaternionf dq_inj = qExp(delta.segment<3>(6));
        q_ = (q_ * dq_inj).normalized();

        a_b_ += delta.segment<3>(9);
        w_b_ += delta.segment<3>(12);
        g_   += delta.segment<3>(15);

        // 4) Reset error mean & rotate covariance ---
        Matrix<float,18,18> G = Matrix<float,18,18>::Identity();
        G.block<3,3>(6,6) = Matrix3f::Identity() - 0.5*skew(delta.segment<3>(6));
        P_ = G*P_*G.transpose();
    }

    //---------------- helper API --------------------------
    /** forward & lateral velocity in navigation frame */
    std::pair<float,float> getHorizontalVel() const {
        Eigen::Matrix3f Rnb = q_.toRotationMatrix();
        float yaw = std::atan2( Rnb(0,0), Rnb(1,0) );
        float vE = v_.x();
        float vN = v_.y();
        float v_fw  = vE * std::sin(yaw) + vN * std::cos(yaw);
        float v_lat = vE * std::cos(yaw) - vN * std::sin(yaw);
        return {v_fw, v_lat};
    }

    //---------------- public state ------------------------
    Eigen::Vector3f p_   = Eigen::Vector3f::Zero();
    Eigen::Vector3f v_   = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3f a_b_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f w_b_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f g_;
    Eigen::Matrix<float,18,18> P_;

private:
    Params params_;

    //---------------- static helpers ----------------------
    static inline Eigen::Matrix3f skew(const Eigen::Vector3f& v)
    {
        Eigen::Matrix3f S;
        S <<     0, -v.z(),  v.y(),
               v.z(),     0, -v.x(),
              -v.y(),  v.x(),     0;
        return S;
    }

    /** small-angle quaternion exponential  exp(½ θ) */
    static inline Eigen::Quaternionf qExp(const Eigen::Vector3f& theta)
    {
        float ang = theta.norm();
        if (ang < 1e-8)
            return Eigen::Quaternionf::Identity();
        Eigen::Vector3f axis = theta / ang;
        float half = 0.5 * ang;
        return {cosf(half),
                axis.x()*sinf(half),
                axis.y()*sinf(half),
                axis.z()*sinf(half)};
    }
};

} // namespace eskf
