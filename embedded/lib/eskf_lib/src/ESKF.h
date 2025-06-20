#include "Prelude.h"
/**
 * Some shorthand used in this file:
 *
 * CT: continous time
 * DT: Discrete Time
 * _b is usually bias
 */
namespace eskf {

    enum IntegratorMode {
        EULER_PART,
        EULER_FULL
    };


    class ESKF {
    public: //Were this not cpp, i would not make everything public.
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


    private:
        //CT noise, filled with signa_an, sigma_wn, sigma_aw, sigma_ww
        ProcessNoiseSized sigmas = ProcessNoiseSized::Zero();




    public:
        //Nominal State
        NominalState state;

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

        ESKF(const ProcessNoise& noise, const NominalState& state) : state(state) {
            sigmas = noise.toVector();
        }

        static inline Eigen::Quaternionf qExp(const Eigen::Vector3f& theta)
        {
            float ang = theta.norm();
            if(ang < 1e-8f) return Eigen::Quaternionf::Identity();
            Eigen::Vector3f axis = theta / ang;
            float s = std::sin(0.5f*ang);
            return Eigen::Quaternionf(std::cos(0.5f*ang), axis.x()*s, axis.y()*s, axis.z()*s);
        }

        // static inline Eigen::Quaternionf qExp(const Eigen::Vector3f& theta) noexcept {
        //     const float ang2 = theta.squaredNorm();

        //     if (ang2 < 1e-8f * 1e-8f) {
        //         return Eigen::Quaternionf(1.0f, 0.5f * theta.x(), 0.5f * theta.y(), 0.5f * theta.z());
        //     }
        //     const float ang = std::sqrt(ang2);
        //     const float halfAng = 0.5f * ang;

        //     float s, c; sincosf(halfAng, &s, &c);

        //     const float invAng = 1.0f / ang;
        //     const float scale  = s * invAng;

        //     return Eigen::Quaternionf(c, theta.x() * scale, theta.y() * scale, theta.z() * scale);
        // }


        /**
         * Runs the integration loop at 1khz
         */
        void integrate(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m, float dt);

        /**
         * Every 100hz, call predict after integrate to do a kalman prediction
         */
        void predict(const Eigen::Vector3f& a_m, const Eigen::Vector3f& w_m, float dt);

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

        template<int M, class HFun>
        void correct(const Eigen::Matrix<float,M,1>& y,
                             HFun&& h,
                             const Eigen::Matrix<float,M,18>& Hx,
                             const Eigen::Matrix<float,M,M>&  Rvv)
        {
            Eigen::Matrix<float,18,18> X = Eigen::Matrix<float,18,18>::Identity();
            const auto& qc = state.q.coeffs(); float w=qc[3], x=qc[0], y_=qc[1], z=qc[2];
            Eigen::Matrix<float,4,3> Qdq;
            Qdq << -x,-y_,-z,
                    w,-z, y_,
                    z, w,-x,
                   -y_,x, w;
            X.block<4,3>(6,6) = 0.5f*Qdq;

            auto H = Hx * X;
            Eigen::Matrix<float,M,1> y_pred = std::forward<HFun>(h)(*this);
            Eigen::Matrix<float,M,M> S = H*P*H.transpose() + Rvv;
            Eigen::Matrix<float,18,M> K = P*H.transpose() * S.inverse();
            Eigen::Matrix<float,18,1> delta = K * (y - y_pred);

            // Inject
            state.p   += delta.segment<3>(0);
            state.v   += delta.segment<3>(3);
            state.q    = (state.q * qExp(delta.segment<3>(6))).normalized();
            state.a_b += delta.segment<3>(9);
            state.w_b += delta.segment<3>(12);
            state.g   += delta.segment<3>(15);

            auto I_KH = Eigen::Matrix<float,18,18>::Identity() - K*H;
            P = I_KH*P*I_KH.transpose() + K*Rvv*K.transpose();
        }

         // namespace eskf


        // template<int M, class HFun> void correct(const Eigen::Matrix<float,M,1>& y, HFun&& h, const Eigen::Matrix<float,M,18>& Hx,const Eigen::Matrix<float,M,M>&  Rvv) {

        //     // ——— 1) Build the small‐angle “inject” matrix X ———
        //     Eigen::Matrix<float,18,18> X = Eigen::Matrix<float,18,18>::Identity();

        //     auto qc = state.q.coeffs();
        //     const float w=qc[3], x=qc[0], y_=qc[1], z=qc[2];
        //     Eigen::Matrix<float,4,3> Qdq;
        //     Qdq << -x,-y_,-z,
        //             w,-z, y_,
        //             z, w,-x,
        //            -y_,x,  w;
        //     X.block<4,3>(6,6).noalias() = 0.5f * Qdq;

        //     Eigen::Matrix<float,M,18> H = Eigen::Matrix<float,M,18>::Zero();
        //     H.noalias() = Hx * X;

        //     Eigen::Matrix<float,M,18> HP = Eigen::Matrix<float,M,18>::Zero();
        //     Eigen::Matrix<float,M,M> S = Eigen::Matrix<float,M,M>::Zero();
        //     HP.noalias() = H * P;
        //     S.noalias() = HP * H.transpose();
        //     S += Rvv;

        //     Eigen::LLT<Eigen::Matrix<float,M,M>> llt; llt.compute(S);
        //     //llt.compute(S);
        //     Eigen::Matrix<float,M,18> SiHP = llt.solve(HP);
        //     Eigen::Matrix<float,18,M> K = SiHP.transpose();

        //     Eigen::Matrix<float,M,1> y_hat = std::forward<HFun>(h)(*this);
        //     Eigen::Matrix<float,18,1> del = K * (y - y_hat);

        //     state.p += del.template segment<3>( 0);
        //     state.v += del.template segment<3>( 3);
        //     state.q = (state.q * qExp(del.template segment<3>(6))).normalized();
        //     state.a_b += del.template segment<3>( 9);
        //     state.w_b += del.template segment<3>(12);
        //     state.g += del.template segment<3>(15);

        //     P.noalias() -= K * HP;
        // }
    };


}
