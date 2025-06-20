#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../src/ESKF2.h"
#include "../src/Prelude.h"

namespace py = pybind11;
using namespace eskf;

PYBIND11_MODULE(pyeskf, m) {
    m.doc() = "Header‑only Error‑State Kalman Filter (local‑angle) bindings";

        // ---------------- Params struct -----------------
        py::class_<ESKF::Params>(m, "Params")
            .def(py::init<>())
            .def_readwrite("sigma_an", &ESKF::Params::sigma_an)
            .def_readwrite("sigma_wn", &ESKF::Params::sigma_wn)
            .def_readwrite("sigma_aw", &ESKF::Params::sigma_aw)
            .def_readwrite("sigma_ww", &ESKF::Params::sigma_ww)
            .def_readwrite("init_cov", &ESKF::Params::init_cov);

        // ---------------- Main ESKF class ---------------
        py::class_<ESKF>(m, "ESKF")
            .def(py::init<const ESKF::Params&>(), py::arg("params") = ESKF::Params())

            // -------- Nominal‑state accessors -----------
            .def_readwrite("p",   &ESKF::p_)
            .def_readwrite("v",   &ESKF::v_)
            .def_readwrite("q",   &ESKF::q_)
            .def_readwrite("a_b", &ESKF::a_b_)
            .def_readwrite("w_b", &ESKF::w_b_)
            .def_readwrite("g",   &ESKF::g_)
            .def_readwrite("P",   &ESKF::P_)

            // ---------------- Prediction ---------------
            .def("predict", &ESKF::predict,
                 py::arg("accel_meas"),
                 py::arg("gyro_meas"),
                 py::arg("dt"),
                 "IMU propagation step")

            // ---------------- Correction ---------------
            .def("correct",
                 [](ESKF &self,
                    const Eigen::VectorXd &y,
                    const Eigen::MatrixXd &Hx,
                    const Eigen::MatrixXd &Rvv,
                    py::function h_fun)
                 {
                     // wrap the Python callable so C++ sees an Eigen::VectorXd
                    auto h_cpp = [&](const ESKF& f) -> Eigen::VectorXd
                    {
                        py::gil_scoped_acquire gil;      // always safe
                        py::object out = h_fun(py::cast(&f, py::return_value_policy::reference));
                        return out.cast<Eigen::VectorXd>();
                    };

                    /* ✨ EXPLICIT TEMPLATE ARGUMENTS HERE ✨
                    *  1st  → type of the functor we just built
                    *  2nd  → the measurement‐vector type (Eigen::VectorXd in your API)
                    */
                    using Vec = Eigen::VectorXd;
                    self.correct<decltype(h_cpp), >(y, std::move(h_cpp), Hx, Rvv);
                 },
                 py::arg("y"), py::arg("Hx"), py::arg("Rvv"), py::arg("h_fun"),
                 R"pbdoc(Generic correction step.

    Arguments:
        y      (np.ndarray) : measurement vector
        Hx     (np.ndarray) : nominal‑state Jacobian (m×18)
        Rvv    (np.ndarray) : measurement noise covariance (m×m)
        h_fun  (callable)   : python function f(eskf) -> np.ndarray of size m
    )pbdoc")

            // -------- Utility --------------------------
            .def("get_horizontal_vel", &ESKF::getHorizontalVel,
                 "Returns (v_forward, v_lateral) in navigation frame");
}
