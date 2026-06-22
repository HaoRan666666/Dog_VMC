#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kinematics/leg_kinematics.h"
#include "kinematics/types.h"

namespace py = pybind11;

PYBIND11_MODULE(kinematics_py, m) {
    m.doc() = "Leg kinematics: forward/inverse/jacobian/footForceToTorques";

    py::class_<vmc::Vec3>(m, "Vec3")
        .def(py::init<float, float, float>())
        .def_readwrite("x", &vmc::Vec3::x)
        .def_readwrite("y", &vmc::Vec3::y)
        .def_readwrite("z", &vmc::Vec3::z);

    py::class_<vmc::LegKinematics>(m, "LegKinematics")
        .def(py::init([](float L1, float L2, float dx, float dy) {
            vmc::LegKinematics::Params p;
            p.L1 = L1; p.L2 = L2; p.dx = dx; p.dy = dy;
            return std::make_unique<vmc::LegKinematics>(p);
        }), py::arg("L1")=0.2125f, py::arg("L2")=0.25025f,
            py::arg("dx")=0.0f, py::arg("dy")=0.0f)
        .def("forward", [](vmc::LegKinematics &kin, const std::array<float, 3> &q) {
            return kin.forward(q);
        })
        .def("inverse", [](vmc::LegKinematics &kin, const vmc::Vec3 &foot) {
            return kin.inverse(foot);
        })
        .def("foot_force_to_torques",
             [](vmc::LegKinematics &kin,
                const std::array<float, 3> &q,
                const vmc::Vec3 &force) {
                 return kin.footForceToTorques(q, force);
             });
}
