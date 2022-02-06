#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace mav_trajectory_generation;

PYBIND11_MODULE(mav_traj_gen, m) {
  m.doc() = "mav_trajectory_generation python bindings";

  py::class_<Segment>(m, "Segment")
      .def("getPolynomialsRef", &Segment::getPolynomialsRef)
      .def("evaluate", &Segment::evaluate);

  py::class_<Polynomial>(m, "Polynomial")
      .def("getCoefficients", &Polynomial::getCoefficients);

  py::class_<Vertex>(m, "Vertex")
      .def(py::init<size_t>())
      .def("addConstraint",
           static_cast<void (Vertex::*)(int, const Eigen::VectorXd&)>(
               &Vertex::addConstraint));

  py::class_<Trajectory>(m, "Trajectory")
      .def(py::init<>())
      .def("get_segments", &Trajectory::segments)
      .def("get_segment_times", &Trajectory::getSegmentTimes);

  py::class_<NonlinearOptimizationParameters>(m,
                                              "NonlinearOptimizationParameters")
      .def(py::init<>())
      .def_readwrite("time_penalty",
                     &NonlinearOptimizationParameters::time_penalty);

  // TODO(laura) nontype template parameter, how to do?
  py::class_<PolynomialOptimizationNonLinear<10>>(
      m, "PolynomialOptimizationNonLinear")
      .def(py::init<size_t, const NonlinearOptimizationParameters&>())
      .def("addMaximumMagnitudeConstraint",
           &PolynomialOptimizationNonLinear<10>::addMaximumMagnitudeConstraint)
      .def("optimize", &PolynomialOptimizationNonLinear<10>::optimize)
      .def("setupFromVertices",
           &PolynomialOptimizationNonLinear<10>::setupFromVertices)
      .def("getTrajectory", &PolynomialOptimizationNonLinear<10>::getTrajectory)
      .def("getTotalCostWithSoftConstraints",
           &PolynomialOptimizationNonLinear<
               10>::getTotalCostWithSoftConstraints);

  py::enum_<derivative_order>(m, "derivative_order")
      .value("POSITION", POSITION)
      .value("VELOCITY", VELOCITY)
      .value("ACCELERATION", ACCELERATION)
      .value("JERK", JERK)
      .value("SNAP", SNAP)
      .value("ORIENTATION", ORIENTATION)
      .value("ANGULAR_VELOCITY", ANGULAR_VELOCITY)
      .value("ANGULAR_ACCELERATION", ANGULAR_ACCELERATION)
      .value("INVALID", INVALID)
      .export_values();

  m.def("estimateSegmentTimes", &estimateSegmentTimes);
}
