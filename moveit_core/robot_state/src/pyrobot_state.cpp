#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

namespace py = pybind11;
using namespace robot_state;

void def_robot_state_bindings(py::module& m)
{
  m.doc() = "Representation of a robot's state. This includes position, velocity, acceleration and effort.";
  py::class_<RobotState, RobotStatePtr>(m, "RobotState")
      .def(py::init<const robot_model::RobotModelConstPtr&>(), py::arg("robot_model"))
      .def("setToRandomPositions", py::overload_cast<>(&RobotState::setToRandomPositions))
      .def("setToRandomPositions", py::overload_cast<const JointModelGroup*>(&RobotState::setToRandomPositions))
      .def("getJointModelGroup", &RobotState::getJointModelGroup, py::return_value_policy::reference)
      .def("getJointModel", &RobotState::getJointModel, py::return_value_policy::reference)
      .def("getLinkModel", &RobotState::getLinkModel, py::return_value_policy::reference)
      .def("getVariableNames", &RobotState::getVariableNames)
      .def("getVariablePositions", py::overload_cast<>(&RobotState::getVariablePositions, py::const_),
           py::return_value_policy::reference)
      .def("getVariableCount", &RobotState::getVariableCount)
      .def("hasVelocities", &RobotState::hasVelocities)
      .def("setJointGroupPositions",
           py::overload_cast<const std::string&, const std::vector<double>&>(&RobotState::setJointGroupPositions))
      .def("setJointGroupPositions",
           py::overload_cast<const JointModelGroup*, const std::vector<double>&>(&RobotState::setJointGroupPositions))
      .def("satisfiesBounds",
           py::overload_cast<const JointModelGroup*, double>(&RobotState::satisfiesBounds, py::const_),
           py::arg("joint_model_group"), py::arg("margin") = 0.0)
      .def("update", &RobotState::update, py::arg("force") = false)
      .def("printStateInfo",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStateInfo(ss);
             return ss.str();
           })
      .def("printStatePositions",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStatePositions(ss);
             return ss.str();
           })
      .def("__repr__",
           [](const RobotState& s) {
             std::stringstream ss;
             s.printStatePositions(ss);
             return ss.str();
           })
      //
      ;

  m.def("jointStateToRobotState", &jointStateToRobotState);
  m.def(
      "robotStateToRobotStateMsg",
      [](const RobotState& state, bool copy_attached_bodies) {
        moveit_msgs::RobotState state_msg;
        robotStateToRobotStateMsg(state, state_msg, copy_attached_bodies);
        return state_msg;
      },
      py::arg("state"), py::arg("copy_attached_bodies") = true);
}
