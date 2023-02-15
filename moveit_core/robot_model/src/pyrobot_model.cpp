/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <moveit/py_bindings_tools/ros_msg_typecasters.h>

namespace py = pybind11;
using namespace robot_model;

void def_robot_model_bindings(py::module& m)
{
  m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def("get_link_model_names", &JointModelGroup::getLinkModelNames)
      .def("get_joint_model_names", &JointModelGroup::getJointModelNames)
      .def("get_active_joint_model_names", &JointModelGroup::getActiveJointModelNames)
      .def("get_default_state_names", &JointModelGroup::getDefaultStateNames)
      .def("get_end_effector_tips",
           [](const JointModelGroup& self) {
             std::vector<std::string> tips;
             self.getEndEffectorTips(tips);
             return tips;
           })
      // keep semicolon on next line
      ;

  py::class_<RobotModel, RobotModelPtr>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("get_name", &RobotModel::getName)
      .def("get_model_frame", &RobotModel::getModelFrame)
      .def("get_joint_model_group_names", &RobotModel::getJointModelGroupNames)
      .def("get_joint_model_group", py::overload_cast<const std::string&>(&RobotModel::getJointModelGroup, py::const_),
           py::return_value_policy::reference_internal, py::arg("group"))
      .def("get_link_model_names", &RobotModel::getLinkModelNames)
      .def("get_joint_model_names", &RobotModel::getJointModelNames)
      .def("get_active_joint_model_names", &RobotModel::getActiveJointModelNames)

      .def("get_planning_frame", &RobotModel::getModelFrame)
      .def("get_robot_root_link", &RobotModel::getRootLinkName)
      .def("has_group", &RobotModel::hasJointModelGroup, py::arg("group"))
      .def("get_robot_name", &RobotModel::getName)
      // keep semicolon on next line
      ;
}
