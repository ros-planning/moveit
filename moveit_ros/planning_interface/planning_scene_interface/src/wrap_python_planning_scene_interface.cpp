/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Ioan Sucan */

#ifndef PY_SSIZE_T_CLEAN
#define PY_SSIZE_T_CLEAN
#endif

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/ros_msg_typecasters.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/** @cond IGNORE */

namespace py = pybind11;

namespace moveit
{
namespace planning_interface
{
class PlanningSceneInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public PlanningSceneInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfaceWrapper(const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer(), PlanningSceneInterface(ns)
  {
  }
};
}  // namespace planning_interface
}  // namespace moveit

PYBIND11_MODULE(pymoveit_planning_scene_interface, m)
{
  using moveit::planning_interface::PlanningSceneInterface;
  using moveit::planning_interface::PlanningSceneInterfaceWrapper;

  py::class_<PlanningSceneInterfaceWrapper>(m, "PlanningSceneInterface")
      .def(py::init<std::string>(), py::arg("namespace") = std::string(""))

      .def("get_known_object_names", &PlanningSceneInterface::getKnownObjectNames, py::arg("with_type") = false)
      .def("get_known_object_names_in_roi",
           py::overload_cast<double, double, double, double, double, double, bool>(
               &PlanningSceneInterface::getKnownObjectNamesInROI),
           py::arg("minx"), py::arg("miny"), py::arg("minz"), py::arg("maxx"), py::arg("maxy"), py::arg("maxz"),
           py::arg("with_type") = false)
      .def("get_object_poses", &PlanningSceneInterface::getObjectPoses, py::arg("object_ids"))
      .def("get_objects", &PlanningSceneInterface::getObjects, py::arg("object_ids") = std::vector<std::string>{})
      .def("get_attached_objects", &PlanningSceneInterface::getAttachedObjects,
           py::arg("object_ids") = std::vector<std::string>{})
      .def("apply_planning_scene", &PlanningSceneInterface::applyPlanningScene, py::arg("planning_scene"))
      // keep semicolon on next line
      ;
}

/** @endcond */
