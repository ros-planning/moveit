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

#include <moveit/robot_model/robot_model.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/ros_msg_typecasters.h>
#include <moveit_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/** @cond IGNORE */

namespace py = pybind11;
namespace pi = moveit::planning_interface;

using moveit::python::throwDeserializationError;

class RobotInterfacePython : protected moveit::py_bindings_tools::ROScppInitializer
{
public:
  RobotInterfacePython(const std::string& robot_description, const std::string& ns = "")
    : moveit::py_bindings_tools::ROScppInitializer()
  {
    robot_model_ = pi::getSharedRobotModel(robot_description);
    if (!robot_model_)
      throw std::runtime_error("RobotInterfacePython: invalid robot model");
    current_state_monitor_ = pi::getSharedStateMonitor(robot_model_, pi::getSharedTF(), ns);
  }

  const char* getRobotName() const
  {
    return robot_model_->getName().c_str();
  }

  const std::vector<std::string>& getActiveJointNames() const
  {
    return robot_model_->getActiveJointModelNames();
  }

  std::vector<std::string> getGroupActiveJointNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getActiveJointModelNames();
    else
      return {};
  }

  const std::vector<std::string>& getJointNames() const
  {
    return robot_model_->getJointModelNames();
  }

  std::vector<std::string> getGroupJointNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getJointModelNames();
    else
      return {};
  }

  std::vector<std::string> getGroupJointTips(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::vector<std::string> tips;
      jmg->getEndEffectorTips(tips);
      return tips;
    }
    else
      return {};
  }

  const std::vector<std::string>& getLinkNames() const
  {
    return robot_model_->getLinkModelNames();
  }

  std::vector<std::string> getGroupLinkNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getLinkModelNames();
    else
      return {};
  }

  const std::vector<std::string>& getGroupNames() const
  {
    return robot_model_->getJointModelGroupNames();
  }

  py::list getJointLimits(const std::string& name) const
  {
    py::list result;
    const moveit::core::JointModel* jm = robot_model_->getJointModel(name);
    if (jm)
    {
      const std::vector<moveit_msgs::JointLimits>& lim = jm->getVariableBoundsMsg();
      for (const moveit_msgs::JointLimits& joint_limit : lim)
      {
        py::list l;
        l.append(joint_limit.min_position);
        l.append(joint_limit.max_position);
        result.append(l);
      }
    }
    return result;
  }

  const std::string& getPlanningFrame() const
  {
    return robot_model_->getModelFrame();
  }

  py::list getLinkPose(const std::string& name)
  {
    if (!ensureCurrentState())
      return py::list{};
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::LinkModel* lm = state->getLinkModel(name);
    if (lm)
    {
      // getGlobalLinkTransform() returns a valid isometry by contract
      const Eigen::Isometry3d& t = state->getGlobalLinkTransform(lm);
      std::array<double, 7> v;
      v[0] = t.translation().x();
      v[1] = t.translation().y();
      v[2] = t.translation().z();
      Eigen::Quaterniond q(t.linear());
      v[3] = q.x();
      v[4] = q.y();
      v[5] = q.z();
      v[6] = q.w();
      return py::cast(v);
    }
    return py::list{};
  }

  py::list getDefaultStateNames(const std::string& group)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      return py::cast(jmg->getDefaultStateNames());
    }
    return py::list{};
  }

  py::list getCurrentJointValues(const std::string& name)
  {
    if (!ensureCurrentState())
      return py::list{};
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::JointModel* jm = state->getJointModel(name);
    if (jm)
    {
      py::list l;
      const double* pos = state->getJointPositions(jm);
      const unsigned int sz = jm->getVariableCount();
      for (unsigned int i = 0; i < sz; ++i)
        l.append(pos[i]);
      return l;
    }

    return py::list{};
  }

  std::map<std::string, double> getJointValues(const std::string& group, const std::string& named_state)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return {};
    std::map<std::string, double> values;
    jmg->getVariableDefaultPositions(named_state, values);
    return values;
  }

  bool ensureCurrentState(double wait = 1.0)
  {
    if (!current_state_monitor_)
    {
      ROS_ERROR("Unable to get current robot state");
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
    {
      py::gil_scoped_acquire gr;
      current_state_monitor_->startStateMonitor();
      if (!current_state_monitor_->waitForCompleteState(wait))
        ROS_WARN("Joint values for monitored state are requested but the full state is not known");
    }
    return true;
  }

  moveit_msgs::RobotState getCurrentState()
  {
    if (!ensureCurrentState())
      throwDeserializationError();
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    moveit_msgs::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(*s, msg);
    return msg;
  }

  std::pair<std::string, std::string> getEndEffectorParentGroup(const std::string& group)
  {
    // name of the group that is parent to this end-effector group;
    // Second: the link this in the parent group that this group attaches to
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return { "", "" };
    return jmg->getEndEffectorParentGroup();
  }

  visualization_msgs::MarkerArray getRobotMarkersPythonDictList(std::map<std::string, double> const& values,
                                                                const std::vector<std::string>& links)
  {
    moveit::core::RobotStatePtr state;
    if (ensureCurrentState())
    {
      state = current_state_monitor_->getCurrentState();
    }
    else
    {
      state = std::make_shared<moveit::core::RobotState>(robot_model_);
    }

    const auto l = values.size();
    sensor_msgs::JointState joint_state;
    joint_state.name.reserve(l);
    joint_state.position.reserve(l);
    for (const auto& kv : values)
    {
      joint_state.name.push_back(kv.first);
      joint_state.position.push_back(kv.second);
    }
    state->setVariableValues(joint_state);
    visualization_msgs::MarkerArray msg;
    state->getRobotMarkers(msg, links);

    return msg;
  }

  visualization_msgs::MarkerArray getRobotMarkersPythonDict(std::map<std::string, double> const& values)
  {
    return getRobotMarkersPythonDictList(values, robot_model_->getLinkModelNames());
  }

  visualization_msgs::MarkerArray getRobotMarkersFromMsg(const moveit_msgs::RobotState& state_msg)
  {
    moveit::core::RobotState state(robot_model_);
    moveit::core::robotStateMsgToRobotState(state_msg, state);

    visualization_msgs::MarkerArray msg;
    state.getRobotMarkers(msg, state.getRobotModel()->getLinkModelNames());

    return msg;
  }

  visualization_msgs::MarkerArray getRobotMarkers()
  {
    if (!ensureCurrentState())
      throwDeserializationError();
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, s->getRobotModel()->getLinkModelNames());

    return msg;
  }

  visualization_msgs::MarkerArray getRobotMarkersPythonList(const std::vector<std::string>& links)
  {
    if (!ensureCurrentState())
      throwDeserializationError();

    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, links);

    return msg;
  }

  visualization_msgs::MarkerArray getRobotMarkersGroup(const std::string& group)
  {
    if (!ensureCurrentState())
      throwDeserializationError();

    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      visualization_msgs::MarkerArray msg;
      s->getRobotMarkers(msg, jmg->getLinkModelNames());
      return msg;
    }
    throw std::invalid_argument("Group not found");
  }

  visualization_msgs::MarkerArray getRobotMarkersGroupPythonDict(const std::string& group,
                                                                 std::map<std::string, double> const& values)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      throwDeserializationError();
    return getRobotMarkersPythonDictList(values, jmg->getLinkModelNames());
  }

  std::map<std::string, double> getCurrentVariableValues()
  {
    if (!ensureCurrentState())
      return {};

    return current_state_monitor_->getCurrentStateValues();
  }

  const char* getRobotRootLink() const
  {
    return robot_model_->getRootLinkName().c_str();
  }

  bool hasGroup(const std::string& group) const
  {
    return robot_model_->hasJointModelGroup(group);
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  ros::NodeHandle nh_;
};

PYBIND11_MODULE(pymoveit_robot_interface, m)
{
  py::class_<RobotInterfacePython>(m, "RobotInterface")
      .def(py::init<std::string, std::string>(), py::arg("robot_description"), py::arg("namespace") = std::string{})

      .def("get_joint_names", &RobotInterfacePython::getJointNames)
      .def("get_group_joint_names", &RobotInterfacePython::getGroupJointNames, py::arg("group"))
      .def("get_active_joint_names", &RobotInterfacePython::getActiveJointNames)
      .def("get_group_active_joint_names", &RobotInterfacePython::getGroupActiveJointNames, py::arg("group"))
      .def("get_group_default_states", &RobotInterfacePython::getDefaultStateNames, py::arg("group"))
      .def("get_group_joint_tips", &RobotInterfacePython::getGroupJointTips, py::arg("group"))
      .def("get_group_names", &RobotInterfacePython::getGroupNames)
      .def("get_link_names", &RobotInterfacePython::getLinkNames)
      .def("get_group_link_names", &RobotInterfacePython::getGroupLinkNames, py::arg("group"))
      .def("get_joint_limits", &RobotInterfacePython::getJointLimits, py::arg("name"))
      .def("get_link_pose", &RobotInterfacePython::getLinkPose, py::arg("name"))
      .def("get_planning_frame", &RobotInterfacePython::getPlanningFrame)
      .def("get_current_state", &RobotInterfacePython::getCurrentState)
      .def("get_current_variable_values", &RobotInterfacePython::getCurrentVariableValues)
      .def("get_current_joint_values", &RobotInterfacePython::getCurrentJointValues, py::arg("name"))
      .def("get_joint_values", &RobotInterfacePython::getJointValues, py::arg("group"), py::arg("named_state"))
      .def("get_robot_root_link", &RobotInterfacePython::getRobotRootLink)
      .def("has_group", &RobotInterfacePython::hasGroup, py::arg("group"))
      .def("get_robot_name", &RobotInterfacePython::getRobotName)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkers)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonList, py::arg("links"))
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersFromMsg, py::arg("robot_state_msg"))
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDictList, py::arg("joint_values"),
           py::arg("links"))
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDict, py::arg("joint_values"))
      .def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroup, py::arg("group"))
      .def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroupPythonDict, py::arg("group"),
           py::arg("joint_values"))
      .def("get_parent_group", &RobotInterfacePython::getEndEffectorParentGroup, py::arg("group"))
      // keep semicolon on next line
      ;
}

/** @endcond */
