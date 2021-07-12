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
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;
using namespace robot_model;

void def_robot_model_bindings(py::module& m)
{
  m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
  py::class_<RobotModel, RobotModelPtr>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("getActiveJointModelsBounds", &RobotModel::getActiveJointModelsBounds, py::return_value_policy::reference)
      .def("getMaximumExtent", py::overload_cast<>(&RobotModel::getMaximumExtent, py::const_))
      .def("getMaximumExtent", py::overload_cast<const JointBoundsVector&>(&RobotModel::getMaximumExtent, py::const_))
      .def("getModelFrame", &RobotModel::getModelFrame)
      .def("getName", &RobotModel::getName)
      .def("getVariableBounds", &RobotModel::getVariableBounds)
      .def("getVariableCount", &RobotModel::getVariableCount)
      .def("getVariableDefaultPositions",
           py::overload_cast<std::map<std::string, double>&>(&RobotModel::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<std::vector<double>&>(&RobotModel::getVariableDefaultPositions, py::const_))
      .def("getVariableIndex", &RobotModel::getVariableIndex)
      .def("getVariableNames", &RobotModel::getVariableNames)
      .def("getRootJoint", &RobotModel::getRootJoint, py::return_value_policy::reference)
      .def("getRootJointName", &RobotModel::getRootJointName)
      .def("getRootLink", &RobotModel::getRootLink, py::return_value_policy::reference)
      .def("getJointModel", py::overload_cast<std::string const&>(&RobotModel::getJointModel, py::const_),
           py::return_value_policy::reference)
      .def("getJointModelNames", &RobotModel::getJointModelNames)
      .def("getJointOfVariable", py::overload_cast<int>(&RobotModel::getJointOfVariable, py::const_))
      .def("getJointOfVariable", py::overload_cast<const std::string&>(&RobotModel::getJointOfVariable, py::const_))
      .def("getJointModelCount", &RobotModel::getJointModelCount)
      .def("getJointModelGroup", py::overload_cast<std::string const&>(&RobotModel::getJointModelGroup, py::const_),
           py::return_value_policy::reference)
      .def("hasJointModelGroup", &RobotModel::hasJointModelGroup)
      //
      ;

  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def("canSetStateFromIK", &JointModelGroup::canSetStateFromIK)
      .def("getActiveJointModelNames", &JointModelGroup::getActiveJointModelNames)
      .def("getActiveJointModels", &JointModelGroup::getActiveJointModels, py::return_value_policy::reference)
      .def("getActiveJointModelsBounds", &JointModelGroup::getActiveJointModelsBounds,
           py::return_value_policy::reference)
      .def("getAttachedEndEffectorNames", &JointModelGroup::getAttachedEndEffectorNames)
      .def("getCommonRoot", &JointModelGroup::getCommonRoot, py::return_value_policy::reference)
      .def("getContinuousJointModels", &JointModelGroup::getContinuousJointModels, py::return_value_policy::reference)
      .def("getDefaultIKTimeout", &JointModelGroup::getDefaultIKTimeout)
      .def("getDefaultStateNames", &JointModelGroup::getDefaultStateNames)
      .def("getEndEffectorName", &JointModelGroup::getEndEffectorName)
      .def("getEndEffectorParentGroup", &JointModelGroup::getEndEffectorParentGroup, py::return_value_policy::reference)
      .def("getEndEffectorTips",
           py::overload_cast<std::vector<const LinkModel*>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("getEndEffectorTips",
           py::overload_cast<std::vector<std::string>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("getFixedJointModels", &JointModelGroup::getFixedJointModels, py::return_value_policy::reference)
      .def("getJointModel", &JointModelGroup::getJointModel, py::return_value_policy::reference)
      .def("getJointModelNames", &JointModelGroup::getJointModelNames)
      .def("getJointModels", &JointModelGroup::getJointModels, py::return_value_policy::reference)
      .def("getJointRoots", &JointModelGroup::getJointRoots, py::return_value_policy::reference)
      .def("getLinkModel", &JointModelGroup::getLinkModel, py::return_value_policy::reference)
      .def("getLinkModelNames", &JointModelGroup::getLinkModelNames)
      .def("getLinkModelNamesWithCollisionGeometry", &JointModelGroup::getLinkModelNamesWithCollisionGeometry)
      .def("getLinkModels", &JointModelGroup::getLinkModels, py::return_value_policy::reference)
      .def("getMaximumExtent", py::overload_cast<>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("getMaximumExtent",
           py::overload_cast<const JointBoundsVector&>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("getMimicJointModels", &JointModelGroup::getMimicJointModels, py::return_value_policy::reference)
      .def("getName", &JointModelGroup::getName)
      .def("getOnlyOneEndEffectorTip", &JointModelGroup::getOnlyOneEndEffectorTip, py::return_value_policy::reference)
      .def("getParentModel", &JointModelGroup::getParentModel, py::return_value_policy::reference)
      .def("getSubgroupNames", &JointModelGroup::getSubgroupNames)
      .def("getSubgroups", &JointModelGroup::getSubgroups, py::return_value_policy::reference)
      .def("getUpdatedLinkModelNames", &JointModelGroup::getUpdatedLinkModelNames)
      .def("getUpdatedLinkModels", &JointModelGroup::getUpdatedLinkModels, py::return_value_policy::reference)
      .def("getUpdatedLinkModelsSet", &JointModelGroup::getUpdatedLinkModelsSet, py::return_value_policy::reference)
      .def("getUpdatedLinkModelsWithGeometry", &JointModelGroup::getUpdatedLinkModelsWithGeometry,
           py::return_value_policy::reference)
      .def("getUpdatedLinkModelsWithGeometryNames", &JointModelGroup::getUpdatedLinkModelsWithGeometryNames)
      .def("getUpdatedLinkModelsWithGeometryNamesSet", &JointModelGroup::getUpdatedLinkModelsWithGeometryNamesSet)
      .def("getUpdatedLinkModelsWithGeometrySet", &JointModelGroup::getUpdatedLinkModelsWithGeometrySet,
           py::return_value_policy::reference)
      .def("getVariableCount", &JointModelGroup::getVariableCount)
      .def("getVariableDefaultPositions", py::overload_cast<const std::string&, std::map<std::string, double>&>(
                                              &JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<double*>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<std::map<std::string, double>&>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableDefaultPositions",
           py::overload_cast<std::vector<double>&>(&JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("getVariableGroupIndex", &JointModelGroup::getVariableGroupIndex)
      .def("getVariableIndexList", &JointModelGroup::getVariableIndexList)
      .def("getVariableNames", &JointModelGroup::getVariableNames)
      .def("hasJointModel", &JointModelGroup::hasJointModel)
      .def("hasLinkModel", &JointModelGroup::hasLinkModel)
      .def("isChain", &JointModelGroup::isChain)
      .def("isContiguousWithinState", &JointModelGroup::isContiguousWithinState)
      .def("isEndEffector", &JointModelGroup::isEndEffector)
      .def("isLinkUpdated", &JointModelGroup::isLinkUpdated)
      .def("isSingleDOFJoints", &JointModelGroup::isSingleDOFJoints)
      .def("isSubgroup", &JointModelGroup::isSubgroup)
      .def("printGroupInfo",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
      .def("setDefaultIKTimeout", &JointModelGroup::setDefaultIKTimeout)
      .def("setEndEffectorName", &JointModelGroup::setEndEffectorName)
      .def("setEndEffectorParent", &JointModelGroup::setEndEffectorParent)
      .def("setRedundantJoints", &JointModelGroup::setRedundantJoints)
      .def("setSubgroupNames", &JointModelGroup::setSubgroupNames)
      .def("__repr__",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
      //
      ;
  py::class_<LinkModel>(m, "LinkModel")
      .def(py::init<std::string>())
      .def("getName", &LinkModel::getName)
      .def("areCollisionOriginTransformsIdentity", &LinkModel::areCollisionOriginTransformsIdentity)
      .def("getCenteredBoundingBoxOffset", &LinkModel::getCenteredBoundingBoxOffset)
      .def("getCollisionOriginTransforms",
           [&](LinkModel const& link) {
             std::vector<Eigen::Matrix4d> matrices;
             auto const& transforms = link.getCollisionOriginTransforms();
             std::transform(transforms.cbegin(), transforms.cend(), std::back_inserter(matrices),
                            [&](Eigen::Isometry3d t) { return t.matrix(); });
             return matrices;
           })
      .def("getFirstCollisionBodyTransformIndex", &LinkModel::getFirstCollisionBodyTransformIndex)
      .def("getJointOriginTransform", [&](LinkModel const& link) { link.getJointOriginTransform().matrix(); })
      .def("getLinkIndex", &LinkModel::getLinkIndex)
      .def("getShapeExtentsAtOrigin", &LinkModel::getShapeExtentsAtOrigin)
      .def("getVisualMeshFilename", &LinkModel::getVisualMeshFilename)
      .def("getVisualMeshOrigin", [&](LinkModel const& link) { return link.getVisualMeshOrigin().matrix(); })
      .def("getVisualMeshScale", &LinkModel::getVisualMeshScale)
      .def("jointOriginTransformIsIdentity", &LinkModel::jointOriginTransformIsIdentity)
      .def("parentJointIsFixed", &LinkModel::parentJointIsFixed)
      //
      ;
  py::class_<JointModel>(m, "JointModel")
      .def("getName", &JointModel::getName)
      .def("addDescendantJointModel", &JointModel::addDescendantJointModel)
      .def("addDescendantLinkModel", &JointModel::addDescendantLinkModel)
      .def("getChildLinkModel", &JointModel::getChildLinkModel)
      .def("getDescendantJointModels", &JointModel::getDescendantJointModels, py::return_value_policy::reference)
      .def("getDescendantLinkModels", &JointModel::getDescendantLinkModels, py::return_value_policy::reference)
      .def("getDistanceFactor", &JointModel::getDistanceFactor)
      .def("getMaximumExtent", py::overload_cast<>(&JointModel::getMaximumExtent, py::const_))
      .def("getMimic", &JointModel::getMimic)
      .def("getMimicFactor", &JointModel::getMimicFactor)
      .def("getMimicOffset", &JointModel::getMimicOffset)
      .def("getMimicRequests", &JointModel::getMimicRequests)
      .def("getNonFixedDescendantJointModels", &JointModel::getNonFixedDescendantJointModels,
           py::return_value_policy::reference)
      .def("getParentLinkModel", &JointModel::getParentLinkModel)
      .def("getStateSpaceDimension", &JointModel::getStateSpaceDimension)
      .def("getTypeName", &JointModel::getTypeName)
      .def("isPassive", &JointModel::isPassive)
      .def("setChildLinkModel", &JointModel::setChildLinkModel, py::keep_alive<1, 2>())
      .def("setDistanceFactor", &JointModel::setDistanceFactor)
      .def("setMimic", &JointModel::setMimic, py::keep_alive<1, 2>())
      .def("setParentLinkModel", &JointModel::setParentLinkModel, py::keep_alive<1, 2>())
      .def("setPassive", &JointModel::setPassive)
      //
      ;
  py::class_<FloatingJointModel, JointModel>(m, "FloatingJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<PlanarJointModel, JointModel>(m, "PlanarJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<PrismaticJointModel, JointModel>(m, "PrismaticJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<FixedJointModel, JointModel>(m, "FixedJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<RevoluteJointModel, JointModel>(m, "RevoluteJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<VariableBounds, std::shared_ptr<VariableBounds>>(m, "VariableBounds")
      .def(py::init<>())
      .def_readwrite("min_position_", &VariableBounds::min_position_)
      .def_readwrite("max_position_", &VariableBounds::max_position_)
      .def_readwrite("position_bounded_", &VariableBounds::position_bounded_)
      .def_readwrite("min_velocity_", &VariableBounds::min_velocity_)
      .def_readwrite("max_velocity_", &VariableBounds::max_velocity_)
      .def_readwrite("velocity_bounded_", &VariableBounds::velocity_bounded_)
      .def_readwrite("min_acceleration_", &VariableBounds::min_acceleration_)
      .def_readwrite("max_acceleration_", &VariableBounds::max_acceleration_)
      .def_readwrite("acceleration_bounded_", &VariableBounds::acceleration_bounded_)
      //
      ;
}
