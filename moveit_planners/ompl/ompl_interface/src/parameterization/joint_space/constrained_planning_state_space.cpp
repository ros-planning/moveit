/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer */

#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

namespace ompl_interface
{
const std::string ConstrainedPlanningStateSpace::PARAMETERIZATION_TYPE = "ConstrainedPlanningJointModel";

ConstrainedPlanningStateSpace::ConstrainedPlanningStateSpace(const ModelBasedStateSpaceSpecification& spec)
  : ModelBasedStateSpace(spec)
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);
}

double* ConstrainedPlanningStateSpace::getValueAddressAtIndex(ompl::base::State* ompl_state,
                                                              const unsigned int index) const
{
  assert(ompl_state != nullptr);
  if (index >= variable_count_)
  {
    return nullptr;
  }

  // Developer tip: replace this with a dynamic_cast for debugging
  return ompl_state->as<ompl_interface::ConstrainedPlanningStateSpace::StateType>()->values + index;
}

void ConstrainedPlanningStateSpace::copyToRobotState(moveit::core::RobotState& robot_state,
                                                     const ompl::base::State* ompl_state) const
{
  assert(ompl_state != nullptr);
  robot_state.setJointGroupPositions(
      spec_.joint_model_group_,
      ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->values);
  robot_state.update();
}

void ConstrainedPlanningStateSpace::copyToOMPLState(ompl::base::State* ompl_state,
                                                    const moveit::core::RobotState& robot_state) const
{
  assert(ompl_state != nullptr);
  robot_state.copyJointGroupPositions(
      spec_.joint_model_group_,
      ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->values);

  // clear any cached info (such as validity known or not)
  ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->clearKnownInformation();
}

void ConstrainedPlanningStateSpace::copyJointToOMPLState(ompl::base::State* ompl_state,
                                                         const moveit::core::RobotState& robot_state,
                                                         const moveit::core::JointModel* joint_model,
                                                         int ompl_state_joint_index) const
{
  assert(ompl_state != nullptr);
  assert(joint_model != nullptr);
  memcpy(getValueAddressAtIndex(ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState(),
                                ompl_state_joint_index),
         robot_state.getVariablePositions() + joint_model->getFirstVariableIndex(),
         joint_model->getVariableCount() * sizeof(double));

  // clear any cached info (such as validity known or not)
  ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()->clearKnownInformation();
}
}  // namespace ompl_interface
