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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

namespace ompl_interface
{
MOVEIT_CLASS_FORWARD(ConstrainedPlanningStateSpace);

/** \brief State space to be used in combination with OMPL's constrained planning functionality.
 *
 * The state space is called ConstrainedPlanningStateSpace, as in "use this state space for constrained planning".
 * Not to be confused with the `ompl::base::ConstrainedStateSpace`, which is constrained in the sense that it generates
 * states that satisfy the constraints. This state space does not! OMPL's `ompl::base::ConstrainedStateSpace` will wrap
 *around ConstrainedPlanningStateSpace.
 *
 * This class overrides the virtual methods to copy states from OMPL to MoveIt and vice-versa. It is used as state space
 * in the `ModelBasedPlanningContextSpecification` and can handle general `ompl::base::State*` pointers that need to be
 * casted to a `ompl::Base::ConstrainedStateSpace::StateType`, instead of the 'normal'
 * `ompl_interface::ModelBasedStateSpace::StateType`. This casting looks like this:
 *
 *    ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<StateType>()
 *
 * where `getState()` is used to access the underlying state space, which should be an instance of this class.
 **/
class ConstrainedPlanningStateSpace : public ModelBasedStateSpace
{
public:
  static const std::string PARAMETERIZATION_TYPE;

  ConstrainedPlanningStateSpace(const ModelBasedStateSpaceSpecification& spec);

  const std::string& getParameterizationType() const override
  {
    return PARAMETERIZATION_TYPE;
  }

  // override copy operations between OMPL and ROS, because a constrained state has a different internal structure
  double* getValueAddressAtIndex(ompl::base::State* ompl_state, const unsigned int index) const override;
  void copyToRobotState(moveit::core::RobotState& robot_state, const ompl::base::State* ompl_state) const override;
  void copyToOMPLState(ompl::base::State* ompl_state, const moveit::core::RobotState& robot_state) const override;
  void copyJointToOMPLState(ompl::base::State* ompl_state, const moveit::core::RobotState& robot_state,
                            const moveit::core::JointModel* joint_model, int ompl_state_joint_index) const override;
};
}  // namespace ompl_interface
