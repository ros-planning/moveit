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
/* Mostly copied from Ioan Sucan's code */

#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>
#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space_factory.h>

ompl_interface::ConstrainedPlanningStateSpaceFactory::ConstrainedPlanningStateSpaceFactory()
  : ModelBasedStateSpaceFactory()
{
  type_ = ConstrainedPlanningStateSpace::PARAMETERIZATION_TYPE;
}

int ompl_interface::ConstrainedPlanningStateSpaceFactory::canRepresentProblem(
    const std::string& /*group*/, const moveit_msgs::MotionPlanRequest& /*req*/,
    const moveit::core::RobotModelConstPtr& /*robot_model*/) const
{
  // Return the lowest priority currently existing in the ompl interface.
  // This state space will only be selected if it is the only option to choose from.
  // See header file for more info.
  return -2;
}

ompl_interface::ModelBasedStateSpacePtr ompl_interface::ConstrainedPlanningStateSpaceFactory::allocStateSpace(
    const ModelBasedStateSpaceSpecification& space_spec) const
{
  return std::make_shared<ConstrainedPlanningStateSpace>(space_spec);
}
