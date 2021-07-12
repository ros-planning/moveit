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

#pragma once

#include <moveit/ompl_interface/parameterization/model_based_state_space_factory.h>

namespace ompl_interface
{
class ConstrainedPlanningStateSpaceFactory : public ModelBasedStateSpaceFactory
{
public:
  ConstrainedPlanningStateSpaceFactory();

  /** \brief Return a priority that this planner should be used for this specific planning problem.
   *
   * This state space factory is currently only used if `use_ompl_constrained_state_space` was set to `true` in
   * ompl_planning.yaml. In that case it is the only factory to choose from, so the priority does not matter.
   * It returns a low priority so it will never be choosen when others are available.
   * (The second lowest priority is -1 in the PoseModelStateSpaceFactory.)
   *
   * For more details on this state space selection process, see:
   * https://github.com/JeroenDM/moveit/pull/2
   * **/
  int canRepresentProblem(const std::string& group, const moveit_msgs::MotionPlanRequest& req,
                          const moveit::core::RobotModelConstPtr& robot_model) const override;

protected:
  ModelBasedStateSpacePtr allocStateSpace(const ModelBasedStateSpaceSpecification& space_spec) const override;
};
}  // namespace ompl_interface
