/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Jeroen De Maeyer  */

/** A state validity checker checks:
 *
 * - Bounds (joint limits).
 * - Collision.
 * - Kinematic path constraints.
 * - Generic user-specified feasibility using the `isStateFeasible` of the planning scene.
 *
 * IMPORTANT: Although the isValid method takes the state as `const ompl::base::State* state`,
 * it uses const_cast to modify the validity of the state with `markInvalid` and `markValid` for caching.
 * **/

#pragma once

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <ompl/base/StateValidityChecker.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class StateValidityChecker
    @brief An interface for a OMPL state validity checker*/
class StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(const ModelBasedPlanningContext* planning_context);

  bool isValid(const ompl::base::State* state) const override
  {
    return isValid(state, verbose_);
  }

  bool isValid(const ompl::base::State* state, double& dist) const override
  {
    return isValid(state, dist, verbose_);
  }

  bool isValid(const ompl::base::State* state, double& dist, ompl::base::State* /*validState*/,
               bool& /*validStateAvailable*/) const override
  {
    return isValid(state, dist, verbose_);
  }

  virtual bool isValid(const ompl::base::State* state, bool verbose) const;
  virtual bool isValid(const ompl::base::State* state, double& dist, bool verbose) const;

  virtual double cost(const ompl::base::State* state) const;
  double clearance(const ompl::base::State* state) const override;

  void setVerbose(bool flag);

protected:
  const ModelBasedPlanningContext* planning_context_;
  std::string group_name_;
  TSStateStorage tss_;
  collision_detection::CollisionRequest collision_request_simple_;
  collision_detection::CollisionRequest collision_request_with_distance_;
  collision_detection::CollisionRequest collision_request_simple_verbose_;
  collision_detection::CollisionRequest collision_request_with_distance_verbose_;

  collision_detection::CollisionRequest collision_request_with_cost_;
  bool verbose_;
};

/** \brief A StateValidityChecker that can handle states of type `ompl::base::ConstraintStateSpace::StateType`.
 *
 * We cannot not just cast the states and pass them to the isValid version of the parent class, because inside the
 * state-validity checker, the line:
 *
 *     planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, state);
 *
 * requires the state type to be of the constrained state space, while:
 *
 *     state->as<ModelBasedStateSpace::StateType>()->isValidityKnown()
 *
 * requires accessing the underlying state by calling `getState()` on the constrained state space state.
 * Therefore this class implements specific versions of the isValid methods.
 *
 * We still check the path constraints, because not all states sampled by the constrained state space
 * satisfy the constraints unfortunately. This is a complicated issue. For more details see:
 * https://github.com/ros-planning/moveit/issues/2092#issuecomment-669911722.
 **/
class ConstrainedPlanningStateValidityChecker : public StateValidityChecker
{
public:
  using StateValidityChecker::isValid;

public:
  ConstrainedPlanningStateValidityChecker(const ModelBasedPlanningContext* planning_context)
    : StateValidityChecker(planning_context)
  {
  }

  /** \brief Check validity for states of type ompl::base::ConstrainedStateSpace
   *
   * This state type is special in that it "wraps" around a normal state,
   * which can be accessed by the getState() method. In this class we assume that this state,
   * is of type `ompl_interface::ConstrainedPlanningStateSpace`, which inherits from
   * `ompl_interface::ModelBasedStateSpace`.
   *
   * (For the actual implementation of this, look at the ompl::base::WrapperStateSpace.)
   *
   * Code sample that can be used to check all the assumptions:
   *
   *    #include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>
   *    #include <ompl/base/ConstrainedSpaceInformation.h>
   *
   *    // the code below should be pasted at the top of the isValid method
   *    auto css = dynamic_cast<const ompl::base::ConstrainedStateSpace::StateType*>(wrapped_state);
   *    assert(css != nullptr);
   *    auto cpss = dynamic_cast<const ConstrainedPlanningStateSpace*>(planning_context_->getOMPLStateSpace().get());
   *    assert(cpss != nullptr);
   *    auto cssi = dynamic_cast<const ompl::base::ConstrainedSpaceInformation*>(si_);
   *    assert(cssi != nullptr);
   *
   **/
  bool isValid(const ompl::base::State* wrapped_state, bool verbose) const override;
  bool isValid(const ompl::base::State* wrapped_state, double& dist, bool verbose) const override;
};
}  // namespace ompl_interface
