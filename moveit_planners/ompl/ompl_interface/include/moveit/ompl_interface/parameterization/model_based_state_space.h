/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#pragma once

#include <ompl/base/StateSpace.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <atomic>
#include <memory>

namespace ompl_interface
{
typedef std::function<bool(const ompl::base::State* from, const ompl::base::State* to, const double t,
                           ompl::base::State* state)>
    InterpolationFunction;
typedef std::function<double(const ompl::base::State* state1, const ompl::base::State* state2)> DistanceFunction;

struct ModelBasedStateSpaceSpecification
{
  ModelBasedStateSpaceSpecification(const moveit::core::RobotModelConstPtr& robot_model,
                                    const moveit::core::JointModelGroup* jmg)
    : robot_model_(robot_model), joint_model_group_(jmg)
  {
  }

  ModelBasedStateSpaceSpecification(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name)
    : robot_model_(robot_model), joint_model_group_(robot_model_->getJointModelGroup(group_name))
  {
    if (!joint_model_group_)
      throw std::runtime_error("Group '" + group_name + "'  was not found");
  }

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::JointBoundsVector joint_bounds_;
};

OMPL_CLASS_FORWARD(ModelBasedStateSpace);

class ModelBasedStateSpace : public ompl::base::StateSpace
{
public:
  class StateType : public ompl::base::State
  {
  public:
    enum
    {
      VALIDITY_KNOWN = 1,
      GOAL_DISTANCE_KNOWN = 2,
      VALIDITY_TRUE = 4,
      IS_START_STATE = 8,
      IS_GOAL_STATE = 16
    };

    /** \brief Thread-safe container for cached values.
     *
     * OMPL requires all \c const qualified member functions to be thread-safe.
     * Some classes like ompl_interface::StateValidityChecker use this State type
     * to cache some results, using \c const_cast . This container is a \c mutable
     * atomic member of the state, which allows caching results without UB and data races.
     * It should only be modified with the help of modifyCache(),
     * which ensures that the state itself is consistent to other threads.
     * To process the data in the cache, always get a copy of the cache using getCache().
     * The 16 byte alignment is needed for Compare-and-Exchange on x64.
     */
    struct alignas(16) AtomicCache
    {
      int tag;
      int flags;
      double distance;

      bool isValidityKnown() const
      {
        return flags & VALIDITY_KNOWN;
      }

      bool isMarkedValid() const
      {
        return flags & VALIDITY_TRUE;
      }

      bool isGoalDistanceKnown() const
      {
        return flags & GOAL_DISTANCE_KNOWN;
      }

      bool isStartState() const
      {
        return flags & IS_START_STATE;
      }

      bool isGoalState() const
      {
        return flags & IS_GOAL_STATE;
      }

      bool isInputState() const
      {
        return flags & (IS_START_STATE | IS_GOAL_STATE);
      }
    };
    static_assert(sizeof(AtomicCache) == 16, "Padding not well supported for CAS");

    StateType() : ompl::base::State(), atomic_cache(AtomicCache{ -1, 0, 0.0 }), values_(nullptr)
    {
    }

    /** \brief Mark state as valid and set the known distance.
     *
     * This function is \c const qualified to make it mutable from
     * within OMPL interfaces which pass a <tt>const State *</tt>.
     * \param d Distance
     */
    void markValid(double d) const
    {
      modifyCache([d](AtomicCache& desired) {
        desired.distance = d;
        desired.flags |= (VALIDITY_KNOWN | VALIDITY_TRUE | GOAL_DISTANCE_KNOWN);
      });
    }

    /** \brief Mark state as valid.
     *
     * This function is \c const qualified to make it mutable from
     * within OMPL interfaces which pass a <tt>const State *</tt>.
     */
    void markValid() const
    {
      setFlag(VALIDITY_KNOWN | VALIDITY_TRUE);
    }

    /** \brief Mark state as invalid and set the known distance.
     *
     * This function is \c const qualified to make it mutable from
     * within OMPL interfaces which pass a <tt>const State *</tt>.
     * \param d Distance
     */
    void markInvalid(double d) const
    {
      modifyCache([d](AtomicCache& desired) {
        desired.distance = d;
        desired.flags &= ~VALIDITY_TRUE;
        desired.flags |= (VALIDITY_KNOWN | GOAL_DISTANCE_KNOWN);
      });
    }

    /** \brief Mark state as invalid.
     *
     * This function is \c const qualified to make it mutable from
     * within OMPL interfaces which pass a <tt>const State *</tt>.
     */
    void markInvalid() const
    {
      modifyCache([](AtomicCache& desired) {
        desired.flags &= ~VALIDITY_TRUE;
        desired.flags |= VALIDITY_KNOWN;
      });
    }

    void clearKnownInformation()
    {
      modifyCache([](AtomicCache& desired) { desired.flags = 0; });
    }

    void markStartState()
    {
      setFlag(IS_START_STATE);
    }

    void markGoalState()
    {
      setFlag(IS_GOAL_STATE);
    }

    int flags() const
    {
      return atomic_cache.load().flags;
    }
    void setFlag(int flag) const
    {
      modifyCache([flag](AtomicCache& desired) { desired.flags |= flag; });
    }
    void clearFlag(int flag) const
    {
      modifyCache([flag](AtomicCache& desired) { desired.flags &= ~flag; });
    }
    int tag() const
    {
      return atomic_cache.load().tag;
    }
    void setTag(int tag)
    {
      modifyCache([tag](AtomicCache& desired) { desired.tag = tag; });
    }
    /** \brief Get a copy of the cached data.
     *
     * Use this getter to get a consistent dataset
     * (e.g. valid distance with GOAL_DISTANCE_KNOWN flag set)
     * for further processing. Doing otherwise might result in data races.
     * \return A valid and consistent dataset (flags, tag, distance).
     */
    AtomicCache getCache() const
    {
      return atomic_cache.load();
    }
    void setCache(const AtomicCache& cache)
    {
      atomic_cache.store(cache);
    }

    double* values()
    {
      return values_.get();
    }

    const double* values() const
    {
      return values_.get();
    }

    void setValues(std::unique_ptr<double[]> values)
    {
      values_ = std::move(values);
    }

  protected:
    mutable std::atomic<AtomicCache> atomic_cache;
    /**
     * \brief Helper function to modify the cache in a compare-and-swap loop.
     *
     * The Lambda will be called at least one time.
     * It takes a reference to a copy of the cache and should update it
     * (e.g. set a bit in the flags). The copy is then written back,
     * but only if the original cache was left untouched.
     *
     * \param func Lambda which takes exactly one parameter, an AtomicCache Reference.
     *
     */
    template <class Func>
    void modifyCache(Func&& func) const
    {
      AtomicCache desired, expected = atomic_cache.load();
      do
      {
        desired = expected;
        func(desired);
      } while (!atomic_cache.compare_exchange_weak(expected, desired));
    }

    std::unique_ptr<double[]> values_;
  };

  ModelBasedStateSpace(ModelBasedStateSpaceSpecification spec);
  ~ModelBasedStateSpace() override;

  void setInterpolationFunction(const InterpolationFunction& fun)
  {
    interpolation_function_ = fun;
  }

  void setDistanceFunction(const DistanceFunction& fun)
  {
    distance_function_ = fun;
  }

  ompl::base::State* allocState() const override;
  void freeState(ompl::base::State* state) const override;
  unsigned int getDimension() const override;
  void enforceBounds(ompl::base::State* state) const override;
  bool satisfiesBounds(const ompl::base::State* state) const override;

  void copyState(ompl::base::State* destination, const ompl::base::State* source) const override;
  void interpolate(const ompl::base::State* from, const ompl::base::State* to, const double t,
                   ompl::base::State* state) const override;
  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;
  bool equalStates(const ompl::base::State* state1, const ompl::base::State* state2) const override;
  double getMaximumExtent() const override;
  double getMeasure() const override;

  unsigned int getSerializationLength() const override;
  void serialize(void* serialization, const ompl::base::State* state) const override;
  void deserialize(ompl::base::State* state, const void* serialization) const override;
  double* getValueAddressAtIndex(ompl::base::State* state, const unsigned int index) const override;

  ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  virtual const std::string& getParameterizationType() const = 0;

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return spec_.robot_model_;
  }

  const moveit::core::JointModelGroup* getJointModelGroup() const
  {
    return spec_.joint_model_group_;
  }

  const std::string& getJointModelGroupName() const
  {
    return getJointModelGroup()->getName();
  }

  const ModelBasedStateSpaceSpecification& getSpecification() const
  {
    return spec_;
  }

  void printState(const ompl::base::State* state, std::ostream& out) const override;
  void printSettings(std::ostream& out) const override;

  /// Set the planning volume for the possible SE2 and/or SE3 components of the state space
  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);

  const moveit::core::JointBoundsVector& getJointsBounds() const
  {
    return spec_.joint_bounds_;
  }

  /// Copy the data from an OMPL state to a set of joint states.
  // The joint states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToRobotState(moveit::core::RobotState& rstate, const ompl::base::State* state) const;

  /// Copy the data from a set of joint states to an OMPL state.
  //  The joint states \b must be specified in the same order as the joint models in the constructor
  virtual void copyToOMPLState(ompl::base::State* state, const moveit::core::RobotState& rstate) const;

  /**
   * \brief Copy a single joint's values (which might have multiple variables) from a MoveIt robot_state to an OMPL
   * state.
   * \param state - output OMPL state with single joint modified
   * \param robot_state - input MoveIt state to get the joint value from
   * \param joint_model - the joint to copy values of
   * \param ompl_state_joint_index - the index of the joint in the ompl state (passed in for efficiency, you should
   * cache this index)
   *        e.g. ompl_state_joint_index = joint_model_group_->getVariableGroupIndex("virtual_joint");
   */
  virtual void copyJointToOMPLState(ompl::base::State* state, const moveit::core::RobotState& robot_state,
                                    const moveit::core::JointModel* joint_model, int ompl_state_joint_index) const;

  double getTagSnapToSegment() const;
  void setTagSnapToSegment(double snap);

protected:
  ModelBasedStateSpaceSpecification spec_;
  std::vector<moveit::core::JointModel::Bounds> joint_bounds_storage_;
  std::vector<const moveit::core::JointModel*> joint_model_vector_;
  unsigned int variable_count_;
  size_t state_values_size_;

  InterpolationFunction interpolation_function_;
  DistanceFunction distance_function_;

  double tag_snap_to_segment_;
  double tag_snap_to_segment_complement_;
};
}  // namespace ompl_interface
