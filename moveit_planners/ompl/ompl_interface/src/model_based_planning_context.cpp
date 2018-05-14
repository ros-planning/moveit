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

#include <boost/algorithm/string/trim.hpp>
#include <pluginlib/class_list_macros.h>

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/goal_union.h>
#include <moveit/ompl_interface/detail/projection_evaluators.h>
#include <moveit/ompl_interface/detail/constraints_library.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/PDF.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/sbl/SBL.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

ompl_interface::ModelBasedPlanningContext::ModelBasedPlanningContext()
  : OMPLPlanningContext(), ptc_(NULL), last_plan_time_(0.0), last_simplify_time_(0.0), use_state_validity_cache_(true)
{
}

void ompl_interface::ModelBasedPlanningContext::initialize(const OMPLPlanningContextSpecification& spec)
{
  OMPLPlanningContext::initialize(spec);

  if (!complete_initial_robot_state_)
    complete_initial_robot_state_.reset(new robot_state::RobotState(spec.robot_model_));

  ompl_state_space_ = getStateSpace();

  ompl_simple_setup_ = std::make_shared<og::SimpleSetup>(ompl_state_space_);
  ompl_benchmark_ = std::make_shared<ot::Benchmark>(*ompl_simple_setup_);
  ompl_parallel_plan_ = std::make_shared<ot::ParallelPlan>(ompl_simple_setup_->getProblemDefinition());
  constraints_library_ = std::make_shared<ConstraintsLibrary>(this);

  registerDefaultPlanners();
}

void ompl_interface::ModelBasedPlanningContext::configure(const ros::NodeHandle& nh,
                                                          const OMPLDynamicReconfigureConfig& config)
{
  OMPLPlanningContext::configure(nh, config);

  loadConstraintApproximations(nh);

  if (!config.use_constraints_approximations)
    setConstraintsApproximations(ConstraintsLibraryPtr());

  complete_initial_robot_state_->update();
  ompl_simple_setup_->getStateSpace()->computeSignature(space_signature_);
  ompl_simple_setup_->getStateSpace()->setStateSamplerAllocator(
      boost::bind(&ModelBasedPlanningContext::allocPathConstrainedSampler, this, _1));

  // convert the input state to the corresponding OMPL state
  ob::ScopedState<> ompl_start_state(ompl_state_space_);
  ompl_state_space_->copyToOMPLState(ompl_start_state.get(), getCompleteInitialRobotState());
  ompl_simple_setup_->setStartState(ompl_start_state);
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(this)));

  if (path_constraints_ && constraints_library_)
  {
    const ConstraintApproximationPtr& ca = constraints_library_->getConstraintApproximation(path_constraints_msg_);
    if (ca)
    {
      getOMPLStateSpace()->setInterpolationFunction(ca->getInterpolationFunction());
      ROS_INFO_NAMED("model_based_planning_context", "Using precomputed interpolation states");
    }
  }

  useConfig();
  if (ompl_simple_setup_->getGoal())
    ompl_simple_setup_->setup();
}

namespace
{
using namespace ompl_interface;
template <typename T>
static ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr& si, const std::string& new_name,
                                      const OMPLPlanningContextSpecification& spec)
{
  ob::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_.config, true);
  planner->setup();
  return planner;
}
}

ompl_interface::ConfiguredPlannerAllocator
ompl_interface::ModelBasedPlanningContext::plannerSelector(const std::string& planner) const
{
  std::map<std::string, ConfiguredPlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

void ompl_interface::ModelBasedPlanningContext::registerDefaultPlanners()
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::InformedRRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::SORRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTXstatic", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTsharp", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::BiTRRT", boost::bind(&allocatePlanner<og::BiTRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBTRRT", boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyLBTRRT", boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2, _3));

  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
  registerPlannerAllocator("geometric::BiEST", boost::bind(&allocatePlanner<og::BiEST>, _1, _2, _3));
  registerPlannerAllocator("geometric::ProjEST", boost::bind(&allocatePlanner<og::ProjEST>, _1, _2, _3));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));

  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));

  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyPRM", boost::bind(&allocatePlanner<og::LazyPRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyPRMstar", boost::bind(&allocatePlanner<og::LazyPRMstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::SPARS", boost::bind(&allocatePlanner<og::SPARS>, _1, _2, _3));
  registerPlannerAllocator("geometric::SPARStwo", boost::bind(&allocatePlanner<og::SPARStwo>, _1, _2, _3));

  registerPlannerAllocator("geometric::FMT", boost::bind(&allocatePlanner<og::FMT>, _1, _2, _3));
  registerPlannerAllocator("geometric::BFMT", boost::bind(&allocatePlanner<og::BFMT>, _1, _2, _3));
  registerPlannerAllocator("geometric::BITstar", boost::bind(&allocatePlanner<og::BITstar>, _1, _2, _3));

  registerPlannerAllocator("geometric::PDST", boost::bind(&allocatePlanner<og::PDST>, _1, _2, _3));
  registerPlannerAllocator("geometric::STRIDE", boost::bind(&allocatePlanner<og::STRIDE>, _1, _2, _3));
}

void ompl_interface::ModelBasedPlanningContext::setProjectionEvaluator(const std::string& peval)
{
  if (!ompl_state_space_)
  {
    ROS_ERROR_NAMED("model_based_planning_context", "No state space is configured yet");
    return;
  }
  ob::ProjectionEvaluatorPtr pe = getProjectionEvaluator(peval);
  if (pe)
    ompl_state_space_->registerDefaultProjection(pe);
}

ob::ProjectionEvaluatorPtr
ompl_interface::ModelBasedPlanningContext::getProjectionEvaluator(const std::string& peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getRobotModel()->hasLinkModel(link_name))
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR_NAMED("model_based_planning_context",
                      "Attempted to set projection evaluator with respect to position of link '%s', "
                      "but that link is not known to the kinematic model.",
                      link_name.c_str());
  }
  else if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string joints = peval.substr(7, peval.length() - 8);
    boost::replace_all(joints, ",", " ");
    std::vector<unsigned int> j;
    std::stringstream ss(joints);
    while (ss.good() && !ss.eof())
    {
      std::string v;
      ss >> v >> std::ws;
      if (getJointModelGroup()->hasJointModel(v))
      {
        unsigned int vc = getJointModelGroup()->getJointModel(v)->getVariableCount();
        if (vc > 0)
        {
          int idx = getJointModelGroup()->getVariableGroupIndex(v);
          for (int q = 0; q < vc; ++q)
            j.push_back(idx + q);
        }
        else
          ROS_WARN_NAMED("model_based_planning_context", "%s: Ignoring joint '%s' in projection since it has 0 DOF",
                         name_.c_str(), v.c_str());
      }
      else
        ROS_ERROR_NAMED("model_based_planning_context",
                        "%s: Attempted to set projection evaluator with respect to value of joint "
                        "'%s', but that joint is not known to the group '%s'.",
                        name_.c_str(), v.c_str(), getGroupName().c_str());
    }
    if (j.empty())
      ROS_ERROR_NAMED("model_based_planning_context", "%s: No valid joints specified for joint projection",
                      name_.c_str());
    else
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
  }
  else
    ROS_ERROR_NAMED("model_based_planning_context",
                    "Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
  return ob::ProjectionEvaluatorPtr();
}

ob::StateSamplerPtr
ompl_interface::ModelBasedPlanningContext::allocPathConstrainedSampler(const ob::StateSpace* ss) const
{
  if (ompl_state_space_.get() != ss)
  {
    ROS_ERROR_NAMED("model_based_planning_context",
                    "%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
    return ob::StateSamplerPtr();
  }

  ROS_DEBUG_NAMED("model_based_planning_context",
                  "%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());

  if (path_constraints_)
  {
    if (constraints_library_)
    {
      const ConstraintApproximationPtr& ca = constraints_library_->getConstraintApproximation(path_constraints_msg_);
      if (ca)
      {
        ob::StateSamplerAllocator c_ssa = ca->getStateSamplerAllocator(path_constraints_msg_);
        if (c_ssa)
        {
          ob::StateSamplerPtr res = c_ssa(ss);
          if (res)
          {
            ROS_INFO_NAMED("model_based_planning_context",
                           "%s: Using precomputed state sampler (approximated constraint space) for constraint '%s'",
                           name_.c_str(), path_constraints_msg_.name.c_str());
            return res;
          }
        }
      }
    }

    constraint_samplers::ConstraintSamplerPtr cs;
    if (spec_.csm_)
      cs = spec_.csm_->selectSampler(getPlanningScene(), getGroupName(), path_constraints_->getAllConstraints());

    if (cs)
    {
      ROS_INFO_NAMED("model_based_planning_context", "%s: Allocating specialized state sampler for state space",
                     name_.c_str());
      return ob::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG_NAMED("model_based_planning_context", "%s: Allocating default state sampler for state space",
                  name_.c_str());
  return ss->allocDefaultStateSampler();
}

void ompl_interface::ModelBasedPlanningContext::useConfig()
{
  const std::map<std::string, std::string>& config = getSpecificationConfig();
  if (config.empty())
    return;
  std::map<std::string, std::string> cfg = config;

  // set the distance between waypoints when interpolating and collision checking.
  std::map<std::string, std::string>::iterator it = cfg.find("longest_valid_segment_fraction");
  // If one of the two variables is set.
  if (it != cfg.end() || max_solution_segment_length_ != 0.0)
  {
    // clang-format off
    double longest_valid_segment_fraction_config = (it != cfg.end())
      ? boost::lexical_cast<double>(it->second)  // value from config file if there
      : 0.01;  // default value in OMPL.
    double longest_valid_segment_fraction_final = longest_valid_segment_fraction_config;
    if (max_solution_segment_length_ > 0.0)
    {
      // If this parameter is specified too, take the most conservative of the two variables,
      // i.e. the one that uses the shorter segment length.
      longest_valid_segment_fraction_final = std::min(
          longest_valid_segment_fraction_config,
          max_solution_segment_length_ / ompl_state_space_->getMaximumExtent()
      );
    }
    // clang-format on
    cfg["longest_valid_segment_fraction"] = boost::lexical_cast<std::string>(longest_valid_segment_fraction_final);
  }

  // set the projection evaluator
  it = cfg.find("projection_evaluator");
  if (it != cfg.end())
  {
    setProjectionEvaluator(boost::trim_copy(it->second));
    cfg.erase(it);
  }

  if (cfg.empty())
    return;

  std::string optimizer;
  ob::OptimizationObjectivePtr objective;
  it = cfg.find("optimization_objective");
  if (it == cfg.end())
  {
    optimizer = "PathLengthOptimizationObjective";
    ROS_DEBUG_NAMED("model_based_planning_context", "No optimization objective specified, defaulting to %s",
                    optimizer.c_str());
  }
  else
  {
    optimizer = it->second;
    cfg.erase(it);
  }

  if (optimizer == "PathLengthOptimizationObjective")
  {
    objective.reset(new ob::PathLengthOptimizationObjective(ompl_simple_setup_->getSpaceInformation()));
  }
  else if (optimizer == "MinimaxObjective")
  {
    objective.reset(new ob::MinimaxObjective(ompl_simple_setup_->getSpaceInformation()));
  }
  else if (optimizer == "StateCostIntegralObjective")
  {
    objective.reset(new ob::StateCostIntegralObjective(ompl_simple_setup_->getSpaceInformation()));
  }
  else if (optimizer == "MechanicalWorkOptimizationObjective")
  {
    objective.reset(new ob::MechanicalWorkOptimizationObjective(ompl_simple_setup_->getSpaceInformation()));
  }
  else if (optimizer == "MaximizeMinClearanceObjective")
  {
    objective.reset(new ob::MaximizeMinClearanceObjective(ompl_simple_setup_->getSpaceInformation()));
  }
  else
  {
    objective.reset(new ob::PathLengthOptimizationObjective(ompl_simple_setup_->getSpaceInformation()));
  }

  ompl_simple_setup_->setOptimizationObjective(objective);

  // remove the 'type' parameter; the rest are parameters for the planner itself
  it = cfg.find("type");
  if (it == cfg.end())
  {
    if (name_ != getGroupName())
      ROS_WARN_NAMED("model_based_planning_context", "%s: Attribute 'type' not specified in planner configuration",
                     name_.c_str());
  }
  else
  {
    std::string type = it->second;
    cfg.erase(it);
    ompl_simple_setup_->setPlannerAllocator(
        boost::bind(plannerSelector(type), _1, name_ != getGroupName() ? name_ : "", spec_));
    ROS_INFO_NAMED("model_based_planning_context",
                   "Planner configuration '%s' will use planner '%s'. "
                   "Additional configuration parameters will be set when the planner is constructed.",
                   name_.c_str(), type.c_str());
  }

  // call the setParams() after setup(), so we know what the params are
  ompl_simple_setup_->getSpaceInformation()->setup();
  ompl_simple_setup_->getSpaceInformation()->params().setParams(cfg, true);
  // call setup() again for possibly new param values
  ompl_simple_setup_->getSpaceInformation()->setup();
}

void ompl_interface::ModelBasedPlanningContext::setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams)
{
  if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
      wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
      wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
    ROS_WARN_NAMED("model_based_planning_context", "It looks like the planning volume was not specified.");

  ROS_DEBUG_NAMED("model_based_planning_context",
                  "%s: Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = "
                  "[%f, %f], z = [%f, %f]",
                  name_.c_str(), wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y,
                  wparams.min_corner.z, wparams.max_corner.z);

  ompl_state_space_->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y,
                                       wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);
}

void ompl_interface::ModelBasedPlanningContext::simplifySolution(double timeout)
{
  ompl_simple_setup_->simplifySolution(timeout);
  last_simplify_time_ = ompl_simple_setup_->getLastSimplificationTime();
}

void ompl_interface::ModelBasedPlanningContext::interpolateSolution()
{
  if (ompl_simple_setup_->haveSolutionPath())
  {
    og::PathGeometric& pg = ompl_simple_setup_->getSolutionPath();

    // Find the number of states that will be in the interpolated solution.
    // This is what interpolate() does internally.
    int eventual_states = 1;
    std::vector<ob::State*> states = pg.getStates();
    for (size_t i = 0; i < states.size() - 1; i++)
    {
      eventual_states += ompl_simple_setup_->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    }

    if (eventual_states < minimum_waypoint_count_)
    {
      // If that's not enough states, use the minimum amount instead.
      pg.interpolate(minimum_waypoint_count_);
    }
    else
    {
      // Interpolate the path to have as the exact states that are checked when validating motions.
      pg.interpolate();
    }
  }
}

void ompl_interface::ModelBasedPlanningContext::convertPath(const ompl::geometric::PathGeometric& pg,
                                                            robot_trajectory::RobotTrajectory& traj) const
{
  robot_state::RobotState& ks = *complete_initial_robot_state_;
  for (std::size_t i = 0; i < pg.getStateCount(); ++i)
  {
    ompl_state_space_->copyToRobotState(ks, pg.getState(i));
    traj.addSuffixWayPoint(ks, 0.0);
  }
}

bool ompl_interface::ModelBasedPlanningContext::getSolutionPath(robot_trajectory::RobotTrajectory& traj) const
{
  traj.clear();
  if (!ompl_simple_setup_->haveSolutionPath())
    return false;
  convertPath(ompl_simple_setup_->getSolutionPath(), traj);
  return true;
}

void ompl_interface::ModelBasedPlanningContext::setVerboseStateValidityChecks(bool flag)
{
  if (ompl_simple_setup_->getStateValidityChecker())
    static_cast<StateValidityChecker*>(ompl_simple_setup_->getStateValidityChecker().get())->setVerbose(flag);
}

ob::GoalPtr ompl_interface::ModelBasedPlanningContext::constructGoal()
{
  std::vector<ob::GoalPtr> goals;
  for (std::size_t i = 0; i < goal_constraints_.size(); ++i)
  {
    constraint_samplers::ConstraintSamplerPtr cs;
    if (spec_.csm_)
      cs = spec_.csm_->selectSampler(getPlanningScene(), getGroupName(), goal_constraints_[i]->getAllConstraints());
    if (cs)
    {
      ob::GoalPtr g = ob::GoalPtr(new ConstrainedGoalSampler(this, goal_constraints_[i], cs));
      goals.push_back(g);
    }
  }

  if (!goals.empty())
    return goals.size() == 1 ? goals[0] : ob::GoalPtr(new GoalSampleableRegionMux(goals));
  else
    ROS_ERROR_NAMED("model_based_planning_context", "Unable to construct goal representation");

  return ob::GoalPtr();
}

void ompl_interface::ModelBasedPlanningContext::clear()
{
  ompl_simple_setup_->clear();
  ompl_simple_setup_->clearStartStates();
  ompl_simple_setup_->setGoal(ob::GoalPtr());
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr());
  path_constraints_.reset();
  goal_constraints_.clear();
  getOMPLStateSpace()->setInterpolationFunction(InterpolationFunction());
}

bool ompl_interface::ModelBasedPlanningContext::setPathConstraints(const moveit_msgs::Constraints& path_constraints,
                                                                   moveit_msgs::MoveItErrorCodes* error)
{
  // ******************* set the path constraints to use
  path_constraints_.reset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
  path_constraints_->add(path_constraints, getPlanningScene()->getTransforms());
  path_constraints_msg_ = path_constraints;

  return true;
}

bool ompl_interface::ModelBasedPlanningContext::setGoalConstraints(
    const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraints,
    moveit_msgs::MoveItErrorCodes* error)
{
  // ******************* check if the input is correct
  goal_constraints_.clear();
  for (std::size_t i = 0; i < goal_constraints.size(); ++i)
  {
    moveit_msgs::Constraints constr = kinematic_constraints::mergeConstraints(goal_constraints[i], path_constraints);
    kinematic_constraints::KinematicConstraintSetPtr kset(
        new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    kset->add(constr, getPlanningScene()->getTransforms());
    if (!kset->empty())
      goal_constraints_.push_back(kset);
  }

  if (goal_constraints_.empty())
  {
    ROS_WARN_NAMED("model_based_planning_context", "%s: No goal constraints specified. There is no problem to solve.",
                   name_.c_str());
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  ob::GoalPtr goal = constructGoal();
  ompl_simple_setup_->setGoal(goal);
  if (goal)
    return true;
  else
    return false;
}

bool ompl_interface::ModelBasedPlanningContext::benchmark(double timeout, unsigned int count,
                                                          const std::string& filename)
{
  ompl_benchmark_->clearPlanners();
  ompl_simple_setup_->setup();
  ompl_benchmark_->addPlanner(ompl_simple_setup_->getPlanner());
  ompl_benchmark_->setExperimentName(getRobotModel()->getName() + "_" + getGroupName() + "_" +
                                     getPlanningScene()->getName() + "_" + name_);

  ot::Benchmark::Request req;
  req.maxTime = timeout;
  req.runCount = count;
  req.displayProgress = true;
  req.saveConsoleOutput = false;
  ompl_benchmark_->benchmark(req);
  return filename.empty() ? ompl_benchmark_->saveResultsToFile() : ompl_benchmark_->saveResultsToFile(filename.c_str());
}

void ompl_interface::ModelBasedPlanningContext::startSampling()
{
  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->startSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(ompl_simple_setup_->getGoal().get())->startSampling();
}

void ompl_interface::ModelBasedPlanningContext::stopSampling()
{
  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->stopSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(ompl_simple_setup_->getGoal().get())->stopSampling();
}

void ompl_interface::ModelBasedPlanningContext::preSolve()
{
  // clear previously computed solutions
  ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
  const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
  if (planner)
    planner->clear();
  startSampling();
  ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

void ompl_interface::ModelBasedPlanningContext::postSolve()
{
  stopSampling();
  int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
  int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
  ROS_DEBUG_NAMED("model_based_planning_context", "There were %d valid motions and %d invalid motions.", v, iv);

  if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
    ROS_WARN_NAMED("model_based_planning_context", "Computed solution is approximate");
}

bool ompl_interface::ModelBasedPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  if (solve(request_.allowed_planning_time, request_.num_planning_attempts))
  {
    double ptime = getLastPlanTime();
    if (simplify_solutions_)
    {
      simplifySolution(request_.allowed_planning_time - ptime);
      ptime += getLastSimplifyTime();
    }
    interpolateSolution();

    // fill the response
    ROS_DEBUG_NAMED("model_based_planning_context", "%s: Returning successful solution with %lu states",
                    getName().c_str(), getOMPLSimpleSetup()->getSolutionPath().getStateCount());

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_);
    res.planning_time_ = ptime;
    return true;
  }
  else
  {
    ROS_INFO_NAMED("model_based_planning_context", "Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool ompl_interface::ModelBasedPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  if (solve(request_.allowed_planning_time, request_.num_planning_attempts))
  {
    res.trajectory_.reserve(3);

    // add info about planned solution
    double ptime = getLastPlanTime();
    res.processing_time_.push_back(ptime);
    res.description_.push_back("plan");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_.back());

    // simplify solution if time remains
    if (simplify_solutions_)
    {
      simplifySolution(request_.allowed_planning_time - ptime);
      res.processing_time_.push_back(getLastSimplifyTime());
      res.description_.push_back("simplify");
      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
      getSolutionPath(*res.trajectory_.back());
    }

    ompl::time::point start_interpolate = ompl::time::now();
    interpolateSolution();
    res.processing_time_.push_back(ompl::time::seconds(ompl::time::now() - start_interpolate));
    res.description_.push_back("interpolate");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_.back());

    // fill the response
    ROS_DEBUG_NAMED("model_based_planning_context", "%s: Returning successful solution with %lu states",
                    getName().c_str(), getOMPLSimpleSetup()->getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    ROS_INFO_NAMED("model_based_planning_context", "Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool ompl_interface::ModelBasedPlanningContext::solve(double timeout, unsigned int count)
{
  moveit::tools::Profiler::ScopedBlock sblock("PlanningContext:Solve");
  ompl::time::point start = ompl::time::now();
  preSolve();

  bool result = false;
  if (count <= 1)
  {
    ROS_DEBUG_NAMED("model_based_planning_context", "%s: Solving the planning problem once...", name_.c_str());
    ob::PlannerTerminationCondition ptc =
        ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);
    result = ompl_simple_setup_->solve(ptc) == ob::PlannerStatus::EXACT_SOLUTION;
    last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
    unregisterTerminationCondition();
  }
  else
  {
    ROS_DEBUG_NAMED("model_based_planning_context", "%s: Solving the planning problem %u times...", name_.c_str(),
                    count);
    ompl_parallel_plan_->clearHybridizationPaths();
    if (count <= max_planning_threads_)
    {
      ompl_parallel_plan_->clearPlanners();
      if (ompl_simple_setup_->getPlannerAllocator())
        for (unsigned int i = 0; i < count; ++i)
          ompl_parallel_plan_->addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
      else
        for (unsigned int i = 0; i < count; ++i)
          ompl_parallel_plan_->addPlanner(ot::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));

      ob::PlannerTerminationCondition ptc =
          ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
      registerTerminationCondition(ptc);
      result = ompl_parallel_plan_->solve(ptc, 1, count, true) == ob::PlannerStatus::EXACT_SOLUTION;
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
    else
    {
      ob::PlannerTerminationCondition ptc =
          ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
      registerTerminationCondition(ptc);
      int n = count / max_planning_threads_;
      result = true;
      for (int i = 0; i < n && ptc() == false; ++i)
      {
        ompl_parallel_plan_->clearPlanners();
        if (ompl_simple_setup_->getPlannerAllocator())
          for (unsigned int i = 0; i < max_planning_threads_; ++i)
            ompl_parallel_plan_->addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
        else
          for (unsigned int i = 0; i < max_planning_threads_; ++i)
            ompl_parallel_plan_->addPlanner(ot::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));
        bool r = ompl_parallel_plan_->solve(ptc, 1, count, true) == ob::PlannerStatus::EXACT_SOLUTION;
        result = result && r;
      }
      n = count % max_planning_threads_;
      if (n && ptc() == false)
      {
        ompl_parallel_plan_->clearPlanners();
        if (ompl_simple_setup_->getPlannerAllocator())
          for (int i = 0; i < n; ++i)
            ompl_parallel_plan_->addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
        else
          for (int i = 0; i < n; ++i)
            ompl_parallel_plan_->addPlanner(ot::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));
        bool r = ompl_parallel_plan_->solve(ptc, 1, count, true) == ob::PlannerStatus::EXACT_SOLUTION;
        result = result && r;
      }
      last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
  }

  postSolve();

  return result;
}

void ompl_interface::ModelBasedPlanningContext::registerTerminationCondition(const ob::PlannerTerminationCondition& ptc)
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = &ptc;
}

void ompl_interface::ModelBasedPlanningContext::unregisterTerminationCondition()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = NULL;
}

bool ompl_interface::ModelBasedPlanningContext::terminate()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  if (ptc_)
    ptc_->terminate();
  return true;
}

bool ompl_interface::ModelBasedPlanningContext::saveConstraintApproximations(const ros::NodeHandle& nh)
{
  std::string cpath;
  if (nh.getParam("constraint_approximations_path", cpath))
  {
    constraints_library_->saveConstraintApproximations(cpath);
    return true;
  }
  ROS_WARN("ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
  return false;
}

bool ompl_interface::ModelBasedPlanningContext::loadConstraintApproximations(const ros::NodeHandle& nh)
{
  std::string cpath;
  if (nh.getParam("constraint_approximations_path", cpath))
  {
    constraints_library_->loadConstraintApproximations(cpath);
    std::stringstream ss;
    constraints_library_->printConstraintApproximations(ss);
    ROS_INFO_STREAM(ss.str());
    return true;
  }
  return false;
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::ModelBasedPlanningContext::getStateSpaceFactory1(
    const std::string& /* dummy */, const std::string& factory_type) const
{
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f =
      factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::ModelBasedPlanningContext::getStateSpaceFactory2(
    const std::string& group, const moveit_msgs::MotionPlanRequest& req) const
{
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin();
       it != state_space_factories_.end(); ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, spec_.robot_model_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR_NAMED("planning_context_manager", "There are no known state spaces that can represent the given planning "
                                                "problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    ROS_DEBUG_NAMED("planning_context_manager", "Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

ompl_interface::ModelBasedStateSpacePtr ompl_interface::ModelBasedPlanningContext::getStateSpace()
{
  // Check if sampling in JointModelStateSpace is enforced for this group by user. This is done by setting
  // 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via
  // IK. However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be
  // flipped, leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection
  // sampling in JointModelStateSpace.
  registerDefaultStateSpaces();
  const std::map<std::string, std::string>& config = getSpecificationConfig();

  StateSpaceFactoryTypeSelector factory_selector;
  std::map<std::string, std::string>::const_iterator it = config.find("enforce_joint_model_state_space");

  if (it != config.end() && boost::lexical_cast<bool>(it->second))
    factory_selector = boost::bind(&ModelBasedPlanningContext::getStateSpaceFactory1, this, _1,
                                   JointModelStateSpace::PARAMETERIZATION_TYPE);
  else
    factory_selector = boost::bind(&ModelBasedPlanningContext::getStateSpaceFactory2, this, _1, spec_.req_);

  const ompl_interface::ModelBasedStateSpaceFactoryPtr& factory = factory_selector(getGroupName());
  ModelBasedStateSpaceSpecification space_spec(spec_.robot_model_, spec_.jmg_);
  space_spec.joint_bounds_ = spec_.joint_bounds_;

  return factory->getNewStateSpace(space_spec);
}

PLUGINLIB_EXPORT_CLASS(ompl_interface::ModelBasedPlanningContext, ompl_interface::OMPLPlanningContext)
