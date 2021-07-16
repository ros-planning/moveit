/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Henning Kayser */

#include <stdexcept>

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_cpp
{
constexpr char LOGNAME[] = "planning_component";

PlanningComponent::PlanningComponent(const std::string& group_name, const MoveItCppPtr& moveit_cpp)
  : nh_(moveit_cpp->getNodeHandle()), moveit_cpp_(moveit_cpp), group_name_(group_name)
{
  joint_model_group_ = moveit_cpp_->getRobotModel()->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    std::string error = "Could not find joint model group '" + group_name + "'.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }
  planning_pipeline_names_ = moveit_cpp_->getPlanningPipelineNames(group_name);
  plan_request_parameters_.load(nh_);
  ROS_DEBUG_STREAM_NAMED(
      LOGNAME, "Plan request parameters loaded with --"
                   << " planning_pipeline: " << plan_request_parameters_.planning_pipeline << ","
                   << " planner_id: " << plan_request_parameters_.planner_id << ","
                   << " planning_time: " << plan_request_parameters_.planning_time << ","
                   << " planning_attempts: " << plan_request_parameters_.planning_attempts << ","
                   << " max_velocity_scaling_factor: " << plan_request_parameters_.max_velocity_scaling_factor << ","
                   << " max_acceleration_scaling_factor: " << plan_request_parameters_.max_acceleration_scaling_factor);
}

PlanningComponent::PlanningComponent(const std::string& group_name, const ros::NodeHandle& nh)
  : PlanningComponent(group_name, std::make_shared<MoveItCpp>(nh))
{
}

PlanningComponent::~PlanningComponent()
{
  ROS_INFO_NAMED(LOGNAME, "Deleting PlanningComponent '%s'", group_name_.c_str());
  clearContents();
}

const std::vector<std::string> PlanningComponent::getNamedTargetStates()
{
  if (joint_model_group_)
  {
    return joint_model_group_->getDefaultStateNames();
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Unable to find joint group with name '%s'.", group_name_.c_str());
  }

  std::vector<std::string> empty;
  return empty;
}

const std::string& PlanningComponent::getPlanningGroupName() const
{
  return group_name_;
}

PlanningComponent::PlanSolution PlanningComponent::plan(const PlanRequestParameters& parameters)
{
  last_plan_solution_ = std::make_shared<PlanSolution>();
  if (!joint_model_group_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrieve joint model group for name '%s'.", group_name_.c_str());
    last_plan_solution_->error_code = MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME);
    return *last_plan_solution_;
  }

  // Clone current planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      moveit_cpp_->getPlanningSceneMonitorNonConst();
  planning_scene_monitor->updateFrameTransforms();
  planning_scene_monitor->lockSceneRead();  // LOCK planning scene
  planning_scene::PlanningScenePtr planning_scene =
      planning_scene::PlanningScene::clone(planning_scene_monitor->getPlanningScene());
  planning_scene_monitor->unlockSceneRead();  // UNLOCK planning scene
  planning_scene_monitor.reset();             // release this pointer

  // Init MotionPlanRequest
  ::planning_interface::MotionPlanRequest req;
  req.group_name = group_name_;
  req.planner_id = parameters.planner_id;
  req.num_planning_attempts = std::max(1, parameters.planning_attempts);
  req.allowed_planning_time = parameters.planning_time;
  req.max_velocity_scaling_factor = parameters.max_velocity_scaling_factor;
  req.max_acceleration_scaling_factor = parameters.max_acceleration_scaling_factor;
  if (workspace_parameters_set_)
    req.workspace_parameters = workspace_parameters_;

  // Set start state
  moveit::core::RobotStatePtr start_state = considered_start_state_;
  if (!start_state)
    start_state = moveit_cpp_->getCurrentState();
  start_state->update();
  moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);
  planning_scene->setCurrentState(*start_state);

  // Set goal constraints
  if (current_goal_constraints_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No goal constraints set for planning request");
    last_plan_solution_->error_code = MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
    return *last_plan_solution_;
  }
  req.goal_constraints = current_goal_constraints_;

  // Run planning attempt
  ::planning_interface::MotionPlanResponse res;
  if (planning_pipeline_names_.find(parameters.planning_pipeline) == planning_pipeline_names_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No planning pipeline available for name '%s'", parameters.planning_pipeline.c_str());
    last_plan_solution_->error_code = MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
    return *last_plan_solution_;
  }
  const planning_pipeline::PlanningPipelinePtr pipeline =
      moveit_cpp_->getPlanningPipelines().at(parameters.planning_pipeline);
  pipeline->generatePlan(planning_scene, req, res);
  last_plan_solution_->error_code = res.error_code_.val;
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return *last_plan_solution_;
  }
  last_plan_solution_->start_state = req.start_state;
  last_plan_solution_->trajectory = res.trajectory_;
  // TODO(henningkayser): Visualize trajectory
  // std::vector<const moveit::core::LinkModel*> eef_links;
  // if (joint_model_group->getEndEffectorTips(eef_links))
  //{
  //  for (const auto& eef_link : eef_links)
  //  {
  //    ROS_INFO_STREAM("Publishing trajectory for end effector " << eef_link->getName());
  //    visual_tools_->publishTrajectoryLine(last_solution_trajectory_, eef_link);
  //    visual_tools_->publishTrajectoryPath(last_solution_trajectory_, false);
  //    visual_tools_->publishRobotState(last_solution_trajectory_->getLastWayPoint(), rviz_visual_tools::TRANSLUCENT);
  //  }
  //}
  return *last_plan_solution_;
}

PlanningComponent::PlanSolution PlanningComponent::plan()
{
  return plan(plan_request_parameters_);
}

bool PlanningComponent::setStartState(const moveit::core::RobotState& start_state)
{
  considered_start_state_ = std::make_shared<moveit::core::RobotState>(start_state);
  return true;
}

moveit::core::RobotStatePtr PlanningComponent::getStartState()
{
  if (considered_start_state_)
    return considered_start_state_;
  else
  {
    moveit::core::RobotStatePtr s;
    moveit_cpp_->getCurrentState(s, 1.0);
    return s;
  }
}

bool PlanningComponent::setStartState(const std::string& start_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), start_state_name) == named_targets.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No predefined joint state found for target name '%s'", start_state_name.c_str());
    return false;
  }
  moveit::core::RobotState start_state(moveit_cpp_->getRobotModel());
  start_state.setToDefaultValues(joint_model_group_, start_state_name);
  return setStartState(start_state);
}

void PlanningComponent::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

std::map<std::string, double> PlanningComponent::getNamedTargetStateValues(const std::string& name)
{
  // TODO(henningkayser): verify result
  std::map<std::string, double> positions;
  joint_model_group_->getVariableDefaultPositions(name, positions);
  return positions;
}

void PlanningComponent::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
  workspace_parameters_set_ = true;
}

void PlanningComponent::unsetWorkspace()
{
  workspace_parameters_set_ = false;
}

bool PlanningComponent::setGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints)
{
  current_goal_constraints_ = goal_constraints;
  return true;
}

bool PlanningComponent::setGoal(const moveit::core::RobotState& goal_state)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_) };
  return true;
}

bool PlanningComponent::setGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& link_name)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(link_name, goal_pose) };
  return true;
}

bool PlanningComponent::setGoal(const std::string& goal_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), goal_state_name) == named_targets.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No predefined joint state found for target name '%s'", goal_state_name.c_str());
    return false;
  }
  moveit::core::RobotState goal_state(moveit_cpp_->getRobotModel());
  goal_state.setToDefaultValues(joint_model_group_, goal_state_name);
  return setGoal(goal_state);
}

bool PlanningComponent::execute(bool blocking)
{
  if (!last_plan_solution_)
  {
    ROS_ERROR_NAMED(LOGNAME, "There is no successfull plan to execute");
    return false;
  }

  // TODO(henningkayser): parameterize timestamps if required
  // trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // if (!totg.computeTimeStamps(*last_solution_trajectory_, max_velocity_scaling_factor_,
  // max_acceleration_scaling_factor_))
  //{
  //  ROS_ERROR("Failed to parameterize trajectory");
  //  return false;
  //}
  return moveit_cpp_->execute(group_name_, last_plan_solution_->trajectory, blocking);
}

const PlanningComponent::PlanSolutionPtr PlanningComponent::getLastPlanSolution()
{
  return last_plan_solution_;
}

void PlanningComponent::clearContents()
{
  considered_start_state_.reset();
  last_plan_solution_.reset();
  current_goal_constraints_.clear();
  moveit_cpp_.reset();
  planning_pipeline_names_.clear();
}
}  // namespace moveit_cpp
