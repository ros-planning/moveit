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

/* Author: Omid Heidari */

#ifndef TRAJOPT_INTERFACE_TRAJOPT_INTERFACE_H
#define TRAJOPT_INTERFACE_TRAJOPT_INTERFACE_H

#include <ros/ros.h>

#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/solver_interface.hpp>

#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <tesseract_planning/basic_planner_types.h>

#include <trajopt_sco/solver_interface.hpp>

#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>

#include "problem_description.h"

#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>

namespace trajopt_interface
{
MOVEIT_CLASS_FORWARD(TrajOptInterface);

class TrajOptInterface //: public trajopt::TrajOptPlanner
{
public:

  TrajOptInterface(const ros::NodeHandle& nh = ros::NodeHandle("~"));

  const sco::BasicTrustRegionSQPParameters& getParams() const { return params_; }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
             const sco::BasicTrustRegionSQPParameters& params, planning_interface::MotionPlanResponse& res);

  trajopt::TrajArray generateInitialTrajectory(const int& num_steps, const std::vector<double>& joint_vals);

protected:
  /** @brief Configure everything using the param server */
  void setParamsFromRosParamServer();
  void setDefaultParams();

  ros::NodeHandle nh_;  /// The ROS node handle

  sco::BasicTrustRegionSQPParameters params_;

  sco::Optimizer::Callback callbacks_;

  TrajOptProblemPtr prob_;

  // a function to convert TrajArray (TrajArray has no dependency on tesseract) from trajopt_ros to trajectory_msgs::JointTrajectory.Points
  trajectory_msgs::JointTrajectory convertTrajArrayToJointTrajectory(const trajopt::TrajArray& traj_array, const std::vector<std::string>& j_names);
};

void callBackFunc(sco::OptProb* opt_prob, sco::OptResults& opt_res);

}

#endif
