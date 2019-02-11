/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2018, Michael 'v4hn' Goerner
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

/* Authors: Ioan Sucan, Michael Goerner */

#include <tf/transform_datatypes.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/profiler/profiler.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/kinematic_constraints/utils.h>

#include <boost/math/constants/constants.hpp>
#include <sstream>

static const std::string LOGNAME = "generate_state_database";

static const std::string ROBOT_DESCRIPTION = "robot_description";

static const std::string CONSTRAINT_PARAMETER = "constraints";

struct GenerateStateDatabaseParameters
{
  bool setFromHandle(ros::NodeHandle& nh)
  {
    use_current_scene_ = nh.param("use_current_scene", false);

    // number of states in joint space approximation
    construction_opts_.samples = nh.param("state_cnt", 10000);

    // generate edges together with states?
    construction_opts_.edges_per_sample = nh.param("edges_per_sample", 0);
    construction_opts_.max_edge_length = nh.param("max_edge_length", 0.2);

    // verify constraint validity on edges
    construction_opts_.explicit_motions = nh.param("explicit_motions", true);
    construction_opts_.explicit_points_resolution = nh.param("explicit_points_resolution", 0.05);
    construction_opts_.max_explicit_points = nh.param("max_explicit_points", 200);

    // local planning in JointModel state space
    construction_opts_.state_space_parameterization =
        nh.param<std::string>("state_space_parameterization", "JointModel");

    output_folder_ = nh.param<std::string>("output_folder", "constraint_approximations_database");

    if (!nh.getParam("planning_group", planning_group_))
    {
      ROS_FATAL_NAMED(LOGNAME, "~planning_group parameter has to be specified.");
      return false;
    }

    XmlRpc::XmlRpcValue constraint_description;
    if (!nh.getParam(CONSTRAINT_PARAMETER, constraint_description) ||
        !kinematic_constraints::constructConstraints(constraint_description, constraints_))
    {
      ROS_FATAL_STREAM_NAMED(LOGNAME,
                             "Could not find valid constraint description in parameter '"
                                 << nh.resolveName(CONSTRAINT_PARAMETER)
                                 << "'. "
                                    "Please upload correct correct constraint description or remap the parameter.");
      return false;
    }

    return true;
  };

  std::string planning_group_;

  // path to folder for generated database
  std::string output_folder_;

  // request the current scene via get_planning_scene service
  bool use_current_scene_;

  // constraints the approximation should satisfy
  moveit_msgs::Constraints constraints_;

  // internal parameters of approximation generator
  ompl_interface::ConstraintApproximationConstructionOptions construction_opts_;
};

void computeDB(const planning_scene::PlanningScenePtr& scene, struct GenerateStateDatabaseParameters& params)
{
  // required by addConstraintApproximation
  scene->getCurrentStateNonConst().update();

  ompl_interface::OMPLInterface ompl_interface(scene->getRobotModel());

  ROS_INFO_STREAM_NAMED(LOGNAME, "Generating Joint Space Constraint Approximation Database for constraint:\n"
                                     << params.constraints_);

  ompl_interface::ConstraintApproximationConstructionResults result =
      ompl_interface.getConstraintsLibrary().addConstraintApproximation(params.constraints_, params.planning_group_,
                                                                        scene, params.construction_opts_);

  if (!result.approx)
  {
    ROS_FATAL_NAMED(LOGNAME, "Failed to generate approximation.");
    return;
  }
  ompl_interface.getConstraintsLibrary().saveConstraintApproximations(params.output_folder_);
  ROS_INFO_STREAM_NAMED(LOGNAME,
                        "Successfully generated Joint Space Constraint Approximation Database for constraint:\n"
                            << params.constraints_);
  ROS_INFO_STREAM_NAMED(LOGNAME, "The database has been saved in your local folder '" << params.output_folder_ << "'");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "construct_tool_constraint_database", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  GenerateStateDatabaseParameters params;
  if (!params.setFromHandle(nh))
    return 1;

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getRobotModel())
    return 1;

  if (params.use_current_scene_)
  {
    ROS_INFO_NAMED(LOGNAME, "Requesting current planning scene to generate database");
    if (!psm.requestPlanningSceneState())
    {
      ROS_FATAL_NAMED(LOGNAME, "Abort. The current scene could not be retrieved.");
      return 1;
    }
  }

  if (kinematic_constraints::isEmpty(params.constraints_))
  {
    ROS_FATAL_NAMED(LOGNAME, "Abort. Constraint description is an empty set of constraints.");
    return 1;
  }

  computeDB(psm.getPlanningScene(), params);

  return 0;
}
