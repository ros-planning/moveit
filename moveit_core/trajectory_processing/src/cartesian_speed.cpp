/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Benjamin Scholz
 *  Copyright (c) 2021, Thies Oelerich
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
 *   * Neither the name of the authors nor the names of other
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

/* Authors: Benjamin Scholz, Thies Oelerich */

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/cartesian_speed.h>

// Name of logger
const char* LOGGER_NAME = "trajectory_processing.cartesian_speed";

namespace trajectory_processing
{
bool limitMaxCartesianLinkSpeed(robot_trajectory::RobotTrajectory& trajectory, const double max_speed,
                                std::string link_name)
{
  // In case the link name is not set, retrieve an end effector name from the
  // joint model group specified in the robot trajectory
  if (link_name.empty())
  {
    std::vector<std::string> tips;
    trajectory.getGroup()->getEndEffectorTips(tips);
    if (tips.empty())
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "No end effector defined for group attached to trajectory, cannot set max "
                                          "cartesian link speed without argument.");
      return false;
    }
    link_name = tips[0];
    if (tips.size() > 1)
      ROS_INFO_STREAM_NAMED(LOGGER_NAME,
                            "More than one end effector found, using first one which is '" << link_name << "'");
  }

  // Get link model based on given end link
  if (!trajectory.getGroup()->hasLinkModel(link_name))
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Link model was not specified in the robot trajectory");
  const moveit::core::LinkModel* link_model = trajectory.getGroup()->getLinkModel(link_name);

  // Call function for speed setting using the created link model
  return limitMaxCartesianLinkSpeed(trajectory, max_speed, link_model);
}

bool limitMaxCartesianLinkSpeed(robot_trajectory::RobotTrajectory& trajectory, const double max_speed,
                                const moveit::core::LinkModel* link_model)
{
  if (max_speed <= 0.0)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Link speed must be greater than 0.");
    return false;
  }

  size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints == 0)
    return false;

  robot_state::RobotStatePtr kinematic_state = trajectory.getFirstWayPointPtr();

  // do forward kinematics to get cartesian positions of link for current waypoint
  double euclidean_distance, new_time_diff, old_time_diff;
  std::vector<double> time_diff(num_waypoints - 1, 0.0);

  for (size_t i = 0; i < num_waypoints - 1; i++)
  {
    // get link state for current and next waypoint
    const Eigen::Isometry3d& current_link_state = trajectory.getWayPointPtr(i)->getGlobalLinkTransform(link_model);
    const Eigen::Isometry3d& next_link_state = trajectory.getWayPointPtr(i + 1)->getGlobalLinkTransform(link_model);

    // get euclidean distance between the two waypoints
    euclidean_distance = (next_link_state.translation() - current_link_state.translation()).norm();

    new_time_diff = (euclidean_distance / max_speed);
    old_time_diff = trajectory.getWayPointDurationFromPrevious(i + 1);

    // if constraints allow, save the new time difference between waypoints
    if (new_time_diff > old_time_diff)
    {
      time_diff[i] = new_time_diff;
    }
    else
    {
      time_diff[i] = old_time_diff;
    }
  }
  // update time stamps, velocities and accelerations of the trajectory
  IterativeParabolicTimeParameterization::updateTrajectory(trajectory, time_diff);
  return true;
}
}  // namespace trajectory_processing
