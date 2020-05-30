/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*      Title     : jog_arm.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include "moveit_jog_arm/jog_arm.h"

static const std::string LOGNAME = "jog_arm";

namespace moveit_jog_arm
{
JogArm::JogArm(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh), planning_scene_monitor_(planning_scene_monitor)
{
  // Read ROS parameters, typically from YAML file
  if (!readParameters())
    exit(EXIT_FAILURE);

  // loop period
  period_ = ros::Duration(parameters_.publish_period);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(parameters_.command_out_topic, 1);
  else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_out_topic, 1);

  // Subscribe to internal namespace
  ros::NodeHandle internal_nh("~internal");
  joint_trajectory_sub_ = internal_nh.subscribe("joint_trajectory", 1, &JogArm::jointTrajectoryCB, this);
  ok_to_publish_sub_ = internal_nh.subscribe("ok_to_publish", 1, &JogArm::okToPublishCB, this);

  // Wait for incoming topics to appear
  ROS_DEBUG_NAMED(LOGNAME, "Waiting for JointState topic");
  ros::topic::waitForMessage<sensor_msgs::JointState>(parameters_.joint_topic);

  jog_calcs_ = std::make_unique<JogCalcs>(nh_, parameters_, planning_scene_monitor_);

  collision_checker_ = std::make_unique<CollisionCheckThread>(nh_, parameters_, planning_scene_monitor_);
}

// Read ROS parameters, typically from YAML file
bool JogArm::readParameters()
{
  std::size_t error = 0;

  // Specified in the launch file. All other parameters will be read from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);
  if (parameter_ns.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "A namespace must be specified in the launch file, like:");
    ROS_ERROR_STREAM_NAMED(LOGNAME, "<param name=\"parameter_ns\" "
                                    "type=\"string\" "
                                    "value=\"left_jog_server\" />");
    return false;
  }

  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_period", parameters_.publish_period);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_check_rate", parameters_.collision_check_rate);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/num_outgoing_halt_msgs_to_publish",
                                    parameters_.num_outgoing_halt_msgs_to_publish);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/linear", parameters_.linear_scale);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/rotational", parameters_.rotational_scale);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/scale/joint", parameters_.joint_scale);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/low_pass_filter_coeff", parameters_.low_pass_filter_coeff);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_topic", parameters_.joint_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_in_type", parameters_.command_in_type);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/cartesian_command_in_topic",
                                    parameters_.cartesian_command_in_topic);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_command_in_topic", parameters_.joint_command_in_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/robot_link_command_frame",
                                    parameters_.robot_link_command_frame);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/incoming_command_timeout",
                                    parameters_.incoming_command_timeout);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/lower_singularity_threshold",
                                    parameters_.lower_singularity_threshold);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/hard_stop_singularity_threshold",
                                    parameters_.hard_stop_singularity_threshold);
  // parameter was removed, replaced with separate self- and scene-collision proximity thresholds; the logic handling
  // the different possible sets of defined parameters is somewhat complicated at this point
  // TODO(JStech): remove this deprecation warning in ROS Noetic; simplify error case handling
  bool have_self_collision_proximity_threshold = rosparam_shortcuts::get(
      "", nh_, parameter_ns + "/self_collision_proximity_threshold", parameters_.self_collision_proximity_threshold);
  bool have_scene_collision_proximity_threshold = rosparam_shortcuts::get(
      "", nh_, parameter_ns + "/scene_collision_proximity_threshold", parameters_.scene_collision_proximity_threshold);
  double collision_proximity_threshold;
  if (nh_.hasParam(parameter_ns + "/collision_proximity_threshold") &&
      rosparam_shortcuts::get("", nh_, parameter_ns + "/collision_proximity_threshold", collision_proximity_threshold))
  {
    ROS_WARN_NAMED(LOGNAME, "'collision_proximity_threshold' parameter is deprecated, and has been replaced by separate"
                            "'self_collision_proximity_threshold' and 'scene_collision_proximity_threshold' "
                            "parameters. Please update the jogging yaml file.");
    if (!have_self_collision_proximity_threshold)
    {
      parameters_.self_collision_proximity_threshold = collision_proximity_threshold;
      have_self_collision_proximity_threshold = true;
    }
    if (!have_scene_collision_proximity_threshold)
    {
      parameters_.scene_collision_proximity_threshold = collision_proximity_threshold;
      have_scene_collision_proximity_threshold = true;
    }
  }
  error += !have_self_collision_proximity_threshold;
  error += !have_scene_collision_proximity_threshold;
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/move_group_name", parameters_.move_group_name);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/planning_frame", parameters_.planning_frame);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/use_gazebo", parameters_.use_gazebo);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/check_collisions", parameters_.check_collisions);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/joint_limit_margin", parameters_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_out_topic", parameters_.command_out_topic);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/command_out_type", parameters_.command_out_type);
  error +=
      !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_positions", parameters_.publish_joint_positions);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_velocities",
                                    parameters_.publish_joint_velocities);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/publish_joint_accelerations",
                                    parameters_.publish_joint_accelerations);

  // This parameter name was changed recently.
  // Try retrieving from the correct name. If it fails, then try the deprecated name.
  // TODO(andyz): remove this deprecation warning in ROS Noetic
  if (!rosparam_shortcuts::get("", nh_, parameter_ns + "/status_topic", parameters_.status_topic))
  {
    ROS_WARN_NAMED(LOGNAME, "'status_topic' parameter is missing. Recently renamed from 'warning_topic'. Please update "
                            "the jogging yaml file.");
    error += !rosparam_shortcuts::get("", nh_, parameter_ns + "/warning_topic", parameters_.status_topic);
  }

  rosparam_shortcuts::shutdownIfError(parameter_ns, error);

  // Input checking
  if (parameters_.publish_period <= 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'publish_period' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.num_outgoing_halt_msgs_to_publish < 0)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Parameter 'num_outgoing_halt_msgs_to_publish' should be greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.hard_stop_singularity_threshold < parameters_.lower_singularity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'hard_stop_singularity_threshold' "
                            "should be greater than 'lower_singularity_threshold.' "
                            "Check yaml file.");
    return false;
  }
  if ((parameters_.hard_stop_singularity_threshold < 0.) || (parameters_.lower_singularity_threshold < 0.))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameters 'hard_stop_singularity_threshold' "
                            "and 'lower_singularity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.self_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.scene_collision_proximity_threshold < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'scene_collision_proximity_threshold' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.scene_collision_proximity_threshold < parameters_.self_collision_proximity_threshold)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'self_collision_proximity_threshold' should probably be less "
                            "than or equal to 'scene_collision_proximity_threshold'. Check yaml file.");
  }
  if (parameters_.low_pass_filter_coeff < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'low_pass_filter_coeff' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.joint_limit_margin < 0.)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'joint_limit_margin' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }
  if (parameters_.command_in_type != "unitless" && parameters_.command_in_type != "speed_units")
  {
    ROS_WARN_NAMED(LOGNAME, "command_in_type should be 'unitless' or "
                            "'speed_units'. Check yaml file.");
    return false;
  }
  if (parameters_.command_out_type != "trajectory_msgs/JointTrajectory" &&
      parameters_.command_out_type != "std_msgs/Float64MultiArray")
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter command_out_type should be "
                            "'trajectory_msgs/JointTrajectory' or "
                            "'std_msgs/Float64MultiArray'. Check yaml file.");
    return false;
  }
  if (!parameters_.publish_joint_positions && !parameters_.publish_joint_velocities &&
      !parameters_.publish_joint_accelerations)
  {
    ROS_WARN_NAMED(LOGNAME, "At least one of publish_joint_positions / "
                            "publish_joint_velocities / "
                            "publish_joint_accelerations must be true. Check "
                            "yaml file.");
    return false;
  }
  if ((parameters_.command_out_type == "std_msgs/Float64MultiArray") && parameters_.publish_joint_positions &&
      parameters_.publish_joint_velocities)
  {
    ROS_WARN_NAMED(LOGNAME, "When publishing a std_msgs/Float64MultiArray, "
                            "you must select positions OR velocities.");
    return false;
  }
  if (parameters_.collision_check_rate < 0)
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter 'collision_check_rate' should be "
                            "greater than zero. Check yaml file.");
    return false;
  }

  return true;
}

void JogArm::run(const ros::TimerEvent& timer_event)
{
  // Log warning when the last loop duration was longer than the period
  ROS_WARN_STREAM_COND_NAMED(timer_event.profile.last_duration.toSec() > period_.toSec(), LOGNAME,
                             "last_duration: " << timer_event.profile.last_duration.toSec() << " (" << period_.toSec()
                                               << ")");

  // Get the latest joint trajectory
  trajectory_msgs::JointTrajectory joint_trajectory;
  {
    const std::lock_guard<std::mutex> lock(latest_state_mutex_);
    if (latest_joint_trajectory_)
    {
      joint_trajectory = *latest_joint_trajectory_;
    }
  }

  // Publish the most recent trajectory, unless the jogging calculation thread tells not to
  if (ok_to_publish_ && !paused_)
  {
    // Put the outgoing msg in the right format
    // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    {
      joint_trajectory.header.stamp = ros::Time::now();
      outgoing_cmd_pub_.publish(joint_trajectory);
    }
    else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    {
      std_msgs::Float64MultiArray joints;
      if (parameters_.publish_joint_positions)
        joints.data = joint_trajectory.points[0].positions;
      else if (parameters_.publish_joint_velocities)
        joints.data = joint_trajectory.points[0].velocities;
      outgoing_cmd_pub_.publish(joints);
    }
  }
}

void JogArm::start()
{
  setPaused(false);

  // Crunch the numbers in this thread
  jog_calcs_->start();

  // Check collisions in this thread
  if (parameters_.check_collisions)
    collision_checker_->start();

  // Start the jog server timer
  timer_ = nh_.createTimer(period_, &JogArm::run, this);
}

void JogArm::stop()
{
  timer_.stop();
  jog_calcs_->stop();
  collision_checker_->stop();
}

JogArm::~JogArm()
{
  stop();
}

void JogArm::setPaused(bool paused)
{
  paused_ = paused;
  jog_calcs_->setPaused(paused);
  collision_checker_->setPaused(paused);
}

bool JogArm::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  return jog_calcs_->getCommandFrameTransform(transform);
}

void JogArm::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(latest_state_mutex_);
  latest_joint_trajectory_ = msg;
}

void JogArm::okToPublishCB(const std_msgs::BoolConstPtr& msg)
{
  ok_to_publish_ = msg->data;
}

const JogArmParameters& JogArm::getParameters() const
{
  return parameters_;
}

}  // namespace moveit_jog_arm