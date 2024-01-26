/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include "pilz_industrial_motion_planner/trajectory_generator_circ.h"
#include "pilz_industrial_motion_planner/path_circle_generator.h"

#include <cassert>
#include <sstream>

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pilz_industrial_motion_planner
{
TrajectoryGeneratorCIRC::TrajectoryGeneratorCIRC(const moveit::core::RobotModelConstPtr& robot_model,
                                                 const LimitsContainer& planner_limits,
                                                 const std::string& /*group_name*/)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{
  if (!planner_limits_.hasFullCartesianLimits())
  {
    throw TrajectoryGeneratorInvalidLimitsException(
        "Cartesian limits are not fully set for CIRC trajectory generator.");
  }
}

void TrajectoryGeneratorCIRC::cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& req) const
{
  if (!(req.path_constraints.name == "interim" || req.path_constraints.name == "center"))
  {
    std::ostringstream os;
    os << "No path constraint named \"interim\" or \"center\" found (found "
          "unknown constraint: "
       << "\"req.path_constraints.name\""
       << " instead)";
    throw UnknownPathConstraintName(os.str());
  }

  if (req.path_constraints.position_constraints.size() != 1)
  {
    throw NoPositionConstraints("CIRC trajectory generator needs valid a position constraint");
  }

  if (req.path_constraints.position_constraints.front().constraint_region.primitive_poses.size() != 1)
  {
    throw NoPrimitivePose("CIRC trajectory generator needs valid a primitive pose");
  }
}

void TrajectoryGeneratorCIRC::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                    const planning_interface::MotionPlanRequest& req,
                                                    TrajectoryGenerator::MotionPlanInfo& info) const
{
  ROS_DEBUG("Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  robot_state::RobotState robot_state = scene->getCurrentState();

  // goal given in joint space
  if (!req.goal_constraints.front().joint_constraints.empty())
  {
    // TODO: link name from goal constraint and path constraint
    info.link_name = req.path_constraints.position_constraints.front().link_name;
    if (!robot_model_->hasLinkModel(info.link_name))
    {
      throw UnknownLinkNameOfAuxiliaryPoint("Unknown link name of CIRC auxiliary point");
    }

    if (req.goal_constraints.front().joint_constraints.size() !=
        robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size())
    {
      std::ostringstream os;
      os << "Number of joint constraint = " << req.goal_constraints.front().joint_constraints.size()
         << " not equal to active joints of group = "
         << robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size();
      throw NumberOfConstraintsMismatch(os.str());
    }

    // initializing all joints of the model
    for (const auto& joint_name : robot_model_->getVariableNames())
    {
      info.goal_joint_position[joint_name] = 0;
    }

    for (const auto& joint_item : req.goal_constraints.front().joint_constraints)
    {
      info.goal_joint_position[joint_item.joint_name] = joint_item.position;
    }

    computeLinkFK(robot_state, info.link_name, info.goal_joint_position, info.goal_pose);
  }
  // goal given in Cartesian space
  else
  {
    std::string frame_id;

    info.link_name = req.goal_constraints.front().position_constraints.front().link_name;
    const auto& offset = req.goal_constraints.front().position_constraints.front().target_point_offset;
    info.ioffset = Eigen::Translation3d(offset.x, offset.y, offset.z).inverse();

    if (req.goal_constraints.front().position_constraints.front().header.frame_id.empty() ||
        req.goal_constraints.front().orientation_constraints.front().header.frame_id.empty())
    {
      ROS_WARN("Frame id is not set in position/orientation constraints of "
               "goal. Use model frame as default");
      frame_id = robot_model_->getModelFrame();
    }
    else
    {
      frame_id = req.goal_constraints.front().position_constraints.front().header.frame_id;
    }

    // goal pose with optional offset wrt. the planning frame
    info.goal_pose = scene->getFrameTransform(frame_id) * getPose(req.goal_constraints.front());
    frame_id = robot_model_->getModelFrame();

    // check goal pose ik before Cartesian motion plan start
    std::map<std::string, double> ik_solution;
    if (!computePoseIK(scene, info.group_name, info.link_name, info.goal_pose * info.ioffset, frame_id,
                       info.start_joint_position, ik_solution))
    {
      // LCOV_EXCL_START
      std::ostringstream os;
      os << "Failed to compute inverse kinematics for link: " << info.link_name << " of goal pose";
      throw CircInverseForGoalIncalculable(os.str());
      // LCOV_EXCL_STOP // not able to trigger here since lots of checks before
      // are in place
    }
  }
  computeLinkFK(robot_state, info.link_name, info.start_joint_position, info.start_pose);
  info.start_pose *= info.ioffset.inverse();  // add offset to start pose

  // center point's frame_id
  std::string center_point_frame_id = req.path_constraints.position_constraints.front().header.frame_id;
  info.circ_path_point.first = req.path_constraints.name;
  if (center_point_frame_id.empty())
  {
    ROS_WARN_STREAM("No frame_id specified in path constraint for circle " << req.path_constraints.name << " point. "
                                                                           << "Using model frame as fallback.");
    center_point_frame_id = robot_model_->getModelFrame();
  }
  tf2::fromMsg(req.path_constraints.position_constraints.front().constraint_region.primitive_poses.front().position,
               info.circ_path_point.second);
  info.circ_path_point.second = scene->getFrameTransform(center_point_frame_id) * info.circ_path_point.second;
}

void TrajectoryGeneratorCIRC::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                   const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                                   double sampling_time, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  std::unique_ptr<KDL::Path> cart_path(setPathCIRC(plan_info));
  std::unique_ptr<KDL::VelocityProfile> vel_profile(
      cartesianTrapVelocityProfile(req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, cart_path));

  // combine path and velocity profile into Cartesian trajectory
  // with the third parameter set to false, KDL::Trajectory_Segment does not
  // take
  // the ownship of Path and Velocity Profile
  KDL::Trajectory_Segment cart_trajectory(cart_path.get(), vel_profile.get(), false);

  moveit_msgs::MoveItErrorCodes error_code;
  // sample the Cartesian trajectory and compute joint trajectory using inverse
  // kinematics
  if (!generateJointTrajectory(scene, planner_limits_.getJointLimitContainer(), cart_trajectory, plan_info.group_name,
                               plan_info.link_name, plan_info.ioffset, plan_info.start_joint_position, sampling_time,
                               joint_trajectory, error_code))
  {
    throw CircTrajectoryConversionFailure("Failed to generate valid joint trajectory from the Cartesian path",
                                          error_code.val);
  }
}

std::unique_ptr<KDL::Path> TrajectoryGeneratorCIRC::setPathCIRC(const MotionPlanInfo& info) const
{
  ROS_DEBUG("Set Cartesian path for CIRC command.");

  KDL::Frame start_pose, goal_pose;
  tf2::fromMsg(tf2::toMsg(info.start_pose), start_pose);
  tf2::fromMsg(tf2::toMsg(info.goal_pose), goal_pose);

  const auto& eigen_path_point = info.circ_path_point.second;
  const KDL::Vector path_point{ eigen_path_point.x(), eigen_path_point.y(), eigen_path_point.z() };

  // pass the ratio of translational by rotational velocity as equivalent radius
  // to get a trajectory with rotational speed, if no (or very little)
  // translational distance
  // The KDL::Path implementation chooses the motion with the longer duration
  // (translation vs. rotation)
  // and uses eqradius as scaling factor between the distances.
  double eqradius = planner_limits_.getCartesianLimits().getMaxTranslationalVelocity() /
                    planner_limits_.getCartesianLimits().getMaxRotationalVelocity();

  try
  {
    if (info.circ_path_point.first == "center")
    {
      return PathCircleGenerator::circleFromCenter(start_pose, goal_pose, path_point, eqradius);
    }
    else  // if (info.circ_path_point.first == "interim")
    {
      return PathCircleGenerator::circleFromInterim(start_pose, goal_pose, path_point, eqradius);
    }
  }
  catch (KDL::Error_MotionPlanning_Circle_No_Plane& e)
  {
    std::ostringstream os;
    os << "Failed to create path object for circle." << e.Description();
    throw CircleNoPlane(os.str());
  }
  catch (KDL::Error_MotionPlanning_Circle_ToSmall& e)
  {
    std::ostringstream os;
    os << "Failed to create path object for circle." << e.Description();
    throw CircleToSmall(os.str());
  }
  catch (ErrorMotionPlanningCenterPointDifferentRadius& e)
  {
    std::ostringstream os;
    os << "Failed to create path object for circle." << e.Description();
    throw CenterPointDifferentRadius(os.str());
  }

  return nullptr;
}

}  // namespace pilz_industrial_motion_planner
