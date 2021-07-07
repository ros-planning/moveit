/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Iron Ox.
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
 *   * Neither the name of Iron Ox nor the names of its
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

/* Author: Bianca Homberg, based off empty.cpp by Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>

namespace default_planner_request_adapters
{
class PublishIntermediateTrajectory : public planning_request_adapter::PlanningRequestAdapter
{
public:
  ros::Publisher intermediate_trajectory_;

  PublishIntermediateTrajectory() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& node_handle) override
  {
    std::string topic_name = "intermediate_trajectory";
    node_handle.getParam("intermediate_trajectory_topic_name", topic_name);
    ros::NodeHandle nh = node_handle;
    intermediate_trajectory_ = nh.advertise<moveit_msgs::RobotTrajectory>(topic_name, 1000);
  }

  std::string getDescription() const override
  {
    return "Publish intermediate trajectory at current location within planning adapter chain.";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result)
    {
      moveit_msgs::MotionPlanResponse msg;
      res.getMessage(msg);
      intermediate_trajectory_.publish(msg.trajectory);
    }
    return result;
  }
};
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::PublishIntermediateTrajectory,
                            planning_request_adapter::PlanningRequestAdapter);
