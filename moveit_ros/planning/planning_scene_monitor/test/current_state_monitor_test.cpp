/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, University of Hamburg
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Michael 'v4hn' Goerner
   Desc: Tests for CurrentStateMonitor
*/

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>

// Main class
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <sensor_msgs/JointState.h>

#include <future>

class CurrentStateMonitorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    robot_model_ptr = moveit::core::RobotModelBuilder{ "test", "a" }
                          .addChain("a->b->c", "revolute")
                          .addCollisionBox("a", { 0.1, 0.1, 0.1 },
                                           [] {
                                             geometry_msgs::Pose p;
                                             p.orientation.w = 1.0;
                                             return p;
                                           }())
                          .addGroupChain("a", "c", "group")
                          .addGroup({ "a" }, { "a-b-joint" }, "group_a")
                          .build();

    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);

    csm = std::make_unique<planning_scene_monitor::CurrentStateMonitor>(robot_model_ptr, nullptr);
    csm->startStateMonitor();

    js_a.name = { "a-b-joint" };
    js_a.position = { 0.1 };
    js_a.velocity = { 0.1 };
    js_a.header.stamp = ros::Time{ 10.0 };

    js_b.name = { "b-c-joint" };
    js_b.position = { 0.2 };
    js_b.velocity = { 0.2 };
    js_b.header.stamp = ros::Time{ 9.0 };

    js_ab.name = { "a-b-joint", "b-c-joint" };
    js_ab.position = { 0.3, 0.3 };
    js_ab.velocity = { 0.3, 0.3 };
    js_ab.header.stamp = ros::Time{ 11.0 };

    js_a_old.name = { "a-b-joint" };
    js_a_old.position = { 0.4 };
    js_a_old.velocity = { 0.4 };
    js_a_old.header.stamp = ros::Time{ 9.0 };
  }

  void TearDown() override
  {
    joint_state_pub_.shutdown();
  }

  void sendJointStateAndWait(const sensor_msgs::JointState& js)
  {
    std::promise<void> promise;
    auto future = promise.get_future();
    csm->addUpdateCallback([&promise](const sensor_msgs::JointStateConstPtr& /*unused*/) { promise.set_value(); });
    joint_state_pub_.publish(js);

    std::future_status status{ std::future_status::timeout };
    EXPECT_NO_THROW(status = future.wait_for(std::chrono::seconds{ 1 }));
    csm->clearUpdateCallbacks();
    EXPECT_EQ(std::future_status::ready, status);
  }

protected:
  ros::NodeHandle nh;
  moveit::core::RobotModelConstPtr robot_model_ptr;
  planning_scene_monitor::CurrentStateMonitorPtr csm;
  ros::Publisher joint_state_pub_;

  sensor_msgs::JointState js_a, js_b, js_ab, js_a_old;
};

TEST_F(CurrentStateMonitorTest, CompleteStateTest)
{
  EXPECT_FALSE(csm->haveCompleteState());

  sendJointStateAndWait(js_a);

  EXPECT_FALSE(csm->haveCompleteState());

  sendJointStateAndWait(js_ab);

  EXPECT_TRUE(csm->haveCompleteState());
}

TEST_F(CurrentStateMonitorTest, StateUpdateTest)
{
  sendJointStateAndWait(js_a);
  EXPECT_EQ(js_a.position[0], csm->getCurrentState()->getVariablePosition("a-b-joint"));

  sendJointStateAndWait(js_ab);
  std::vector<double> positions;
  csm->getCurrentState()->copyJointGroupPositions("group", positions);
  EXPECT_EQ(positions, js_ab.position);
}

TEST_F(CurrentStateMonitorTest, IncrementalTimeStamps)
{
  sendJointStateAndWait(js_a);
  EXPECT_EQ(js_a.header.stamp, csm->getCurrentStateTime());

  sendJointStateAndWait(js_b);
  EXPECT_EQ(js_b.header.stamp, csm->getCurrentStateTime())
      << "older partial joint state was ignored in current state retrieval!";

  js_b.position = { 0.25 };
  js_b.header.stamp = ros::Time{ 10.5 };
  sendJointStateAndWait(js_b);
  EXPECT_EQ(js_a.header.stamp, csm->getCurrentStateTime())
      << "older partial joint state was ignored in current state retrieval!";

  sendJointStateAndWait(js_ab);
  EXPECT_EQ(js_ab.header.stamp, csm->getCurrentStateTime()) << "newer stamp did not update csm";

  // send old state for a joint known at a more current time to trigger rosbag loop detection
  sendJointStateAndWait(js_a_old);
  EXPECT_FALSE(csm->haveCompleteState())
      << "jumped back in time for a known joint, but csm still claims it knows all joints";
  EXPECT_EQ(js_a_old.header.stamp, csm->getCurrentStateTime())
      << "jumping back for a known joint did not reset state time";
  EXPECT_EQ(js_a_old.position[0], csm->getCurrentState()->getVariablePosition("a-b-joint"));
}

TEST_F(CurrentStateMonitorTest, NonMonotonicTimeStampsDueToPartialJoints)
{
  sendJointStateAndWait(js_a);
  EXPECT_EQ(js_a.header.stamp, csm->getCurrentStateTime());

  sendJointStateAndWait(js_b);
  EXPECT_EQ(js_b.header.stamp, csm->getCurrentStateTime())
      << "older partial joint state was ignored in current state retrieval!";
  EXPECT_EQ(js_a.header.stamp, csm->getCurrentStateTime("group_a"))
      << "Group is aware of the timestamp of non-group joints!";

  js_b.position = { 0.25 };
  js_b.header.stamp = ros::Time{ 13.0 };
  sendJointStateAndWait(js_b);
  EXPECT_EQ(js_a.header.stamp, csm->getCurrentStateTime())
      << "older partial joint state was ignored in current state retrieval!";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "current_state_monitor_test");

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  int result = RUN_ALL_TESTS();

  return result;
}
