/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Lautman */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <urdf_parser/urdf_parser.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>

class OneRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(mlautman): Use new testing framework for loading models
    // https://ros-planning.github.io/moveit_tutorials/doc/tests/tests_tutorial.html
    static const std::string MODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<link name=\"base_link\">"
        "  <inertial>"
        "    <mass value=\"2.81\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision name=\"my_collision\">"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"panda_joint0\" type=\"continuous\">"
        "   <axis xyz=\"0 0 1\"/>"
        "   <parent link=\"base_link\"/>"
        "   <child link=\"link_a\"/>"
        "   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
        "</joint>"
        "<link name=\"link_a\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"joint_b\" type=\"fixed\">"
        "  <parent link=\"link_a\"/>"
        "  <child link=\"link_b\"/>"
        "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
        "</joint>"
        "<link name=\"link_b\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"panda_joint1\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
        "    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" "
        "soft_upper_limit=\"0.089\"/>"
        "    <parent link=\"link_b\"/>"
        "    <child link=\"link_c\"/>"
        "    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_c\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"mim_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_c\"/>"
        "    <child link=\"link_d\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
        "  </joint>"
        "  <joint name=\"joint_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_d\"/>"
        "    <child link=\"link_e\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_d\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<link name=\"link_e\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "</robot>";

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom_combined\" type=\"planar\"/>"
        "<group name=\"base_from_joints\">"
        "<joint name=\"base_joint\"/>"
        "<joint name=\"panda_joint0\"/>"
        "<joint name=\"panda_joint1\"/>"
        "</group>"
        "<group name=\"mim_joints\">"
        "<joint name=\"joint_f\"/>"
        "<joint name=\"mim_f\"/>"
        "</group>"
        "<group name=\"base_with_subgroups\">"
        "<group name=\"base_from_base_to_tip\"/>"
        "<joint name=\"panda_joint1\"/>"
        "</group>"
        "<group name=\"base_from_base_to_tip\">"
        "<chain base_link=\"base_link\" tip_link=\"link_b\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"arm\">"
        "<chain base_link=\"base_link\" tip_link=\"link_e\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"base_with_bad_subgroups\">"
        "<group name=\"error\"/>"
        "</group>"
        "</robot>";

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(MODEL2);
    srdf::ModelSharedPtr srdf_model(new srdf::Model());
    srdf_model->initString(*urdf_model, SMODEL2);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

std::size_t generateTestTraj(std::vector<std::shared_ptr<moveit::core::RobotState>>& traj,
                             const moveit::core::RobotModelConstPtr& robot_model_,
                             const moveit::core::JointModelGroup* joint_model_group)
{
  traj.clear();

  std::shared_ptr<moveit::core::RobotState> robot_state(new moveit::core::RobotState(robot_model_));
  robot_state->setToDefaultValues();
  double ja, jc;

  // 3 waypoints with default joints
  for (std::size_t traj_ix = 0; traj_ix < 3; ++traj_ix)
  {
    traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state)));
  }

  ja = robot_state->getVariablePosition("panda_joint0");
  jc = robot_state->getVariablePosition("panda_joint1");

  // 4th waypoint with a small jump of 0.01 in revolute joint and prismatic joint. This should not be considered a jump
  ja = ja - 0.01;
  robot_state->setVariablePosition("panda_joint0", ja);
  jc = jc - 0.01;
  robot_state->setVariablePosition("panda_joint1", jc);
  traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state)));

  // 5th waypoint with a large jump of 1.01 in first revolute joint
  ja = ja + 1.01;
  robot_state->setVariablePosition("panda_joint0", ja);
  traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state)));

  // 6th waypoint with a large jump of 1.01 in first prismatic joint
  jc = jc + 1.01;
  robot_state->setVariablePosition("panda_joint1", jc);
  traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state)));

  // 7th waypoint with no jump
  traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state)));

  return traj.size();
}

TEST_F(OneRobot, testGenerateTrajectory)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("arm");
  std::vector<std::shared_ptr<moveit::core::RobotState>> traj;

  // The full trajectory should be of length 7
  const std::size_t expected_full_traj_len = 7;

  // Generate a test trajectory
  std::size_t full_traj_len = generateTestTraj(traj, robot_model_, joint_model_group);

  // Test the generateTestTraj still generates a trajectory of length 7
  EXPECT_EQ(full_traj_len, expected_full_traj_len);  // full traj should be 7 waypoints long
}

TEST_F(OneRobot, checkAbsoluteJointSpaceJump)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("arm");
  std::vector<std::shared_ptr<moveit::core::RobotState>> traj;

  // A revolute joint jumps 1.01 at the 5th waypoint and a prismatic joint jumps 1.01 at the 6th waypoint
  const std::size_t expected_revolute_jump_traj_len = 4;
  const std::size_t expected_prismatic_jump_traj_len = 5;

  // Pre-compute expected results for tests
  std::size_t full_traj_len = generateTestTraj(traj, robot_model_, joint_model_group);
  const double expected_revolute_jump_fraction = (double)expected_revolute_jump_traj_len / (double)full_traj_len;
  const double expected_prismatic_jump_fraction = (double)expected_prismatic_jump_traj_len / (double)full_traj_len;

  // Container for results
  double fraction;

  // Direct call of absolute version
  fraction = moveit::core::CartesianInterpolator::checkAbsoluteJointSpaceJump(joint_model_group, traj, 1.0, 1.0);
  EXPECT_EQ(expected_revolute_jump_traj_len, traj.size());  // traj should be cut
  EXPECT_NEAR(expected_revolute_jump_fraction, fraction, 0.01);

  // Indirect call using checkJointSpaceJumps
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(1.0, 1.0));
  EXPECT_EQ(expected_revolute_jump_traj_len, traj.size());  // traj should be cut before the revolute jump
  EXPECT_NEAR(expected_revolute_jump_fraction, fraction, 0.01);

  // Test revolute joints
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(1.0, 0.0));
  EXPECT_EQ(expected_revolute_jump_traj_len, traj.size());  // traj should be cut before the revolute jump
  EXPECT_NEAR(expected_revolute_jump_fraction, fraction, 0.01);

  // Test prismatic joints
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(0.0, 1.0));
  EXPECT_EQ(expected_prismatic_jump_traj_len, traj.size());  // traj should be cut before the prismatic jump
  EXPECT_NEAR(expected_prismatic_jump_fraction, fraction, 0.01);

  // Ignore all absolute jumps
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(0.0, 0.0));
  EXPECT_EQ(full_traj_len, traj.size());  // traj should not be cut
  EXPECT_NEAR(1.0, fraction, 0.01);
}

TEST_F(OneRobot, checkRelativeJointSpaceJump)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("arm");
  std::vector<std::shared_ptr<moveit::core::RobotState>> traj;

  // The first large jump of 1.01 occurs at the 5th waypoint so the test should trim the trajectory to length 4
  const std::size_t expected_relative_jump_traj_len = 4;

  // Pre-compute expected results for tests
  std::size_t full_traj_len = generateTestTraj(traj, robot_model_, joint_model_group);
  const double expected_relative_jump_fraction = (double)expected_relative_jump_traj_len / (double)full_traj_len;

  // Container for results
  double fraction;

  // Direct call of relative version: 1.01 > 2.97 * (0.01 * 2 + 1.01 * 2)/6.
  fraction = moveit::core::CartesianInterpolator::checkRelativeJointSpaceJump(joint_model_group, traj, 2.97);
  EXPECT_EQ(expected_relative_jump_traj_len, traj.size());  // traj should be cut before the first jump of 1.01
  EXPECT_NEAR(expected_relative_jump_fraction, fraction, 0.01);

  // Indirect call of relative version using checkJointSpaceJumps
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(2.97));
  EXPECT_EQ(expected_relative_jump_traj_len, traj.size());  // traj should be cut before the first jump of 1.01
  EXPECT_NEAR(expected_relative_jump_fraction, fraction, 0.01);

  // Trajectory should not be cut: 1.01 < 2.98 * (0.01 * 2 + 1.01 * 2)/6.
  generateTestTraj(traj, robot_model_, joint_model_group);
  fraction = moveit::core::CartesianInterpolator::checkJointSpaceJump(joint_model_group, traj,
                                                                      moveit::core::JumpThreshold(2.98));
  EXPECT_EQ(full_traj_len, traj.size());  // traj should not be cut
  EXPECT_NEAR(1.0, fraction, 0.01);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
