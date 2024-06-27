/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Universitaet Hamburg
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

/* Author: Rafael A. Rojas*/

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene/collision_object.h>
#include <moveit/utils/message_checks.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>
#include <string>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

/** Test if a urdf is loaded as collision object correctly. This test loads a
 * fanuc model and check if the first link was loaded correctly as a collition
 * object
 **/
TEST(URDF_TO_COLLISION_OBJECT, LoadFromFile)
{
  ros::Time::init();
  std::string path = ros::package::getPath("moveit_resources_fanuc_description") + "/urdf/fanuc.urdf";
  urdf::Model urdf_model;

  urdf_model.initFile(path);
  moveit_msgs::CollisionObject collision_object = planning_scene::urdf_to_collision_object(urdf_model);

  KDL::Tree tree;
  KDL::Chain chain;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  tree.getChain("base_link", "tool0", chain);

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  KDL::JntArray joint_array(6);
  joint_array.data = Eigen::VectorXd::Zero(6);
  KDL::Frame frame;

  for (std::size_t i = 0; i < 7; i++)
  {
    fk_solver.JntToCart(joint_array, frame, i);
    EXPECT_NEAR(collision_object.mesh_poses[i].position.x, frame.p.x(), 0.0001) << "link " << i;
    EXPECT_NEAR(collision_object.mesh_poses[i].position.y, frame.p.y(), 0.0001) << "link " << i;
    EXPECT_NEAR(collision_object.mesh_poses[i].position.z, frame.p.z(), 0.0001) << "link " << i;
  }
  EXPECT_TRUE(collision_object.primitives.empty());
  EXPECT_EQ(collision_object.meshes.size(), 7u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
