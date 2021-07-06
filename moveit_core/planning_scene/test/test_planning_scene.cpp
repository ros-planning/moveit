/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/message_checks.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <ros/package.h>

TEST(PlanningScene, LoadRestore)
{
  urdf::ModelInterfaceSharedPtr urdf_model = moveit::core::loadModelInterface("pr2");
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  planning_scene::PlanningScene ps(urdf_model, srdf_model);
  moveit_msgs::PlanningScene ps_msg;
  ps.getPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps.getName(), ps_msg.name);
  EXPECT_EQ(ps.getRobotModel()->getName(), ps_msg.robot_model_name);
  ps.setPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps.getName(), ps_msg.name);
  EXPECT_EQ(ps.getRobotModel()->getName(), ps_msg.robot_model_name);
}

TEST(PlanningScene, LoadRestoreDiff)
{
  urdf::ModelInterfaceSharedPtr urdf_model = moveit::core::loadModelInterface("pr2");
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  auto ps = std::make_shared<planning_scene::PlanningScene>(urdf_model, srdf_model);

  collision_detection::World& world = *ps->getWorldNonConst();

  /* add one object to ps's world */
  Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  /* ps can be written to and set from message */
  moveit_msgs::PlanningScene ps_msg;
  ps_msg.robot_state.is_diff = true;
  EXPECT_TRUE(moveit::core::isEmpty(ps_msg));
  ps->getPlanningSceneMsg(ps_msg);
  ps->setPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps_msg.world.collision_objects.size(), 1u);
  EXPECT_EQ(ps_msg.world.collision_objects[0].id, "sphere");
  EXPECT_TRUE(world.hasObject("sphere"));

  /* test diff scene on top of ps */
  planning_scene::PlanningScenePtr next = ps->diff();
  /* world is inherited from ps */
  EXPECT_TRUE(next->getWorld()->hasObject("sphere"));

  /* object in overlay is only added in overlay */
  next->getWorldNonConst()->addToObject("sphere_in_next_only", shapes::ShapeConstPtr(new shapes::Sphere(0.5)), id);
  EXPECT_EQ(next->getWorld()->size(), 2u);
  EXPECT_EQ(ps->getWorld()->size(), 1u);

  /* the worlds used for collision detection contain one and two objects, respectively */
  EXPECT_EQ(ps->getCollisionEnv()->getWorld()->size(), 1u);
  EXPECT_EQ(ps->getCollisionEnvUnpadded()->getWorld()->size(), 1u);

  EXPECT_EQ(next->getCollisionEnv()->getWorld()->size(), 2u);
  EXPECT_EQ(next->getCollisionEnvUnpadded()->getWorld()->size(), 2u);

  /* maintained diff contains only overlay object */
  next->getPlanningSceneDiffMsg(ps_msg);
  EXPECT_EQ(ps_msg.world.collision_objects.size(), 1u);

  /* copy ps to next and apply diff */
  next->decoupleParent();
  moveit_msgs::PlanningScene ps_msg2;

  /* diff is empty now */
  next->getPlanningSceneDiffMsg(ps_msg2);
  EXPECT_EQ(ps_msg2.world.collision_objects.size(), 0u);

  /* next's world contains both objects */
  next->getPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps_msg.world.collision_objects.size(), 2u);
  ps->setPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps->getWorld()->size(), 2u);
  EXPECT_EQ(ps->getCollisionEnv()->getWorld()->size(), 2u);
  EXPECT_EQ(ps->getCollisionEnvUnpadded()->getWorld()->size(), 2u);
}

TEST(PlanningScene, MakeAttachedDiff)
{
  urdf::ModelInterfaceSharedPtr urdf_model = moveit::core::loadModelInterface("pr2");
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  auto ps = std::make_shared<planning_scene::PlanningScene>(urdf_model, srdf_model);

  /* add a single object to ps's world */
  collision_detection::World& world = *ps->getWorldNonConst();
  Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  /* attach object in diff */
  planning_scene::PlanningScenePtr attached_object_diff_scene = ps->diff();

  moveit_msgs::AttachedCollisionObject att_obj;
  att_obj.link_name = "r_wrist_roll_link";
  att_obj.object.operation = moveit_msgs::CollisionObject::ADD;
  att_obj.object.id = "sphere";
  attached_object_diff_scene->processAttachedCollisionObjectMsg(att_obj);

  /* object is not in world anymore */
  EXPECT_EQ(attached_object_diff_scene->getWorld()->size(), 0u);
  /* it became part of the robot state though */
  EXPECT_TRUE(attached_object_diff_scene->getCurrentState().hasAttachedBody("sphere"));

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  attached_object_diff_scene->checkCollision(req, res);
  ps->checkCollision(req, res);
}

TEST(PlanningScene, isStateValid)
{
  moveit::core::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("pr2");
  auto ps = std::make_shared<planning_scene::PlanningScene>(robot_model->getURDF(), robot_model->getSRDF());
  moveit::core::RobotState current_state = ps->getCurrentState();
  if (ps->isStateColliding(current_state, "left_arm"))
  {
    EXPECT_FALSE(ps->isStateValid(current_state, "left_arm"));
  }
}

TEST(PlanningScene, loadGoodSceneGeometry)
{
  moveit::core::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("pr2");
  auto ps = std::make_shared<planning_scene::PlanningScene>(robot_model->getURDF(), robot_model->getSRDF());

  std::istringstream good_scene_geometry;
  good_scene_geometry.str("foobar_scene\n"
                          "* foo\n"
                          "0 0 0\n"
                          "0 0 0 1\n"
                          "1\n"
                          "box\n"
                          "2.58 1.36 0.31\n"
                          "1.49257 1.00222 0.170051\n"
                          "0 0 4.16377e-05 1\n"
                          "0 0 1 0.3\n"
                          "0\n"
                          "* bar\n"
                          "0 0 0\n"
                          "0 0 0 1\n"
                          "1\n"
                          "cylinder\n"
                          "0.02 0.0001\n"
                          "0.453709 0.499136 0.355051\n"
                          "0 0 4.16377e-05 1\n"
                          "1 0 0 1\n"
                          "0\n"
                          ".\n");
  EXPECT_TRUE(ps->loadGeometryFromStream(good_scene_geometry));
  EXPECT_EQ(ps->getName(), "foobar_scene");
  EXPECT_TRUE(ps->getWorld()->hasObject("foo"));
  EXPECT_TRUE(ps->getWorld()->hasObject("bar"));
  EXPECT_FALSE(ps->getWorld()->hasObject("baz"));  // Sanity check.
}

TEST(PlanningScene, loadBadSceneGeometry)
{
  moveit::core::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("pr2");
  auto ps = std::make_shared<planning_scene::PlanningScene>(robot_model->getURDF(), robot_model->getSRDF());
  std::istringstream empty_scene_geometry;

  // This should fail since there is no planning scene name and no end of geometry marker.
  EXPECT_FALSE(ps->loadGeometryFromStream(empty_scene_geometry));

  std::istringstream malformed_scene_geometry;
  malformed_scene_geometry.str("malformed_scene_geometry\n"
                               "* foo\n"
                               "1\n"
                               "box\n"
                               "2.58 1.36\n" /* Only two tokens; should be 3 */
                               "1.49257 1.00222 0.170051\n"
                               "0 0 4.16377e-05 1\n"
                               "0 0 1 0.3\n"
                               ".\n");
  EXPECT_FALSE(ps->loadGeometryFromStream(malformed_scene_geometry));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
