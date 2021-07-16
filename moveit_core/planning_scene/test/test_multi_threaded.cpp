/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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

/* Author: Jens Petit */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <gtest/gtest.h>
#include <thread>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection/collision_detector_allocator.h>

const int TRIALS = 1000;
const int THREADS = 2;

bool runCollisionDetection(unsigned int id, unsigned int trials, const planning_scene::PlanningScene* scene,
                           const moveit::core::RobotState* state)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  for (unsigned int i = 0; i < trials; ++i)
  {
    res.clear();
    scene->checkCollision(req, res, *state);
  }
  return res.collision;
}

void runCollisionDetectionAssert(unsigned int id, unsigned int trials, const planning_scene::PlanningScene* scene,
                                 const moveit::core::RobotState* state, bool expected_result)
{
  ASSERT_EQ(expected_result, runCollisionDetection(id, trials, scene, state));
}

class CollisionDetectorThreadedTest : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    ASSERT_TRUE(static_cast<bool>(robot_model_));

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }

  void TearDown() override
  {
  }

  bool robot_model_ok_;

  moveit::core::RobotModelPtr robot_model_;

  collision_detection::CollisionEnvPtr cenv_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  moveit::core::RobotStatePtr robot_state_;

  planning_scene::PlanningScenePtr planning_scene_;
};

/** \brief Tests the FCL collision detector in multiple threads. */
TEST_F(CollisionDetectorThreadedTest, FCLThreaded)
{
  std::vector<moveit::core::RobotStatePtr> states;
  std::vector<std::thread*> threads;
  std::vector<bool> collisions;

  for (unsigned int i = 0; i < THREADS; ++i)
  {
    moveit::core::RobotState* state = new moveit::core::RobotState(planning_scene_->getRobotModel());
    collision_detection::CollisionRequest req;
    state->setToRandomPositions();
    state->update();
    states.push_back(moveit::core::RobotStatePtr(state));
    collisions.push_back(runCollisionDetection(0, 1, planning_scene_.get(), state));
  }

  for (unsigned int i = 0; i < THREADS; ++i)
    threads.push_back(new std::thread(
        std::bind(&runCollisionDetectionAssert, i, TRIALS, planning_scene_.get(), states[i].get(), collisions[i])));

  for (unsigned int i = 0; i < states.size(); ++i)
  {
    threads[i]->join();
    delete threads[i];
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
