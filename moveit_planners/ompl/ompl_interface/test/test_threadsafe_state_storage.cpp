/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021
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
 *   * Neither the name of the authors nor the names of its
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

/* Author: Pradeep Rajendran */
/*
This test checks if a state can be set in TSSafeStateStorage and correctly retrieved.
The skeleton of this test was taken from test_state_validity_checker.cpp by Jeroen De Maeyer.
*/

#include "load_test_robot.h"
#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <gtest/gtest.h>

/** \brief This flag sets the verbosity level for the state validity checker. **/
constexpr bool VERBOSE = false;

constexpr char LOGNAME[] = "test_threadsafe_state_storage";

/** \brief Generic implementation of the tests that can be executed on different robots. **/
class TestThreadSafeStateStorage : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestThreadSafeStateStorage(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
  }

  /** This test if a state is correctly set in TSStateStorage. State is read back and compared with the original state **/
  void testReadback(const std::vector<double>& position_in_limits)
  {
    SCOPED_TRACE("testConstruction");

    // Set the robot_state_ to the given position
    robot_state_->setJointGroupPositions(joint_model_group_, position_in_limits);

    // Construct the TSStateStorage using the constructor taking the robot state
    ompl_interface::TSStateStorage const tss(*robot_state_);

    // Readback the stored state
    auto robot_state_stored = tss.getStateStorage();

    // Check if robot_state_stored's joint angles matches with what we set
    for (auto const& joint_name : robot_state_->getVariableNames())
    {
      auto const expected_value = robot_state_->getVariablePosition(joint_name);
      auto const actual_value = robot_state_stored->getVariablePosition(joint_name);
      EXPECT_EQ(actual_value, expected_value) << "Expecting joint value for " << joint_name << " to match.";
    }
  }

protected:
  void SetUp() override
  {
  }
};

// /***************************************************************************
//  * Run all tests on the Panda robot
//  * ************************************************************************/
class PandaTest : public TestThreadSafeStateStorage
{
protected:
  PandaTest() : TestThreadSafeStateStorage("panda", "panda_arm")
  {
  }
};

TEST_F(PandaTest, testConstruction)
{
  testReadback({ 0., -0.785, 0., -2.356, 0., 1.571, 0.785 });
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucTest : public TestThreadSafeStateStorage
{
protected:
  FanucTest() : TestThreadSafeStateStorage("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucTest, testConstructor)
{
  testReadback({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
