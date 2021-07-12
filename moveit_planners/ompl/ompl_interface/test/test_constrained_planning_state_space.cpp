/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer */

/** This file checks if the ConstrainedPlanningStateSpace can properly copy OMPL States of type
 * ompl::base::ConstrainedStateSpace::StateType into MoveIt's robot state.
 **/

#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>

#include <memory>
#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include "load_test_robot.h"

constexpr char LOGNAME[] = "test_constrained_state_validity_checker";

/** \brief Dummy constraint for testing, always satisfied. We need this to create and OMPL ConstrainedStateSpace. **/
class DummyConstraint : public ompl::base::Constraint
{
public:
  DummyConstraint(const unsigned int num_dofs) : ompl::base::Constraint(num_dofs, 1)
  {
  }
  void function(const Eigen::Ref<const Eigen::VectorXd>& /*unused*/, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    out[0] = 0.0;
  }
};

/** \brief Robot indepentent implementation of the tests.  **/
class TestConstrainedStateSpace : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
protected:
  TestConstrainedStateSpace(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
  }

  void SetUp() override
  {
    // create a constrained state space for testing
    setupMoveItStateSpace();
    setupOMPLStateSpace();
  };

  void TearDown() override
  {
  }

  void setupMoveItStateSpace()
  {
    ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_, group_name_);
    moveit_state_space_ = std::make_shared<ompl_interface::ConstrainedPlanningStateSpace>(space_spec);
  }

  void setupOMPLStateSpace()
  {
    // first call setupMoveItStateSpace()
    assert(moveit_state_space_ != nullptr);
    auto con = std::make_shared<DummyConstraint>(num_dofs_);
    constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(moveit_state_space_, con);
  }

  void testGetValueAddressAtIndex()
  {
    Eigen::VectorXd joint_positions = getDeterministicState();
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    auto state_ptr = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
    double* out_joint_positions = dynamic_cast<ompl_interface::ModelBasedStateSpace::StateType*>(state_ptr)->values;
    EXPECT_FALSE(out_joint_positions == nullptr);
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      *(out_joint_positions + i) = joint_positions[i];
    }

    // check getValueAddressAtIndex
    // it can only be called with an already unwrapped state,
    // this unwrapping is either done in the constrained_state_space_ (see WrapperStateSpace in OMPL),
    // or in copyJointToOMPLState in the implementation of ConstrainedPlanningStateSpace in MoveIt.
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      EXPECT_EQ(joint_positions[i], *(constrained_state_space_->getValueAddressAtIndex(ompl_state.get(), i)));
    }
  }

  void testCopyToRobotState()
  {
    // create and OMPL state
    // The copy operation of the ConstraintStateSpace::StateType show below is not supported
    // ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(joint_positions);
    // Because the state spaces implemented in MoveIt do not support casting to Eigen::VectorXd
    Eigen::VectorXd joint_positions = getDeterministicState();
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    auto state_ptr = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
    double* out_joint_positions = dynamic_cast<ompl_interface::ModelBasedStateSpace::StateType*>(state_ptr)->values;
    EXPECT_FALSE(out_joint_positions == nullptr);
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      *(out_joint_positions + i) = joint_positions[i];
    }

    // copy into a MoveIt state
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state_space_->copyToRobotState(moveit_state, ompl_state.get());

    // check if copy worked out as expected
    Eigen::VectorXd out_joint_position(num_dofs_);
    moveit_state.copyJointGroupPositions(joint_model_group_, out_joint_position);

    EXPECT_EQ(joint_positions.size(), out_joint_position.size());
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      EXPECT_EQ(joint_positions[i], out_joint_position[i]);
    }
  }

  void testCopyToOMPLState()
  {
    // create a MoveIt state
    Eigen::VectorXd joint_positions = getDeterministicState();
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state.setJointGroupPositions(joint_model_group_, joint_positions);

    // copy into an OMPL state
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    moveit_state_space_->copyToOMPLState(ompl_state.get(), moveit_state);

    // check if copy worked out as expected
    // (Again, support for casting to Eigen::VectorXd would have been nice here.)
    auto state_ptr = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
    double* out_joint_positions = dynamic_cast<ompl_interface::ModelBasedStateSpace::StateType*>(state_ptr)->values;
    EXPECT_FALSE(out_joint_positions == nullptr);
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      EXPECT_EQ(joint_positions[i], *(out_joint_positions + i));
    }
  }

  void testCopyJointToOMPLState()
  {
    EXPECT_TRUE(true);
    // create a MoveIt state
    Eigen::VectorXd joint_positions = getDeterministicState();
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state.setJointGroupPositions(joint_model_group_, joint_positions);

    // copy into an OMPL state, one index at a time
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    auto joint_model_names = joint_model_group_->getActiveJointModelNames();
    for (std::size_t joint_index = 0; joint_index < num_dofs_; ++joint_index)
    {
      const moveit::core::JointModel* joint_model = joint_model_group_->getJointModel(joint_model_names[joint_index]);
      EXPECT_FALSE(joint_model == nullptr);

      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Joint model: " << joint_model->getName() << " index: " << joint_index);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "first index: " << joint_model->getFirstVariableIndex() * sizeof(double));
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "width: " << joint_model->getVariableCount() * sizeof(double));

      moveit_state_space_->copyJointToOMPLState(ompl_state.get(), moveit_state, joint_model, joint_index);
    }

    // check if copy worked out as expected
    auto state_ptr = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
    double* out_joint_positions = dynamic_cast<ompl_interface::ModelBasedStateSpace::StateType*>(state_ptr)->values;
    EXPECT_FALSE(out_joint_positions == nullptr);
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      EXPECT_EQ(joint_positions[i], *(out_joint_positions + i));
    }
  }

protected:
  ompl::base::ConstrainedStateSpacePtr constrained_state_space_;
  ompl_interface::ConstrainedPlanningStateSpacePtr moveit_state_space_;
};

// /***************************************************************************
//  * Run all tests on the Panda robot
//  * ************************************************************************/
class PandaCopyStateTest : public TestConstrainedStateSpace
{
protected:
  PandaCopyStateTest() : TestConstrainedStateSpace("panda", "panda_arm")
  {
  }
};

TEST_F(PandaCopyStateTest, testGetValueAddressAtIndex)
{
  testGetValueAddressAtIndex();
}

TEST_F(PandaCopyStateTest, testCopyToRobotState)
{
  testCopyToRobotState();
}

TEST_F(PandaCopyStateTest, testCopyToOMPLState)
{
  testCopyToOMPLState();
}

TEST_F(PandaCopyStateTest, testCopyJointToOMPLState)
{
  testCopyJointToOMPLState();
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucCopyStateTest : public TestConstrainedStateSpace
{
protected:
  FanucCopyStateTest() : TestConstrainedStateSpace("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucCopyStateTest, testGetValueAddressAtIndex)
{
  testGetValueAddressAtIndex();
}

TEST_F(FanucCopyStateTest, testCopyToRobotState)
{
  testCopyToRobotState();
}

TEST_F(FanucCopyStateTest, testCopyToOMPLState)
{
  testCopyToOMPLState();
}

TEST_F(FanucCopyStateTest, testCopyJointToOMPLState)
{
  testCopyJointToOMPLState();
}

// /***************************************************************************
//  * Run all tests on the PR2 robot its left arm
//  * ************************************************************************/
class PR2CopyStateTest : public TestConstrainedStateSpace
{
protected:
  PR2CopyStateTest() : TestConstrainedStateSpace("pr2", "left_arm")
  {
  }
};

TEST_F(PR2CopyStateTest, testGetValueAddressAtIndex)
{
  testGetValueAddressAtIndex();
}

TEST_F(PR2CopyStateTest, testCopyToRobotState)
{
  testCopyToRobotState();
}

TEST_F(PR2CopyStateTest, testCopyToOMPLState)
{
  testCopyToOMPLState();
}

TEST_F(PR2CopyStateTest, testCopyJointToOMPLState)
{
  testCopyJointToOMPLState();
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
