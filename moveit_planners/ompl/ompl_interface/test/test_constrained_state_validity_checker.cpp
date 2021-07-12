
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

#include "load_test_robot.h"

#include <limits>
#include <ostream>

#include <gtest/gtest.h>

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

/** \brief This flag sets the verbosity level for the state validity checker. **/
constexpr bool VERBOSE{ false };

constexpr char LOGNAME[] = "test_constrained_state_validity_checker";

/** \brief Pretty print std:vectors **/
std::ostream& operator<<(std::ostream& os, const std::vector<double>& v)
{
  os << "( ";
  for (auto value : v)
    os << value << ", ";
  os << " )";
  return os;
}

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

/** \brief Generic implementation of the tests that can be executed on different robots. **/
class TestStateValidityChecker : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestStateValidityChecker(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
    initial_robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    initial_robot_state_->setToDefaultValues();
  }

  // /***************************************************************************
  //  * START Test implementations
  //  * ************************************************************************/

  void testConstructor()
  {
    ompl::base::StateValidityCheckerPtr checker =
        std::make_shared<ompl_interface::ConstrainedPlanningStateValidityChecker>(planning_context_.get());
  }

  void testJointLimits(const std::vector<double>& position_in_limits)
  {
    // create a validity checker for this test
    auto checker = std::make_shared<ompl_interface::ConstrainedPlanningStateValidityChecker>(planning_context_.get());
    checker->setVerbose(VERBOSE);

    // setup an ompl state with the "position_in_limits" joint values
    robot_state_->setJointGroupPositions(joint_model_group_, position_in_limits);
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

    // cast ompl state to a specific type, useful in the rest of this test
    auto state = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()
                     ->getState()
                     ->as<ompl_interface::ConstrainedPlanningStateSpace::StateType>();

    ROS_DEBUG_STREAM_NAMED(LOGNAME,
                           std::vector<double>(state->values, state->values + joint_model_group_->getVariableCount()));

    // assume the default position in not in self-collision
    // and there are no collision objects of path constraints so this state should be valid
    bool result = checker->isValid(ompl_state.get());
    EXPECT_TRUE(result);

    // do the same for the version with distance
    double distance{ 0.0 };
    result = checker->isValid(ompl_state.get(), distance);

    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Distance from the isValid function: " << distance);
    EXPECT_TRUE(result);
    EXPECT_GT(distance, 0.0);

    // move first joint obviously outside any joint limits
    state->values[0] = std::numeric_limits<double>::max();
    state->clearKnownInformation();  // make sure the validity checker does not use the cached value

    ROS_DEBUG_STREAM_NAMED(LOGNAME,
                           std::vector<double>(state->values, state->values + joint_model_group_->getVariableCount()));

    bool result_2 = checker->isValid(ompl_state.get());
    EXPECT_FALSE(result_2);

    // do the same for the version with distance
    double distance_2{ 0.0 };
    result_2 = checker->isValid(ompl_state.get(), distance_2);
    EXPECT_FALSE(result_2);
    // isValid function returned false before any distance calculation is done
    EXPECT_EQ(distance_2, 0.0);
  }

  void testSelfCollision(const std::vector<double>& position_in_self_collision)
  {
    // create a validity checker for this test
    auto checker = std::make_shared<ompl_interface::ConstrainedPlanningStateValidityChecker>(planning_context_.get());
    checker->setVerbose(VERBOSE);

    robot_state_->setJointGroupPositions(joint_model_group_, position_in_self_collision);

    // use a scoped OMPL state so we don't have to call allocState and freeState
    // (as recommended in the OMPL documantion)
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

    // The code below is just to print the state to the debug stream
    auto state = ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()
                     ->getState()
                     ->as<ompl_interface::ConstrainedPlanningStateSpace::StateType>();

    // ompl_state.reals() throws a segmentation fault for this state type
    // use a more involved conversion to std::vector for logging
    ROS_DEBUG_STREAM_NAMED(LOGNAME,
                           std::vector<double>(state->values, state->values + joint_model_group_->getVariableCount()));

    // the given state is known to be in self-collision, we check it here
    bool result = checker->isValid(ompl_state.get());
    EXPECT_FALSE(result);

    // do the same for the version with distance
    double distance{ 0.0 };
    result = checker->isValid(ompl_state.get(), distance);
    EXPECT_FALSE(result);

    // but it should respect the joint limits
    bool result_2 = robot_state_->satisfiesBounds();
    EXPECT_TRUE(result_2);
  }

  // /***************************************************************************
  //  * END Test implementation
  //  * ************************************************************************/

protected:
  void SetUp() override
  {
    // setup all the input we need to create a StateValidityChecker
    setupStateSpace();
    setupPlanningContext();
  };

  void TearDown() override
  {
  }

  void setupStateSpace()
  {
    // note: make_shared throws if the allocations below fail, making the test fail when necessary
    ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_, group_name_);
    state_space_ = std::make_shared<ompl_interface::ConstrainedPlanningStateSpace>(space_spec);
    state_space_->computeLocations();  // this gets called in the state space factory normally

    auto dummy_constraint = std::make_shared<DummyConstraint>(num_dofs_);
    constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(state_space_, dummy_constraint);
  }

  void setupPlanningContext()
  {
    ASSERT_NE(state_space_, nullptr) << "Initialize state space before creating the planning context.";
    planning_context_spec_.state_space_ = state_space_;

    // IMPORTANT, we need to create the simple setup with  a ConstrainedSpaceInformation object,
    // not with the state space, as this will allocate the wrong type of SpaceInformation
    auto csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
    planning_context_spec_.ompl_simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(csi);

    // check if the we succeeded in the comment above
    auto si = planning_context_spec_.ompl_simple_setup_->getSpaceInformation();
    auto si_constrained = dynamic_cast<ompl::base::ConstrainedSpaceInformation*>(si.get());
    ASSERT_NE(si_constrained, nullptr);

    planning_context_ =
        std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name_, planning_context_spec_);

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_context_->setPlanningScene(planning_scene_);
    planning_context_->setCompleteInitialState(*initial_robot_state_);

    ROS_DEBUG_NAMED(LOGNAME, "Planning context with name %s is ready (but not configured).",
                    planning_context_->getName().c_str());
  }

  robot_state::RobotStatePtr initial_robot_state_;

  ompl_interface::ModelBasedStateSpacePtr state_space_;
  ompl_interface::ModelBasedPlanningContextSpecification planning_context_spec_;
  ompl_interface::ModelBasedPlanningContextPtr planning_context_;
  planning_scene::PlanningScenePtr planning_scene_;

  ompl::base::ConstrainedStateSpacePtr constrained_state_space_;
};

// /***************************************************************************
//  * Run all tests on the Panda robot
//  * ************************************************************************/
class PandaValidityCheckerTests : public TestStateValidityChecker
{
protected:
  PandaValidityCheckerTests() : TestStateValidityChecker("panda", "panda_arm")
  {
  }
};

TEST_F(PandaValidityCheckerTests, testConstructor)
{
  testConstructor();
}

TEST_F(PandaValidityCheckerTests, testJointLimits)
{
  // use the panda "ready" state from the srdf config
  // we know this state should be within limits and self-collision free
  testJointLimits({ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 });
}

TEST_F(PandaValidityCheckerTests, testSelfCollision)
{
  // the given state has self collision between "hand" and "panda_link2"
  // (I just tried a couple of random states until I found one that collided.)
  testSelfCollision({ 2.31827, -0.169668, 2.5225, -2.98568, -0.36355, 0.808339, 0.0843406 });
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucTestStateValidityChecker : public TestStateValidityChecker
{
protected:
  FanucTestStateValidityChecker() : TestStateValidityChecker("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucTestStateValidityChecker, createStateValidityChecker)
{
  testConstructor();
}

TEST_F(FanucTestStateValidityChecker, testJointLimits)
{
  // I assume the Fanucs's zero state is within limits and self-collision free
  testJointLimits({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(FanucTestStateValidityChecker, testSelfCollision)
{
  // the given state has self collision between "base_link" and "link_5"
  // (I just tried a couple of random states until I found one that collided.)
  testSelfCollision({ -2.95993, -0.682185, -2.43873, -0.939784, 3.0544, 0.882294 });
}

/* (Note: the PR2 has no collision geometry in the moveit_resources package.) */

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
